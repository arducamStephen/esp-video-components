/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "protocol_examples_common.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "lwip/apps/netbiosns.h"
#include "example_video_common.h"
#include "esp_rom_sys.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ArducamIMX500SDK.h"
#include "driver/ppa.h"
#include "app_drawing_utils.h"


#define SPI_MAX_DMA_BYTES        4096
#define SPI_DUMMY_BYTE           0xFF
#define MAX_DATA_R_BUF_SIZE      2 * 1024 * 1024
#define SPI_HOST                 SPI2_HOST
#define PIN_NUM_MOSI             GPIO_NUM_3
#define PIN_NUM_MISO             GPIO_NUM_2
#define PIN_NUM_CLK              GPIO_NUM_5
#define PIN_NUM_CS               GPIO_NUM_4

// bus
static i2c_master_bus_handle_t g_bus_handle;
static i2c_master_dev_handle_t g_client_handle;
static spi_device_handle_t g_spi_dev;
// dev
esp_err_t init_isp_dev(int cam_fd);
void init_detection_result_buf(void);
void init_spi_dev(void);
void init_i2c_dev(void);
esp_err_t i2c_write_reg16_u32(uint16_t reg, uint32_t value);
esp_err_t i2c_read_reg16_u32(uint16_t reg,uint32_t *out);
int32_t spi_read(uint8_t *buf, uint32_t size);

#define EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER  CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER
#define EXAMPLE_JPEG_ENC_QUALITY            CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY
#define EXAMPLE_MDNS_INSTANCE               CONFIG_EXAMPLE_MDNS_INSTANCE
#define EXAMPLE_MDNS_HOST_NAME              CONFIG_EXAMPLE_MDNS_HOST_NAME
#define EXAMPLE_PART_BOUNDARY               CONFIG_EXAMPLE_HTTP_PART_BOUNDARY

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" EXAMPLE_PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t loading_jpg_gz_start[] asm("_binary_loading_jpg_gz_start");
extern const uint8_t loading_jpg_gz_end[] asm("_binary_loading_jpg_gz_end");
extern const uint8_t favicon_ico_gz_start[] asm("_binary_favicon_ico_gz_start");
extern const uint8_t favicon_ico_gz_end[] asm("_binary_favicon_ico_gz_end");
extern const uint8_t assets_index_js_gz_start[] asm("_binary_index_js_gz_start");
extern const uint8_t assets_index_js_gz_end[] asm("_binary_index_js_gz_end");
extern const uint8_t assets_index_css_gz_start[] asm("_binary_index_css_gz_start");
extern const uint8_t assets_index_css_gz_end[] asm("_binary_index_css_gz_end");

/**
 * @brief Web cam control structure
 */
typedef struct web_cam_video {
    int fd;
    uint8_t index;

    example_encoder_handle_t encoder_handle;
    uint8_t *jpeg_out_buf;
    uint32_t jpeg_out_size;

    uint8_t *buffer[EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_size;

    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t jpeg_quality;

    uint32_t frame_rate;

    SemaphoreHandle_t sem;

    uint32_t support_control_jpeg_quality   : 1;
} web_cam_video_t;

typedef struct web_cam {
    uint8_t video_count;
    web_cam_video_t video[0];
} web_cam_t;

typedef struct web_cam_video_config {
    const char *dev_name;
    uint32_t buffer_count;
} web_cam_video_config_t;

typedef struct request_desc {
    int index;
} request_desc_t;

static const char *TAG = "app";
static QueueHandle_t metadata_queue;
static uint8_t *metadata_buf;

typedef struct metadata_frame {
    uint32_t data_size;
} metadata_frame_t;

static bool is_valid_web_cam(web_cam_video_t *video)
{
    return video->fd != -1;
}

static esp_err_t decode_request(web_cam_t *web_cam, httpd_req_t *req, request_desc_t *desc)
{
    esp_err_t ret;
    int index = -1;
    char buffer[32];

    if ((ret = httpd_req_get_url_query_str(req, buffer, sizeof(buffer))) != ESP_OK) {
        return ret;
    }
    ESP_LOGD(TAG, "source: %s", buffer);

    for (int i = 0; i < web_cam->video_count; i++) {
        char source_str[16];

        if (snprintf(source_str, sizeof(source_str), "source=%d", i) <= 0) {
            return ESP_FAIL;
        }

        if (strcmp(buffer, source_str) == 0) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    desc->index = index;
    return ESP_OK;
}

static esp_err_t capture_video_image(httpd_req_t *req, web_cam_video_t *video, bool is_jpeg)
{
    esp_err_t ret;
    struct v4l2_buffer buf;
    const char *type_str = is_jpeg ? "JPEG" : "binary";
    uint32_t jpeg_encoded_size;

    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), TAG, "failed to receive video frame");
    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!is_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
        /* Directly send the buffer of raw data */
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->buffer[buf.index], buf.bytesused), fail0, TAG, "failed to send %s", type_str);
        jpeg_encoded_size = buf.bytesused;
    } else {
        ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take semaphore");
        ret = example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                                      video->jpeg_out_buf, video->jpeg_out_size, &jpeg_encoded_size);
        xSemaphoreGive(video->sem);
        ESP_GOTO_ON_ERROR(ret, fail0, TAG, "failed to encode video frame");
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->jpeg_out_buf, jpeg_encoded_size), fail0, TAG, "failed to send %s", type_str);
    }

    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");

    ESP_GOTO_ON_ERROR(httpd_resp_sendstr_chunk(req, NULL), fail0, TAG, "failed to send null");

    ESP_LOGD(TAG, "send %s image%d size: %" PRIu32, type_str, video->index, jpeg_encoded_size);

    return ESP_OK;

fail0:
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    return ret;
}

static char *get_cameras_json(web_cam_t *web_cam)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *cameras = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "cameras", cameras);

    for (int i = 0; i < web_cam->video_count; i++) {
        char src_str[32];

        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "index", i);
        assert(snprintf(src_str, sizeof(src_str), ":%d/stream", i + 81) > 0);
        cJSON_AddStringToObject(camera, "src", src_str);
        cJSON_AddNumberToObject(camera, "currentFrameRate", web_cam->video[i].frame_rate);
        cJSON_AddNumberToObject(camera, "currentImageFormat", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(camera, "currentImageFormatDescription", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON_AddNumberToObject(camera, "currentQuality", web_cam->video[i].jpeg_quality);
        }

        cJSON *current_resolution = cJSON_CreateObject();
        cJSON_AddNumberToObject(current_resolution, "width", web_cam->video[i].width);
        cJSON_AddNumberToObject(current_resolution, "height", web_cam->video[i].height);
        cJSON_AddItemToObject(camera, "currentResolution", current_resolution);

        cJSON *image_formats = cJSON_CreateArray();
        cJSON *image_format = cJSON_CreateObject();
        cJSON_AddNumberToObject(image_format, "id", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(image_format, "description", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON *image_format_quality = cJSON_CreateObject();

            int min_quality = 1;
            int max_quality = 100;
            int step_quality = 1;
            int default_quality = EXAMPLE_JPEG_ENC_QUALITY;
            if (web_cam->video[i].pixel_format == V4L2_PIX_FMT_JPEG) {
                struct v4l2_query_ext_ctrl qctrl = {0};

                qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
                if (ioctl(web_cam->video[i].fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
                    min_quality = qctrl.minimum;
                    max_quality = qctrl.maximum;
                    step_quality = qctrl.step;
                    default_quality = qctrl.default_value;
                }
            }

            cJSON_AddNumberToObject(image_format_quality, "min", min_quality);
            cJSON_AddNumberToObject(image_format_quality, "max", max_quality);
            cJSON_AddNumberToObject(image_format_quality, "step", step_quality);
            cJSON_AddNumberToObject(image_format_quality, "default", default_quality);
            cJSON_AddItemToObject(image_format, "quality", image_format_quality);
        }
        cJSON_AddItemToArray(image_formats, image_format);

        cJSON_AddItemToObject(camera, "imageFormats", image_formats);
        cJSON_AddItemToArray(cameras, camera);
    }

    char *output = cJSON_Print(root);
    cJSON_Delete(root);
    return output;
}

static esp_err_t set_camera_jpeg_quality(web_cam_video_t *video, int quality)
{
    esp_err_t ret = ESP_OK;
    int quality_reset = quality;

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        struct v4l2_query_ext_ctrl qctrl = {0};

        qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
        if (ioctl(video->fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
            if ((quality > qctrl.maximum) || (quality < qctrl.minimum) ||
                    (((quality - qctrl.minimum) % qctrl.step) != 0)) {

                if (quality > qctrl.maximum) {
                    quality_reset = qctrl.maximum;
                } else if (quality < qctrl.minimum) {
                    quality_reset = qctrl.minimum;
                } else {
                    quality_reset = qctrl.minimum + ((quality - qctrl.minimum) / qctrl.step) * qctrl.step;
                }

                ESP_LOGW(TAG, "video%d: JPEG compression quality=%d is out of sensor's range, reset to %d", video->index, quality, quality_reset);
            }

            controls.ctrl_class = V4L2_CID_JPEG_CLASS;
            controls.count = 1;
            controls.controls = control;
            control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
            control[0].value = quality_reset;
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_S_EXT_CTRLS, &controls), TAG, "failed to set jpeg compression quality");

            video->jpeg_quality = quality_reset;
            video->support_control_jpeg_quality = 1;
        } else {
            video->support_control_jpeg_quality = 0;
            ESP_LOGW(TAG, "video%d: JPEG compression quality control is not supported", video->index);
        }
    } else {
        ESP_RETURN_ON_ERROR(example_encoder_set_jpeg_quality(video->encoder_handle, quality_reset), TAG, "failed to set jpeg quality");
        video->jpeg_quality = quality_reset;
    }

    if (video->support_control_jpeg_quality) {
        ESP_LOGI(TAG, "video%d: set jpeg quality %d success", video->index, quality_reset);
    }

    return ret;
}

static esp_err_t camera_info_handler(httpd_req_t *req)
{
    esp_err_t ret;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    char *output = get_cameras_json(web_cam);

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);

    return ret;
}

static esp_err_t camera_settings_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char *content;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    content = (char *)calloc(1, req->content_len + 1);
    ESP_RETURN_ON_FALSE(content, ESP_ERR_NO_MEM, TAG, "failed to allocate memory");

    ESP_GOTO_ON_FALSE(httpd_req_recv(req, content, req->content_len) > 0, ESP_FAIL, fail0, TAG, "failed to recv content");
    ESP_LOGD(TAG, "content: %s", content);

    cJSON *json_root = cJSON_Parse(content);
    free(content);
    content = NULL;
    ESP_GOTO_ON_FALSE(json_root, ESP_FAIL, fail0, TAG, "failed to parse JSON");

    cJSON *json_index = cJSON_GetObjectItem(json_root, "index");
    ESP_GOTO_ON_FALSE(json_index && cJSON_IsNumber(json_index), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid index field");
    int index = json_index->valueint;
    ESP_GOTO_ON_FALSE(index >= 0 && index < web_cam->video_count && is_valid_web_cam(&web_cam->video[index]), ESP_ERR_INVALID_ARG, fail1, TAG, "invalid index");

    cJSON *json_image_format = cJSON_GetObjectItem(json_root, "image_format");
    ESP_GOTO_ON_FALSE(json_image_format && cJSON_IsNumber(json_image_format), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid image_format field");
    int image_format = json_image_format->valueint;

    cJSON *json_jpeg_quality = cJSON_GetObjectItem(json_root, "jpeg_quality");
    ESP_GOTO_ON_FALSE(json_jpeg_quality && cJSON_IsNumber(json_jpeg_quality), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid jpeg_quality field");
    int jpeg_quality = json_jpeg_quality->valueint;

    ESP_LOGI(TAG, "JSON parse success - index:%d, image_format:%d, jpeg_quality:%d", index, image_format, jpeg_quality);
    cJSON_Delete(json_root);
    json_root = NULL;

    ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(&web_cam->video[index], jpeg_quality), fail1, TAG, "failed to set camera jpeg quality");

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail1:
    if (json_root) {
        cJSON_Delete(json_root);
    }
fail0:
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
    }
    if (content) {
        free(content);
    }
    return ret;
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    /* Route to appropriate static file based on URI */
    if (strcmp(uri, "/") == 0) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    } else if (strcmp(uri, "/loading.jpg") == 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)loading_jpg_gz_start, loading_jpg_gz_end - loading_jpg_gz_start);
    } else if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)favicon_ico_gz_start, favicon_ico_gz_end - favicon_ico_gz_start);
    } else if (strcmp(uri, "/assets/index.js") == 0) {
        httpd_resp_set_type(req, "application/javascript");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_js_gz_start, assets_index_js_gz_end - assets_index_js_gz_start);
    } else if (strcmp(uri, "/assets/index.css") == 0) {
        httpd_resp_set_type(req, "text/css");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_css_gz_start, assets_index_css_gz_end - assets_index_css_gz_start);
    }

    /* If no static file matches, return 404 */
    ESP_LOGW(TAG, "File not found: %s", uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t image_stream_handler(httpd_req_t *req)
{
    esp_err_t ret;
    struct v4l2_buffer buf;
    char http_string[128];
    bool locked = false;
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;

    ESP_RETURN_ON_FALSE(snprintf(http_string, sizeof(http_string), "%" PRIu32, video->frame_rate) > 0,
                        ESP_FAIL, TAG, "failed to format framerate buffer");

    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, STREAM_CONTENT_TYPE), TAG, "failed to set content type");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"), TAG, "failed to set access control allow origin");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "X-Framerate", http_string), TAG, "failed to set x framerate");

    while (1) {
        int hlen;
        struct timespec ts;
        uint32_t jpeg_encoded_size;

        locked = false;

        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        // ESP_LOGW(TAG, "Try DQ");
        ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), TAG, "failed to receive video frame");
        // ESP_LOGW(TAG, "DQ OK");

        

        if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");
            continue;
        }

        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)), fail0, TAG, "failed to send boundary");
        
        // yolov8n
        // BBox* bboxs = g_d_result.bboxs;
        // if (g_d_result.valid_num > 0) {
        //     for (int i=0; i < g_d_result.valid_num; ++i){
        //             draw_rectangle_rgb((uint16_t*)video->buffer[buf.index], video->width, video->height,
        //                 bbox_coordinate_x_scale_map(bboxs[i].x1, 224, 1920),
        //                 bbox_coordinate_x_scale_map(bboxs[i].y1, 224, 1080),
        //                 bbox_coordinate_x_scale_map(bboxs[i].x2, 224, 1920), 
        //                 bbox_coordinate_x_scale_map(bboxs[i].y2, 224, 1080),
        //                 0, 0, 255, 0, 0, 20, false);
        //         }
        // }
        // higherhrnet
        PoseKeyPoints* kps_group = g_pe_result.kps_group;
        BBox* bboxs = g_pe_result.bboxs;
        for (int i=0; i<g_pe_result.valid_num; ++i) {
            draw_rectangle_rgb((uint16_t*)video->buffer[buf.index], video->width, video->height,
                    bbox_coordinate_x_scale_map((int)(bboxs[i].x1), 384, 1920),
                    bbox_coordinate_x_scale_map((int)(bboxs[i].y1), 288, 1080),
                    bbox_coordinate_x_scale_map((int)(bboxs[i].x2), 384, 1920), 
                    bbox_coordinate_x_scale_map((int)(bboxs[i].y2), 288, 1080),
                    0, 0, 255, 0, 0, 20, false);
            for (int j=0; j < 17; ++j) {
                draw_rectangle_rgb((uint16_t*)video->buffer[buf.index], video->width, video->height,
                        bbox_coordinate_x_scale_map((int)(kps_group[i].data[j].x1), 384, 1920),
                        bbox_coordinate_x_scale_map((int)(kps_group[i].data[j].y1), 288, 1080),
                        bbox_coordinate_x_scale_map((int)(kps_group[i].data[j].x1)+2, 384, 1920), 
                        bbox_coordinate_x_scale_map((int)(kps_group[i].data[j].y1)+2, 288, 1080),
                        0, 0, 255, 0, 0, 20, false);
            }
        }

        if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
            video->jpeg_out_buf = video->buffer[buf.index];
            jpeg_encoded_size = buf.bytesused;
        } else {
            ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take semaphore");
            locked = true;

            ESP_GOTO_ON_ERROR(example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                              video->jpeg_out_buf, video->jpeg_out_size, &jpeg_encoded_size),
                              fail0, TAG, "failed to encode video frame");
        }

        ESP_GOTO_ON_ERROR(clock_gettime(CLOCK_MONOTONIC, &ts), fail0, TAG, "failed to get time");
        ESP_GOTO_ON_FALSE((hlen = snprintf(http_string, sizeof(http_string), STREAM_PART, jpeg_encoded_size, ts.tv_sec, ts.tv_nsec)) > 0,
                          ESP_FAIL, fail0, TAG, "failed to format part buffer");
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, http_string, hlen), fail0, TAG, "failed to send boundary");

        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, (char *)video->jpeg_out_buf, jpeg_encoded_size), fail0, TAG, "failed to send jpeg");
        if (locked) {
            xSemaphoreGive(video->sem);
            locked = false;
        }

        ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");
    }

    return ESP_OK;

fail0:
    if (locked) {
        xSemaphoreGive(video->sem);
    }
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    return ret;
}

static esp_err_t capture_image_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    char type_ptr[32];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "image/jpeg;name=image%d.jpg", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], true);
}

static esp_err_t capture_binary_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    char type_ptr[56];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "application/octet-stream;name=image_binary%d.bin", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], false);
}

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index)
{
    int fd;
    int ret;
    struct v4l2_streamparm sparm;
    struct v4l2_requestbuffers req;
    struct v4l2_captureparm *cparam = &sparm.parm.capture;
    struct v4l2_fract *timeperframe = &cparam->timeperframe;
    struct v4l2_format format;
    memset(&format, 0, sizeof(struct v4l2_format));

    fd = open(config->dev_name, O_RDWR);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_NOT_FOUND, TAG, "Open video device %s failed", config->dev_name);

    /* If the sensor outputs in RAW format, configure the ISP to output in YUV format by default. */
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0, /* The format at index 0 is the sensor's primitive output format.*/
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };

    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc), fail0, TAG, "Failed enmu fmt from %s", config->dev_name);
    if (fmtdesc.pixelformat == V4L2_PIX_FMT_SBGGR8) {
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_FMT, &format), fail0, TAG, "Failed get fmt from %s", config->dev_name);
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // format.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR8;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "Failed set fmt to %s", config->dev_name);
        ESP_LOGI(TAG, "Set fmt");
    }

    memset(&sparm, 0, sizeof(sparm));
    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_PARM, &sparm), fail0, TAG, "failed to get frame rate from %s", config->dev_name);
    video->frame_rate = timeperframe->denominator / timeperframe->numerator;

    memset(&req, 0, sizeof(req));
    req.count  = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers from %s", config->dev_name);

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf from %s", config->dev_name);

        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf from %s", config->dev_name);
    }

    video->fd = fd;
    video->width = format.fmt.pix.width;
    video->height = format.fmt.pix.height;
    video->pixel_format = format.fmt.pix.pixelformat;
    video->jpeg_quality = EXAMPLE_JPEG_ENC_QUALITY;

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(video, EXAMPLE_JPEG_ENC_QUALITY), fail0, TAG, "failed to set jpeg quality");
    } else {
        example_encoder_config_t encoder_config = {0};

        encoder_config.width = video->width;
        encoder_config.height = video->height;
        encoder_config.pixel_format = video->pixel_format;
        encoder_config.quality = EXAMPLE_JPEG_ENC_QUALITY;
        ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &video->encoder_handle), fail0, TAG, "failed to init encoder");

        ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size),
                          fail1, TAG, "failed to alloc jpeg output buf");

        video->support_control_jpeg_quality = 1;
    }

    video->sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(video->sem, ESP_ERR_NO_MEM, fail2, TAG, "failed to create semaphore");
    xSemaphoreGive(video->sem);

    return ESP_OK;

fail2:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        video->jpeg_out_buf = NULL;
    }
fail1:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_deinit(video->encoder_handle);
        video->encoder_handle = NULL;
    }
fail0:
    close(fd);
    video->fd = -1;
    return ret;
}

static esp_err_t deinit_web_cam_video(web_cam_video_t *video)
{
    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }

    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        example_encoder_deinit(video->encoder_handle);
    }

    close(video->fd);
    return ESP_OK;
}

static esp_err_t new_web_cam(const web_cam_video_config_t *config, int config_count, web_cam_t **ret_wc)
{
    int i;
    web_cam_t *wc;
    esp_err_t ret = ESP_FAIL;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    wc = calloc(1, sizeof(web_cam_t) + config_count * sizeof(web_cam_video_t));
    ESP_RETURN_ON_FALSE(wc, ESP_ERR_NO_MEM, TAG, "failed to alloc web cam");
    wc->video_count = config_count;

    for (i = 0; i < config_count; i++) {
        wc->video[i].index = i;
        wc->video[i].fd = -1;

        ret = init_web_cam_video(&wc->video[i], &config[i], i);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "failed to find web_cam %d", i);
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to initialize web_cam %d", i);
            goto fail0;
        }

        ESP_LOGI(TAG, "video%d: width=%" PRIu32 " height=%" PRIu32 " format=" V4L2_FMT_STR, i, wc->video[i].width,
                 wc->video[i].height, V4L2_FMT_STR_ARG(wc->video[i].pixel_format));
    }

    for (i = 0; i < config_count; i++) {
        if (is_valid_web_cam(&wc->video[i])) {
            ESP_GOTO_ON_ERROR(ioctl(wc->video[i].fd, VIDIOC_STREAMON, &type), fail1, TAG, "failed to start stream");
        }
    }

    // init_isp_dev(wc->video[0].fd);

    *ret_wc = wc;

    return ESP_OK;

fail1:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ioctl(wc->video[j].fd, VIDIOC_STREAMOFF, &type);
        }
    }
    i = config_count; // deinit all web_cam
fail0:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            deinit_web_cam_video(&wc->video[j]);
        }
    }
    free(wc);
    return ret;
}

static void free_web_cam(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            deinit_web_cam_video(&web_cam->video[i]);
        }
    }
    free(web_cam);
}

static esp_err_t http_server_init(web_cam_t *web_cam)
{
    httpd_handle_t stream_httpd;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* Unified static file handler for all static resources */
    httpd_uri_t static_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = (void *)web_cam
    };

    /* API handlers */
    httpd_uri_t capture_image_uri = {
        .uri = "/api/capture_image",
        .method = HTTP_GET,
        .handler = capture_image_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t capture_binary_uri = {
        .uri = "/api/capture_binary",
        .method = HTTP_GET,
        .handler = capture_binary_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_info_uri = {
        .uri = "/api/get_camera_info",
        .method = HTTP_GET,
        .handler = camera_info_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_settings_uri = {
        .uri = "/api/set_camera_config",
        .method = HTTP_POST,
        .handler = camera_settings_handler,
        .user_ctx = (void *)web_cam
    };

    config.stack_size = 1024 * 6;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        /* Register API handlers (more specific URIs) */
        httpd_register_uri_handler(stream_httpd, &capture_image_uri);
        httpd_register_uri_handler(stream_httpd, &capture_binary_uri);
        httpd_register_uri_handler(stream_httpd, &camera_info_uri);
        httpd_register_uri_handler(stream_httpd, &camera_settings_uri);

        /* Register wildcard static file handler to catch all other requests */
        httpd_register_uri_handler(stream_httpd, &static_file_uri);
    }

    for (int i = 0; i < web_cam->video_count; i++) {
        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        httpd_uri_t stream_0_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = image_stream_handler,
            .user_ctx = (void *) &web_cam->video[i]
        };

        config.stack_size = 1024 * 6;
        config.server_port += 1;
        config.ctrl_port += 1;
        if (httpd_start(&stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(stream_httpd, &stream_0_uri);
        }
    }

    return ESP_OK;
}

static esp_err_t start_cam_web_server(const web_cam_video_config_t *config, int config_count)
{
    esp_err_t ret;
    web_cam_t *web_cam;

    ESP_RETURN_ON_ERROR(new_web_cam(config, config_count, &web_cam), TAG, "Failed to new web cam");
    ESP_GOTO_ON_ERROR(http_server_init(web_cam), fail0, TAG, "Failed to init http server");

    return ESP_OK;

fail0:
    free_web_cam(web_cam);
    return ret;
}

static void initialise_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(EXAMPLE_MDNS_INSTANCE));

    mdns_txt_item_t serviceTxtData[] = {
        {"board", CONFIG_IDF_TARGET},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

bool wait_metadata_ready(void)
{
    int64_t start_us = esp_timer_get_time();
    const int64_t timeout_us = 20 * 1000 * 1000;
    uint32_t data_ready_status = 0;  // 0 not ready | 1 ready
    while ((esp_timer_get_time() - start_us) < timeout_us) {
        i2c_read_reg16_u32(DATA_READY_STATUS_REG, &data_ready_status);
        if (data_ready_status == 1) {
            // ESP_LOGI(TAG, "Data ready! (0x%lx)\n", data_ready_status);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "Timeout: Data not ready\n");
    return false;
}

void init_spi_dev()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO)
    );

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI_HOST, &devcfg, &g_spi_dev)
    );

    gpio_config_t cs_cfg = {
        .pin_bit_mask = 1ULL << PIN_NUM_CS,
        .mode = GPIO_MODE_OUTPUT,
    };

}

// void spi_read_dma(uint8_t *rx_buf, size_t len)
// {
//     static uint8_t dummy_tx[4096];

//     if (len > sizeof(dummy_tx)) {
//         ESP_LOGE("SPI", "len too large");
//         return;
//     }

//     memset(dummy_tx, 0x00, len);

//     spi_transaction_t t = {
//         .length = len * 8,
//         .tx_buffer = dummy_tx,
//         .rx_buffer = rx_buf,
//     };

//     ESP_ERROR_CHECK(spi_device_transmit(g_spi_dev, &t));
// }


void spi_read_dma(uint8_t *rx_buf, size_t len)
{
    static uint8_t dummy_tx[4096];

    if (len > sizeof(dummy_tx)) {
        ESP_LOGE("SPI", "len too large");
        return;
    }

    memset(dummy_tx, 0x00, len);

    spi_transaction_t t = {
        .length = len * 8,   // bits
        .rxlength = len * 8,
        .tx_buffer = dummy_tx,
        .rx_buffer = rx_buf,
    };

    gpio_set_level(PIN_NUM_CS, 0);          // CS ↓
    ESP_ERROR_CHECK(spi_device_transmit(g_spi_dev, &t));
    gpio_set_level(PIN_NUM_CS, 1);          // CS ↑
}

// int32_t spi_read(uint8_t *buf, uint32_t size)
// {
//     if (!buf || size == 0) {
//         return -1;
//     }

//     esp_err_t ret;
//     uint32_t offset = 0;

//     uint8_t *tx_buf = heap_caps_malloc(SPI_MAX_DMA_BYTES, MALLOC_CAP_DMA);
//     uint8_t *rx_buf = heap_caps_malloc(SPI_MAX_DMA_BYTES, MALLOC_CAP_DMA);

//     if (!tx_buf || !rx_buf) {
//         ESP_LOGE(TAG, "DMA malloc failed\n");
//         goto err;
//     }

//     memset(tx_buf, SPI_DUMMY_BYTE, SPI_MAX_DMA_BYTES);

//     while (offset < size) {
//         uint32_t chunk = size - offset;
//         if (chunk > SPI_MAX_DMA_BYTES) {
//             chunk = SPI_MAX_DMA_BYTES;
//         }

//         spi_transaction_t t = {
//             .length    = chunk * 8,
//             .rxlength  = chunk * 8,
//             .tx_buffer = tx_buf,
//             .rx_buffer = rx_buf,
//             .flags     = 0,
//         };

//         ret = spi_device_polling_transmit(g_spi_dev, &t);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "SPI read failed at offset %lu\n", offset);
//             goto err;
//         }

//         memcpy(buf + offset, rx_buf, chunk);
//         offset += chunk;
//     }

//     heap_caps_free(tx_buf);
//     heap_caps_free(rx_buf);
//     return size;

// err:
//     if (tx_buf) heap_caps_free(tx_buf);
//     if (rx_buf) heap_caps_free(rx_buf);
//     return -1;
// }

// int32_t spi_read(uint8_t *buf, uint32_t size)
// {
//     if (!buf || size == 0) {
//         return -1;
//     }

//     uint32_t size_ = size + VALID_DATA_OFFSET;

//     esp_err_t ret;
//     uint32_t offset = 0;

//     uint8_t *tx_buf = heap_caps_malloc(SPI_MAX_DMA_BYTES, MALLOC_CAP_DMA);
//     uint8_t *rx_buf = heap_caps_malloc(SPI_MAX_DMA_BYTES, MALLOC_CAP_DMA);

//     if (!tx_buf || !rx_buf) {
//         ESP_LOGE(TAG, "DMA malloc failed");
//         goto err;
//     }

//     memset(tx_buf, SPI_DUMMY_BYTE, SPI_MAX_DMA_BYTES);

//     while (offset < size_) {
//         uint32_t chunk = size_ - offset;
//         if (chunk > SPI_MAX_DMA_BYTES) {
//             chunk = SPI_MAX_DMA_BYTES;
//         }

//         bool is_last = (offset + chunk) >= size_;

//         spi_transaction_t t = {
//             .length    = chunk * 8,
//             .rxlength  = chunk * 8,
//             .tx_buffer = tx_buf,
//             .rx_buffer = rx_buf,
//             .flags     = is_last ? 0 : SPI_TRANS_CS_KEEP_ACTIVE,
//         };

//         ret = spi_device_polling_transmit(g_spi_dev, &t);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "SPI read failed at offset %lu", offset);
//             goto err;
//         }

//         memcpy(buf + offset, rx_buf, chunk);
//         offset += chunk;
//     }

//     heap_caps_free(tx_buf);
//     heap_caps_free(rx_buf);
//     return size;

// err:
//     if (tx_buf) heap_caps_free(tx_buf);
//     if (rx_buf) heap_caps_free(rx_buf);
//     return -1;
// }

int read_metadata(uint8_t *r_buf, uint32_t max_len, uint32_t* data_size) {

    static int count = 0;
    i2c_read_reg16_u32(METADATA_SIZE_REG, data_size);
    if (*data_size > max_len) {
        ESP_LOGE(TAG, "Error: data_size > max_data_r_buf_size: %lu > %lu\n", *data_size, max_len);
        return -2;  // data_size > max_data_r_buf_size
    }
    uint8_t dummy_read;
    if (count <= 0) { 
        // spi_read_dma(&dummy_read, 1);
        ++count;
        printf("count: %d\n", count);
        i2c_write_reg16_u32(CAPTURE_METADATA_REG, 1);
        vTaskDelay(pdMS_TO_TICKS(3 * 1000));
        ++count;
        
    }
    i2c_write_reg16_u32(CAPTURE_METADATA_REG, 1);
    if (!wait_metadata_ready()){
        return -1;  // metadata not ready
    }
    // ESP_LOGI(TAG, "starting spi read metadata ...\n");
    spi_read_dma(r_buf, *data_size);
    return 0;
}

static void metadata_reader_task(void *arg)
{
    metadata_frame_t frame;
    int ret;
    while (true) {
        ret = read_metadata(metadata_buf, MAX_DATA_R_BUF_SIZE, &frame.data_size);
        if (metadata_queue && ret == 0) {
            xQueueSend(metadata_queue, &frame, portMAX_DELAY);
        }
    }
}

static void metadata_parser_task(void *arg)
{
    metadata_frame_t frame;

    while (xQueueReceive(metadata_queue, &frame, portMAX_DELAY) == pdTRUE) {
        if (frame.data_size == 0 || frame.data_size > MAX_DATA_R_BUF_SIZE) {
            ESP_LOGW(TAG, "Invalid metadata size: %" PRIu32, frame.data_size);
            continue;
        }

        uint8_t *metadata = metadata_buf + VALID_DATA_OFFSET;
        // printf("\n\n\n");
        // print_buf_hex(metadata, frame.data_size);
        // printf("\n\n\n");

        // print_buf_hex(metadata, 12);
        // printf("\n");

        if(parse_ap_params(metadata, frame.data_size)) {
            // detect_postprocess_yolov8n();
            pose_estimate_postprocess_higherhrnet();
            // print_pose_estimation_result();
            // ESP_LOGW(TAG, "Parse Ap Params Failed.");
            // skip
        }
        // ESP_LOGI(TAG, "data_size: %d\n", frame.data_size);
    }
}

void init_i2c_dev(void) {
    i2c_master_get_bus_handle(0, &g_bus_handle);
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x0c,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(g_bus_handle, &dev_cfg, &g_client_handle);
}

esp_err_t i2c_write_reg16_u32(uint16_t reg, uint32_t value) {
    uint8_t buf[6];

    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 24) & 0xFF;
    buf[3] = (value >> 16) & 0xFF;
    buf[4] = (value >> 8) & 0xFF;
    buf[5] = value & 0xFF;

    return i2c_master_transmit(g_client_handle, buf, sizeof(buf), -1);
}

esp_err_t i2c_read_reg16_u32(uint16_t reg,uint32_t *out) {
    uint8_t wbuf[2];
    uint8_t rbuf[4];

    wbuf[0] = (reg >> 8) & 0xFF;
    wbuf[1] = reg & 0xFF;

    esp_err_t ret = i2c_master_transmit_receive(
        g_client_handle,
        wbuf, sizeof(wbuf),
        rbuf, sizeof(rbuf),
        -1
    );
    if (ret != ESP_OK) return ret;

    *out = ((uint32_t)rbuf[0] << 24) |
           ((uint32_t)rbuf[1] << 16) |
           ((uint32_t)rbuf[2] << 8)  |
            (uint32_t)rbuf[3];

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /*For camera devices that require the host to provide XCLK, the video_init() must be called immediately after the device is restarted,
    otherwise the camera device may not be able to start due to the lack of the main clock.*/

    ESP_ERROR_CHECK(example_video_init());
    init_i2c_dev();
    init_spi_dev();
    gpio_reset_pin(PIN_NUM_CS);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);          // CS ↑

    // while (1)
    // {
    //     gpio_set_level(PIN_NUM_CS, 1); // CS idle high
    //     vTaskDelay(pdMS_TO_TICKS(10));
    //     gpio_set_level(PIN_NUM_CS, 0); // CS idle high
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // while (1)
    // {
    //    gpio_set_level(PIN_NUM_CS, 1);
    //    ESP_LOGI(TAG, "CS UP");
    //    vTaskDelay(pdMS_TO_TICKS(10));
    //    gpio_set_level(PIN_NUM_CS, 0);
    //    ESP_LOGI(TAG, "CS DOWN");
    //    vTaskDelay(pdMS_TO_TICKS(10));
    // }
    

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(EXAMPLE_MDNS_HOST_NAME);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    web_cam_video_config_t config[] = {
#if EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR */
#if EXAMPLE_ENABLE_DVP_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_DVP_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_DVP_CAM_SENSOR */
#if EXAMPLE_ENABLE_SPI_CAM_0_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_SPI_CAM_0_SENSOR */
#if EXAMPLE_ENABLE_SPI_CAM_1_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_1_NAME,
        },
#endif /* EXAMPLE_ENABLE_SPI_CAM_1_SENSOR */
#if EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_USB_UVC_DEVICE_NAME(0),
        },
#endif /* EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR */
    };

    int config_count = sizeof(config) / sizeof(config[0]);

    assert(config_count > 0);
    ESP_ERROR_CHECK(start_cam_web_server(config, config_count));

    ESP_LOGI(TAG, "Camera web server starts");

    metadata_buf = (uint8_t *)malloc(MAX_DATA_R_BUF_SIZE);
    if (!metadata_buf) {
        ESP_LOGE(TAG, "Failed to allocate metadata buffer");
        return;
    }

    metadata_queue = xQueueCreate(1, sizeof(metadata_frame_t));
    if (!metadata_queue) {
        ESP_LOGE(TAG, "Failed to create metadata queue");
        return;
    }

    xTaskCreatePinnedToCore(metadata_reader_task, "metadata_reader", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(metadata_parser_task, "metadata_parser", 4096, NULL, 5, NULL, 0);

}
