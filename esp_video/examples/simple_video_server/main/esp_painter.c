#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <math.h>
#include "esp_log.h"
#include "esp_painter.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "driver/ppa.h"

static const char *TAG = "esp_painter";

#define ALIGN_UP(size) (((size) + data_cache_line_size - 1) & ~(data_cache_line_size - 1))

typedef struct {
    struct {
        uint16_t width;
        uint16_t height;
    } canvas;
    struct {
        uint16_t width;
        uint16_t height;
    } text_canvas;
    esp_painter_color_format_t color_format;
    const esp_painter_basic_font_t *default_font;
    bool swap_rgb565;         // Whether to swap RGB565 bytes
    ppa_client_handle_t ppa_client;
    uint8_t *ppa_output_buffer;
} esp_painter_t;

static size_t data_cache_line_size = 0;

static const struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_colors[] = {
    {0x00, 0x00, 0x00},  // BLACK
    {0x00, 0x00, 0x80},  // NAVY
    {0x00, 0x80, 0x00},  // DARKGREEN
    {0x00, 0x80, 0x80},  // DARKCYAN
    {0x80, 0x00, 0x00},  // MAROON
    {0x80, 0x00, 0x80},  // PURPLE
    {0x80, 0x80, 0x00},  // OLIVE
    {0xC0, 0xC0, 0xC0},  // LIGHTGREY
    {0x80, 0x80, 0x80},  // DARKGREY
    {0x00, 0x00, 0xFF},  // BLUE
    {0x00, 0xFF, 0x00},  // GREEN
    {0x00, 0xFF, 0xFF},  // CYAN
    {0xFF, 0x00, 0x00},  // RED
    {0xFF, 0x00, 0xFF},  // MAGENTA
    {0xFF, 0xFF, 0x00},  // YELLOW
    {0xFF, 0xFF, 0xFF},  // WHITE
    {0xFF, 0xA5, 0x00},  // ORANGE
    {0xAD, 0xFF, 0x2F},  // GREENYELLOW
    {0xFF, 0xC0, 0xCB},  // PINK
};

static uint32_t esp_painter_get_color(esp_painter_handle_t handle, esp_painter_color_t color)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || color >= sizeof(rgb_colors) / sizeof(rgb_colors[0])) {
        return 0;
    }

    uint8_t r = rgb_colors[color].r;
    uint8_t g = rgb_colors[color].g;
    uint8_t b = rgb_colors[color].b;

    uint32_t color_value = 0;
    switch (painter->color_format) {
        case ESP_PAINTER_COLOR_FORMAT_YUV420:
        case ESP_PAINTER_COLOR_FORMAT_YUV422:
        case ESP_PAINTER_COLOR_FORMAT_RGB565:
            color_value = RGB565(r, g, b);
            // Swap bytes if needed for RGB565
            if (painter->swap_rgb565) {
                color_value = ((color_value & 0xFF) << 8) | ((color_value >> 8) & 0xFF);
            }
            return color_value;
        case ESP_PAINTER_COLOR_FORMAT_RGB888:
            return RGB888(r, g, b);
        default:
            return 0;
    }
}

static esp_err_t esp_painter_convert_yuv420_to_rgb565(esp_painter_handle_t handle, uint8_t *in_buffer, uint8_t *out_buffer,
                                            uint32_t out_buffer_size,
                                            uint32_t block_w, uint32_t block_h,
                                            uint32_t block_offset_x, uint32_t block_offset_y)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !in_buffer || !out_buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    ppa_srm_oper_config_t srm_config = {
        .in.buffer = in_buffer,
        .in.pic_w = painter->canvas.width,
        .in.pic_h = painter->canvas.height,
        .in.block_w = block_w,
        .in.block_h = block_h,
        .in.block_offset_x = block_offset_x,
        .in.block_offset_y = block_offset_y,
        .in.srm_cm = PPA_SRM_COLOR_MODE_YUV420,
        .out.buffer = out_buffer,
        .out.buffer_size = out_buffer_size,
        .out.pic_w = block_w,
        .out.pic_h = block_h,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1,
        .scale_y = 1,
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    int ret = ppa_do_scale_rotate_mirror(painter->ppa_client, &srm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "First PPA conversion failed");
    }

    return ret;
}

static esp_err_t esp_painter_convert_rgb565_to_yuv420(esp_painter_handle_t handle, uint8_t *in_buffer, uint8_t *out_buffer, uint32_t buffer_size,
                                            uint32_t in_width, uint32_t in_height,
                                            uint32_t out_width, uint32_t out_height,
                                            uint32_t out_offset_x, uint32_t out_offset_y)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !in_buffer || !out_buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    ppa_srm_oper_config_t srm_out_config = {
        .in.buffer = in_buffer,
        .in.pic_w = in_width,
        .in.pic_h = in_height,
        .in.block_w = in_width,
        .in.block_h = in_height,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer = out_buffer,
        .out.buffer_size = buffer_size,
        .out.pic_w = out_width,
        .out.pic_h = out_height,
        .out.block_offset_x = out_offset_x,
        .out.block_offset_y = out_offset_y,
        .out.srm_cm = PPA_SRM_COLOR_MODE_YUV420,
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1,
        .scale_y = 1,
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    int ret = ppa_do_scale_rotate_mirror(painter->ppa_client, &srm_out_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Second PPA conversion failed");
    }

    return ESP_OK;
}

static esp_err_t esp_painter_draw_pixel(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
                           uint16_t x, uint16_t y, esp_painter_color_t color)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    bool using_ppa = (painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV420 || 
                     painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV422);

    uint32_t color_value = esp_painter_get_color(handle, color);

    if (using_ppa) {
        ESP_LOGD(TAG, "Drawing pixel at (%d,%d) with color 0x%04" PRIx32, x, y, color_value);
        
        uint32_t offset = (y * painter->text_canvas.width + x) * 2; 
        if (offset + 2 > buffer_size) {
            ESP_LOGW(TAG, "Buffer overflow: offset %" PRIu32 ", size %" PRIu32, offset, buffer_size);
            return ESP_ERR_INVALID_SIZE;
        }
        
        uint8_t low_byte = color_value & 0xFF;
        uint8_t high_byte = (color_value >> 8) & 0xFF;
        buffer[offset] = low_byte;
        buffer[offset + 1] = high_byte;
        
        return ESP_OK;
    }

    switch (painter->color_format) {
        case ESP_PAINTER_COLOR_FORMAT_RGB565: {
            uint32_t offset = (y * painter->canvas.width + x) * 2;
            if (offset + 2 > buffer_size) {
                return ESP_ERR_INVALID_SIZE;
            }
            buffer[offset] = color_value & 0xFF;
            buffer[offset + 1] = (color_value >> 8) & 0xFF;
            break;
        }
        case ESP_PAINTER_COLOR_FORMAT_RGB888: {
            uint32_t offset = (y * painter->canvas.width + x) * 3;
            if (offset + 3 > buffer_size) {
                return ESP_ERR_INVALID_SIZE;
            }
            buffer[offset] = color_value & 0xFF;
            buffer[offset + 1] = (color_value >> 8) & 0xFF;
            buffer[offset + 2] = (color_value >> 16) & 0xFF;
            break;
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

static esp_err_t esp_painter_draw_char(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
                               uint16_t x, uint16_t y, const esp_painter_basic_font_t *font,
                               esp_painter_color_t color, char c)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!font) {
        font = painter->default_font;
    }
    if (!font) {
        ESP_LOGE(TAG, "No font specified");
        return ESP_ERR_INVALID_ARG;
    }

    if (c < 32 || c > 127) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t char_offset = (c - 32) * font->height * ((font->width + 7) / 8);
    const uint8_t *char_bitmap = font->bitmap + char_offset;
    ESP_LOGD(TAG, "Drawing char '%c' at (%d,%d)", c, x, y);

    for (int dy = 0; dy < font->height; dy++) {
        for (int dx = 0; dx < font->width; dx++) {
            uint8_t byte = char_bitmap[dy * ((font->width + 7) / 8) + (dx / 8)];
            if (byte & (0x80 >> (dx % 8))) {
                ESP_LOGD(TAG, "Setting pixel at relative pos (%d,%d)", dx, dy);
                esp_err_t ret = esp_painter_draw_pixel(handle, buffer, buffer_size, x + dx, y + dy, color);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to draw pixel at (%d,%d)", x + dx, y + dy);
                    return ret;
                }
            }
        }
    }

    return ESP_OK;
}

static esp_err_t esp_painter_draw_text(esp_painter_t *painter, uint8_t *buffer,
                                uint32_t buffer_size, uint16_t x, uint16_t y,
                                uint32_t text_width, uint32_t text_height,
                                const esp_painter_basic_font_t *font,
                                esp_painter_color_t color, const char *text)
{
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;
    esp_err_t ret = ESP_OK;

    while (*text) {
        if (cursor_y >= text_height) {
            break;
        }

        switch (*text) {
            case '\n':
                cursor_x = x;
                cursor_y += font->height;
                break;
            case '\r':
                cursor_x = x;
                break;
            default:
                if (cursor_x + font->width <= text_width) {
                    ret = esp_painter_draw_char(painter, buffer, buffer_size,
                                              cursor_x, cursor_y, font, color, *text);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to draw char at (%d,%d)", cursor_x, cursor_y);
                        return ret;
                    }
                }
                cursor_x += font->width;
                
                if (cursor_x >= text_width) {
                    cursor_x = x;
                    cursor_y += font->height;
                }
                break;
        }
        text++;
    }
    return ret;
}

esp_err_t esp_painter_draw_string(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
                                 uint16_t x, uint16_t y, const esp_painter_basic_font_t *font,
                                 esp_painter_color_t color, const char* text)
{
    esp_err_t ret = ESP_OK;
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer || !text) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!font) {
        font = painter->default_font;
        if (!font) {
            ESP_LOGE(TAG, "No font specified");
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (x >= painter->canvas.width || y >= painter->canvas.height) {
        ESP_LOGW(TAG, "Starting position out of bounds: (%d,%d)", x, y);
        return ESP_ERR_INVALID_ARG;
    }

    bool using_ppa = (painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV420 || 
                     painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV422);

    if (using_ppa) {
        uint16_t text_width = strlen(text) * font->width;
        uint16_t text_height = font->height;

        painter->text_canvas.width = text_width;
        painter->text_canvas.height = text_height;

        uint32_t aligned_size = ALIGN_UP(text_width * text_height * 2);
        if (aligned_size > buffer_size) {
            ESP_LOGE(TAG, "Buffer too small: need %" PRIu32 ", have %" PRIu32, aligned_size, buffer_size);
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t *ppa_output_buffer = heap_caps_aligned_alloc(
            data_cache_line_size, 
            aligned_size, 
            MALLOC_CAP_SPIRAM
        );
        if (!ppa_output_buffer) {
            ESP_LOGE(TAG, "Failed to allocate PPA output buffer");
            return ESP_ERR_NO_MEM;
        }

        int ret = esp_painter_convert_yuv420_to_rgb565(painter, buffer, ppa_output_buffer,
                                                    aligned_size,
                                                    text_width, text_height,
                                                    x, y);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "First PPA conversion failed");
            goto cleanup;
        }

        ret = esp_painter_draw_text(painter, ppa_output_buffer, aligned_size, 0, 0, text_width, text_height, font, color, text);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw text");
            goto cleanup;
        }
ret = esp_painter_convert_rgb565_to_yuv420(painter, ppa_output_buffer, buffer,
                                                    buffer_size,
                                                    text_width, text_height,
                                                    painter->canvas.width, painter->canvas.height,
                                                    x, y);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Second PPA conversion failed");
            goto cleanup;
        }

cleanup:
        if (ppa_output_buffer) {
            heap_caps_free(ppa_output_buffer);
            ppa_output_buffer = NULL;
        }
        
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        ret = esp_painter_draw_text(painter, buffer, buffer_size, x, y, painter->canvas.width, painter->canvas.height, font, color, text);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw text");
            return ret;
        }
    }

    ret = esp_cache_msync(buffer, buffer_size, ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cache sync failed");
    }

    return ret;
}

esp_err_t esp_painter_draw_string_format(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
                                       uint16_t x, uint16_t y, const esp_painter_basic_font_t *font,
                                       esp_painter_color_t color, const char* fmt, ...)
{
    char buf[CONFIG_ESP_PAINTER_FORMAT_SIZE_MAX];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    return esp_painter_draw_string(handle, buffer, buffer_size, x, y, font, color, buf);
}

esp_err_t esp_painter_init(esp_painter_config_t *config, esp_painter_handle_t *handle)
{
    if (!config || !handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    esp_painter_t *painter = (esp_painter_t *)calloc(1, sizeof(esp_painter_t));
    if (!painter) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    painter->canvas.width = config->canvas.width;
    painter->canvas.height = config->canvas.height;
    painter->color_format = config->color_format;
    painter->default_font = config->default_font;
    painter->swap_rgb565 = config->swap_rgb565;

    if(config->color_format == ESP_PAINTER_COLOR_FORMAT_YUV420 || 
       config->color_format == ESP_PAINTER_COLOR_FORMAT_YUV422) {
        ppa_client_config_t ppa_srm_config = {
            .oper_type = PPA_OPERATION_SRM,
        };
        ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &painter->ppa_client));
        ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));
    }

    *handle = painter;
    ESP_LOGI(TAG, "Painter initialized: %dx%d, format: %d", 
             config->canvas.width, config->canvas.height, config->color_format);
    return ESP_OK;
}

// 使用Bresenham算法绘制直线
static esp_err_t esp_painter_draw_line_bresenham(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
    int x1, int y1, int x2, int y2, esp_painter_color_t color)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        esp_err_t ret = esp_painter_draw_pixel(handle, buffer, buffer_size, x1, y1, color);
        if (ret != ESP_OK) {
            return ret;
        }

        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    return ESP_OK;
}

// 绘制圆形点
static esp_err_t esp_painter_draw_circle(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
    uint16_t center_x, uint16_t center_y, uint16_t radius, esp_painter_color_t color)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;

    while (y >= x) {
        // 绘制8个对称点
        for (int i = -x; i <= x; i++) {
            esp_painter_draw_pixel(handle, buffer, buffer_size, center_x + i, center_y + y, color);
            esp_painter_draw_pixel(handle, buffer, buffer_size, center_x + i, center_y - y, color);
        }
        for (int i = -y; i <= y; i++) {
            esp_painter_draw_pixel(handle, buffer, buffer_size, center_x + i, center_y + x, color);
            esp_painter_draw_pixel(handle, buffer, buffer_size, center_x + i, center_y - x, color);
        }

        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }

    return ESP_OK;
}

// 使用PPA优化的绘制点函数
esp_err_t esp_painter_draw_point(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
    uint16_t x, uint16_t y, esp_painter_color_t color, uint16_t radius)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    bool using_ppa = (painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV420 || 
    painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV422);

    if (using_ppa && radius > 1) {
        // 对于大半径的点，使用PPA优化
        uint16_t point_width = radius * 2 + 1;
        uint16_t point_height = radius * 2 + 1;

        painter->text_canvas.width = point_width;
        painter->text_canvas.height = point_height;

        uint32_t aligned_size = ALIGN_UP(point_width * point_height * 2);
        if (aligned_size > buffer_size) {
            ESP_LOGE(TAG, "Buffer too small for point drawing");
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t *ppa_output_buffer = heap_caps_aligned_alloc(
        data_cache_line_size, 
        aligned_size, 
        MALLOC_CAP_SPIRAM);
        if (!ppa_output_buffer) {
            ESP_LOGE(TAG, "Failed to allocate PPA output buffer for point");
            return ESP_ERR_NO_MEM;
        }

        // 转换到RGB565进行绘制
        esp_err_t ret = esp_painter_convert_yuv420_to_rgb565(painter, buffer, ppa_output_buffer,
                                        aligned_size,
                                        point_width, point_height,
                                        x - radius, y - radius);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA conversion failed for point");
            goto cleanup;
        }

        // 在RGB565缓冲区中绘制圆形点
        ret = esp_painter_draw_circle(painter, ppa_output_buffer, aligned_size, radius, radius, radius, color);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw point circle");
            goto cleanup;
        }

        // 转换回YUV420
        ret = esp_painter_convert_rgb565_to_yuv420(painter, ppa_output_buffer, buffer,
                                buffer_size,
                                point_width, point_height,
                                painter->canvas.width, painter->canvas.height,
                                x - radius, y - radius);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA conversion back failed for point");
            goto cleanup;
        }

        cleanup:
        if (ppa_output_buffer) {
            heap_caps_free(ppa_output_buffer);
        }
        return ret;
    } else {
        // 对于小半径或非PPA模式，直接绘制
        if (radius <= 1) {
            return esp_painter_draw_pixel(handle, buffer, buffer_size, x, y, color);
        } else {
            return esp_painter_draw_circle(handle, buffer, buffer_size, x, y, radius, color);
        }
    }
}

// 使用PPA优化的绘制线函数
esp_err_t esp_painter_draw_line(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
                                uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, 
                                esp_painter_color_t color, uint16_t thickness)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    // 参数验证
    if (x1 >= painter->canvas.width || y1 >= painter->canvas.height ||
        x2 >= painter->canvas.width || y2 >= painter->canvas.height) {
        ESP_LOGW(TAG, "Line coordinates out of bounds: (%d,%d)-(%d,%d) on canvas %dx%d", 
                x1, y1, x2, y2, painter->canvas.width, painter->canvas.height);
        return ESP_ERR_INVALID_ARG;
    }

    if (thickness == 0) {
        thickness = 1; // 最小厚度为1
    }

    bool using_ppa = (painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV420 || 
                     painter->color_format == ESP_PAINTER_COLOR_FORMAT_YUV422);

    // 对于PPA格式且厚度大于1的线，使用PPA优化
    if (using_ppa && thickness > 1) {
        // 计算线的边界框，考虑厚度
        uint16_t min_x = (x1 < x2) ? x1 : x2;
        uint16_t min_y = (y1 < y2) ? y1 : y2;
        uint16_t max_x = (x1 > x2) ? x1 : x2;
        uint16_t max_y = (y1 > y2) ? y1 : y2;
        
        // 确保边界框有足够的空间容纳厚度
        uint16_t padding = thickness;
        min_x = (min_x > padding) ? min_x - padding : 0;
        min_y = (min_y > padding) ? min_y - padding : 0;
        max_x = (max_x + padding < painter->canvas.width) ? max_x + padding : painter->canvas.width - 1;
        max_y = (max_y + padding < painter->canvas.height) ? max_y + padding : painter->canvas.height - 1;
        
        uint16_t line_width = max_x - min_x + 1;
        uint16_t line_height = max_y - min_y + 1;

        // 检查边界框是否有效
        if (line_width == 0 || line_height == 0) {
            ESP_LOGE(TAG, "Invalid line dimensions: width=%d, height=%d", line_width, line_height);
            return ESP_ERR_INVALID_SIZE;
        }

        painter->text_canvas.width = line_width;
        painter->text_canvas.height = line_height;

        uint32_t aligned_size = ALIGN_UP(line_width * line_height * 2);
        if (aligned_size > buffer_size) {
            ESP_LOGE(TAG, "Buffer too small for line drawing: need %" PRIu32 ", have %" PRIu32, 
                    aligned_size, buffer_size);
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t *ppa_output_buffer = heap_caps_aligned_alloc(
            data_cache_line_size, 
            aligned_size, 
            MALLOC_CAP_SPIRAM
        );
        if (!ppa_output_buffer) {
            ESP_LOGE(TAG, "Failed to allocate PPA output buffer for line");
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = ESP_OK;

        // 转换到RGB565进行绘制
        ret = esp_painter_convert_yuv420_to_rgb565(painter, buffer, ppa_output_buffer,
                                                  aligned_size,
                                                  line_width, line_height,
                                                  min_x, min_y);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA conversion to RGB565 failed");
            goto cleanup;
        }

        // 在RGB565缓冲区中绘制粗线
        int16_t local_x1 = x1 - min_x;
        int16_t local_y1 = y1 - min_y;
        int16_t local_x2 = x2 - min_x;
        int16_t local_y2 = y2 - min_y;

        // 确保局部坐标在有效范围内
        if (local_x1 < 0 || local_y1 < 0 || local_x2 < 0 || local_y2 < 0 ||
            local_x1 >= line_width || local_y1 >= line_height || 
            local_x2 >= line_width || local_y2 >= line_height) {
            ESP_LOGW(TAG, "Local coordinates out of bounds, using direct drawing");
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        // 绘制粗线：使用多个平行线
        if (thickness == 1) {
            ret = esp_painter_draw_line_bresenham(painter, ppa_output_buffer, aligned_size, 
                                                 local_x1, local_y1, local_x2, local_y2, color);
        } else {
            // 计算线的方向向量
            int16_t dx = local_x2 - local_x1;
            int16_t dy = local_y2 - local_y1;
            
            // 计算垂直方向
            int16_t perp_dx = -dy;
            int16_t perp_dy = dx;
            
            // 归一化垂直向量
            float length = sqrtf(perp_dx * perp_dx + perp_dy * perp_dy);
            if (length > 0) {
                perp_dx = (int16_t)(perp_dx * thickness / (2 * length));
                perp_dy = (int16_t)(perp_dy * thickness / (2 * length));
            }
            
            // 绘制多条平行线来形成粗线
            for (int t = -thickness/2; t <= thickness/2; t++) {
                int16_t offset_x = (int16_t)(t * perp_dx / thickness);
                int16_t offset_y = (int16_t)(t * perp_dy / thickness);
                
                ret = esp_painter_draw_line_bresenham(painter, ppa_output_buffer, aligned_size, 
                                                     local_x1 + offset_x, local_y1 + offset_y,
                                                     local_x2 + offset_x, local_y2 + offset_y, color);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to draw parallel line %d", t);
                    break;
                }
            }
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw line in RGB565 buffer");
            goto cleanup;
        }

        // 转换回YUV420
        ret = esp_painter_convert_rgb565_to_yuv420(painter, ppa_output_buffer, buffer,
                                                  buffer_size,
                                                  line_width, line_height,
                                                  painter->canvas.width, painter->canvas.height,
                                                  min_x, min_y);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA conversion back to YUV420 failed");
            goto cleanup;
        }

        // 同步缓存
        ret = esp_cache_msync(buffer, buffer_size, ESP_CACHE_MSYNC_FLAG_INVALIDATE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cache sync failed after line drawing");
        }

    cleanup:
        if (ppa_output_buffer) {
            heap_caps_free(ppa_output_buffer);
        }
        return ret;
    } else {
        // 对于非PPA模式或细线，直接绘制
        esp_err_t ret = ESP_OK;
        
        if (thickness == 1) {
            // 单像素线
            ret = esp_painter_draw_line_bresenham(handle, buffer, buffer_size, x1, y1, x2, y2, color);
        } else {
            // 计算线的方向向量
            int16_t dx = x2 - x1;
            int16_t dy = y2 - y1;
            
            // 计算垂直方向
            int16_t perp_dx = -dy;
            int16_t perp_dy = dx;
            
            // 归一化垂直向量
            float length = sqrtf(perp_dx * perp_dx + perp_dy * perp_dy);
            if (length > 0) {
                perp_dx = (int16_t)(perp_dx * thickness / (2 * length));
                perp_dy = (int16_t)(perp_dy * thickness / (2 * length));
            }
            
            // 绘制多条平行线来形成粗线
            for (int t = -thickness/2; t <= thickness/2; t++) {
                int16_t offset_x = (int16_t)(t * perp_dx / thickness);
                int16_t offset_y = (int16_t)(t * perp_dy / thickness);
                
                ret = esp_painter_draw_line_bresenham(handle, buffer, buffer_size, 
                                                     x1 + offset_x, y1 + offset_y,
                                                     x2 + offset_x, y2 + offset_y, color);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to draw parallel line %d", t);
                    break;
                }
            }
        }
        
        return ret;
    }
}

// /**
//  * @brief 基于 Bresenham 算法绘制直线
//  * @param handle 绘图句柄
//  * @param buffer 显示缓冲区
//  * @param buffer_size 缓冲区大小
//  * @param x1 起点 X 坐标
//  * @param y1 起点 Y 坐标
//  * @param x2 终点 X 坐标
//  * @param y2 终点 Y 坐标
//  * @param color 直线颜色
//  * @return 错误码：ESP_OK 成功，其他失败
//  */
// esp_err_t esp_painter_draw_line(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
//     uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, esp_painter_color_t color)
// {
//     // 1. 基础参数合法性校验
//     if (handle == NULL || buffer == NULL || buffer_size == 0) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // 3. 起点终点重合，直接画点
//     if (x1 == x2 && y1 == y2) {
//         return esp_painter_draw_pixel(handle, buffer, buffer_size, x1, y1, color);
//     }

//     // 4. Bresenham 算法变量初始化
//     int16_t dx = (int16_t)x2 - (int16_t)x1;  // X 轴差值
//     int16_t dy = (int16_t)y2 - (int16_t)y1;  // Y 轴差值
//     int16_t step_x = (dx > 0) ? 1 : -1;      // X 轴步进方向（左/右）
//     int16_t step_y = (dy > 0) ? 1 : -1;      // Y 轴步进方向（上/下）
//     dx = (dx < 0) ? -dx : dx;                // 取绝对值
//     dy = (dy < 0) ? -dy : dy;

//     int16_t current_x = x1;  // 当前 X 坐标
//     int16_t current_y = y1;  // 当前 Y 坐标
//     int16_t err;             // 误差项

//     // 5. 分情况处理：水平/垂直/斜线
//     if (dx > dy) {
//         // 情况1：X 轴为主方向（水平类直线）
//         err = dx / 2;
//         while (current_x != x2) {
//             // 绘制当前点
//             esp_err_t ret = esp_painter_draw_pixel(handle, buffer, buffer_size, (uint16_t)current_x, (uint16_t)current_y, color);
//             if (ret != ESP_OK) return ret;

//             // 更新误差项和坐标
//             err -= dy;
//             if (err < 0) {
//             current_y += step_y;
//             err += dx;
//             }
//             current_x += step_x;
//         }
//     } else {
//         // 情况2：Y 轴为主方向（垂直/斜线）
//         err = dy / 2;
//         while (current_y != y2) {
//             // 绘制当前点
//             esp_err_t ret = esp_painter_draw_pixel(handle, buffer, buffer_size, (uint16_t)current_x, (uint16_t)current_y, color);
//             if (ret != ESP_OK) return ret;

//             // 更新误差项和坐标
//             err -= dx;
//             if (err < 0) {
//                 current_x += step_x;
//                 err += dy;
//             }
//             current_y += step_y;
//         }
//     }

//     // 绘制最后一个点（确保终点被绘制）
//     return esp_painter_draw_pixel(handle, buffer, buffer_size, x2, y2, color);
// }


// // 绘制矩形
// esp_err_t esp_painter_draw_rectangle(esp_painter_handle_t handle, uint8_t *buffer, uint32_t buffer_size,
//                                     uint16_t x, uint16_t y, uint16_t width, uint16_t height,
//                                     esp_painter_color_t color, uint16_t thickness, bool filled)
// {
//     esp_painter_t *painter = (esp_painter_t *)handle;
//     if (!painter || !buffer) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t ret = ESP_OK;

//     if (filled) {
//         // 绘制实心矩形
//         for (uint16_t dy = 0; dy < height; dy++) {
//             for (uint16_t dx = 0; dx < width; dx++) {
//                 ret = esp_painter_draw_pixel(handle, buffer, buffer_size, x + dx, y + dy, color);
//                 if (ret != ESP_OK) return ret;
//             }
//         }
//     } else {
//         // 绘制空心矩形
//         // 上边
//         ret = esp_painter_draw_line(handle, buffer, buffer_size, x, y, x + width, y, color, thickness);
//         if (ret != ESP_OK) return ret;
//         // 下边
//         ret = esp_painter_draw_line(handle, buffer, buffer_size, x, y + height, x + width, y + height, color, thickness);
//         if (ret != ESP_OK) return ret;
//         // 左边
//         ret = esp_painter_draw_line(handle, buffer, buffer_size, x, y, x, y + height, color, thickness);
//         if (ret != ESP_OK) return ret;
//         // 右边
//         ret = esp_painter_draw_line(handle, buffer, buffer_size, x + width, y, x + width, y + height, color, thickness);
//         if (ret != ESP_OK) return ret;
//     }

//     return ret;
// }

esp_err_t esp_painter_deinit(esp_painter_handle_t handle)
{
    esp_painter_t *painter = (esp_painter_t *)handle;
    if (!painter) {
        return ESP_ERR_INVALID_ARG;
    }

    if(painter->ppa_client) {
        ppa_unregister_client(painter->ppa_client);
        painter->ppa_client = NULL;
    }

    free(painter);
    return ESP_OK;
}