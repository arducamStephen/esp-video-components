#include "ArducamIMX500SDK.h"
#include <algorithm>
#include <vector>
#include "stdio.h"
#include "string.h"
#include <algorithm>
#include "munkres.hpp"

#define ALIGN_DOWN(size, align) ((size) & ~((align) - 1))
#define ALIGN_UP(size, align)   (ALIGN_DOWN((size) + (align) - 1, (align)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static std::vector<const uint8_t*> s_output_tensor_ptrs;
static const ::flatbuffers::Vector<::flatbuffers::Offset<apParams::fb::FBOutputTensor>>* s_output_tensors_fb;
DetectionResult g_d_result;
PoseEstimationResult g_pe_result;

int32_t print_buf_hex(const uint8_t* buf, uint32_t len) {
    uint32_t i;
    for (i = 0; i < len; ++i) {
        printf("0x%02x ", buf[i]);
    }
    return 0;
}

void imx500_print_header(const IMX500OutputHeader *h)
{
    if (!h) {
        printf("IMX500OutputHeader: NULL\n");
        return;
    }

    printf("IMX500OutputHeader {\n");
    printf("  valid_flag               : 0x%02X (%s)\n",
           h->valid_flag,
           h->valid_flag ? "valid" : "invalid");

    printf("  frame_count              : %u\n", h->frame_count);
    printf("  max_length_of_line        : %u\n", h->max_length_of_line);
    printf("  size_of_ap_p_parameter   : %u\n", h->size_of_ap_parameter);
    printf("  network_ordinal           : %u\n", h->network_ordinal);
    printf("  indicator                 : 0x%02X\n", h->indicator);
    printf("}\n");
}

void unpack_imx500_output_header(const uint8_t* data, IMX500OutputHeader* header) {
    const IMX500OutputHeader *data_ = (const IMX500OutputHeader *)data;
    memcpy(header, data_, sizeof(IMX500OutputHeader));
    // imx500_print_header(header); // for debug
}

extern "C" {
#include <algorithm>
#include <vector>
#include <cstdio>
#include "flatbuffers/flatbuffers.h"

bool parse_ap_params(const uint8_t* data, size_t data_len) {
    if (!data) {
        printf("parse_ap_params: null input\n");
        return false;
    }
    if (data_len < IMX500_HEADER_LEN) {
        printf("parse_ap_params: data_len too small: %u\n", (unsigned)data_len);
        return false;
    }

    uint32_t data_offset = 0;
    IMX500OutputHeader header;
    unpack_imx500_output_header(data, &header);
    data_offset += IMX500_HEADER_LEN;

    if (header.size_of_ap_parameter == 0) {
        printf("ApParams size is 0\n");
        return false;
    }

    if ((size_t)data_offset + (size_t)header.size_of_ap_parameter > data_len) {
        printf("ApParams out of range: offset=%lu size=%u data_len=%u\n",
               data_offset, header.size_of_ap_parameter, (unsigned)data_len);
        return false;
    }

    const uint8_t* ap_buf = data + data_offset;
    size_t ap_len = header.size_of_ap_parameter;

    // FlatBuffers verify
    flatbuffers::Verifier verifier(ap_buf, ap_len);
    if (!apParams::fb::VerifyFBApParamsBuffer(verifier)) {
        // printf("ApParams flatbuffer verify failed\n");
        return false;
    }

    const apParams::fb::FBApParams* ap_parameter = apParams::fb::GetFBApParams(ap_buf);
    if (!ap_parameter) {
        printf("GetFBApParams returned null\n");
        return false;
    }

    auto networks = ap_parameter->networks();
    if (!networks || networks->size() == 0) {
        printf("networks is null or empty\n");
        return false;
    }

    auto network = networks->Get(0);
    if (!network) {
        printf("network[0] is null\n");
        return false;
    }

    s_output_tensors_fb = network->outputTensors();
    if (!s_output_tensors_fb) {
        printf("outputTensors is null (schema mismatch or field missing)\n");
        return false;
    }

    data_offset += header.size_of_ap_parameter;
    if ((size_t)data_offset > data_len) {
        printf("output tensor data offset out of range: %lu / %u\n",
               data_offset, (unsigned)data_len);
        return false;
    }

    const uint8_t* output_tensor_data = data + data_offset;

    std::vector<uint32_t> output_tensor_sizes;
    s_output_tensor_ptrs.clear();
    s_output_tensor_ptrs.reserve(s_output_tensors_fb->size());
    output_tensor_sizes.reserve(s_output_tensors_fb->size());

    uint32_t output_data_offset = 0;

    for (uint32_t i = 0; i < s_output_tensors_fb->size(); ++i) {
        auto t = s_output_tensors_fb->Get(i);
        if (!t) {
            printf("output_tensors[%lu] is null\n", i);
            return false;
        }

        auto dims = t->dimensions();
        if (!dims || dims->size() == 0) {
            printf("tensor[%lu] dims is null/empty\n", i);
            return false;
        }

        uint32_t tensor_elements = 1;
        for (uint32_t j = 0; j < dims->size(); ++j) {
            auto d = dims->Get(j);
            if (!d) {
                printf("tensor[%lu] dim[%lu] is null\n", i, j);
                return false;
            }
            uint32_t s = (uint32_t)d->size();
            if (s == 0) {
                printf("tensor[%lu] dim[%lu] size=0\n", i, j);
                return false;
            }
            // over prevention
            if (tensor_elements > (UINT32_MAX / s)) {
                printf("tensor[%lu] elements overflow\n", i);
                return false;
            }
            tensor_elements *= s;
        }

        uint8_t bits_per_element = t->bitsPerElement();
        uint32_t tensor_bytes = (bits_per_element == 16) ? (tensor_elements * 2) : tensor_elements;
        uint32_t tensor_bytes_aligned = ALIGN_UP(tensor_bytes, 4);

        if ((size_t)data_offset + (size_t)output_data_offset + (size_t)tensor_bytes_aligned > data_len) {
            printf("tensor[%lu] data out of range: header_off(imx500_header_len + ap_params_header_len)=%lu off=%lu tensor_elements=%lu bytes=%lu aligned=%lu data_len=%u\n",
                   i, data_offset, output_data_offset, tensor_elements, tensor_bytes, tensor_bytes_aligned, (unsigned)data_len);
            return false;
        }

        s_output_tensor_ptrs.push_back(output_tensor_data + output_data_offset);
        output_tensor_sizes.push_back(tensor_elements);
        output_data_offset += tensor_bytes_aligned;
    }

    return true;
}

void print_pose_estimation_result(void) {
    auto result = &g_pe_result;
    if (!result) {
        printf("[Pose] result is NULL\n");
        return;
    }

    printf("========== PoseEstimationResult ==========\n");
    printf("Valid num: %u\n", result->valid_num);

    for (uint16_t i = 0; i < result->valid_num; ++i) {
        printf("\n--- Person %u ---\n", i);

        const PoseKeyPoints* pose = &result->kps_group[i];

        for (int kp = 0; kp < 17; ++kp) {
            const KeyPoint* k = &pose->data[kp];
            printf(
                "  KP[%02d]: x=%7.2f  y=%7.2f  score=%.3f\n",
                kp, k->x1, k->y1, k->score
            );
        }

        /* 如果你后面需要一起打印 bbox */
        const BBox* b = &result->bboxs[i];
        printf(
            "  BBox: x1=%f y1=%f x2=%f y2=%f score=%.3f\n",
            b->x1, b->y1, b->x2, b->y2, b->score
        );
    }

    printf("==========================================\n");
}

#define PE_DEBUG 0

bool pose_estimate_postprocess_higherhrnet(void) {

    if (!s_output_tensors_fb) {
        printf("[PE] outputTensors is null\n");
        return false;
    }

    if (s_output_tensors_fb->size() != 3) {
        printf("[PE] OutputTensor num error: %lu\n",
               (unsigned long)s_output_tensors_fb->size());
        return false;
    }

    const auto* raw_tag = s_output_tensors_fb->Get(0);
    const auto* raw_ind = s_output_tensors_fb->Get(1);
    const auto* raw_val = s_output_tensors_fb->Get(2);

    const auto* raw_tag_data = reinterpret_cast<const int8_t*>(s_output_tensor_ptrs[0]);
    const auto* raw_ind_data = reinterpret_cast<const int32_t*>(s_output_tensor_ptrs[1]);
    const auto* raw_val_data = reinterpret_cast<const int8_t*>(s_output_tensor_ptrs[2]);

    auto max_num_people = raw_tag->dimensions()->Get(0)->size();
    auto num_joints     = raw_tag->dimensions()->Get(1)->size();

    // printf("[PE] max_num_people=%d, num_joints=%d\n",
    //        max_num_people, num_joints);
    // printf("[PE] tag scale=%.6f shift=%d\n", raw_tag->scale(), raw_tag->shift());
    // printf("[PE] val scale=%.6f shift=%d\n", raw_val->scale(), raw_val->shift());

    auto kps_group = g_pe_result.kps_group;
    auto bboxs = g_pe_result.bboxs;
    g_pe_result.valid_num = 0;

    const float confidence_threshold = 0.35f;

    HigherHRNetOutput higherhrnet_output;
    higherhrnet_output.tag.resize(num_joints, std::vector<float>(max_num_people));
    higherhrnet_output.ind.resize(num_joints, std::vector<int32_t>(max_num_people));
    higherhrnet_output.val.resize(num_joints, std::vector<float>(max_num_people));

    /* --------- tensor unpack & dequant --------- */
    for (uint32_t i = 0; i < max_num_people * num_joints; ++i) {
        uint32_t j = i / max_num_people;
        uint32_t p = i % max_num_people;

        higherhrnet_output.tag[j][p] =
            (raw_tag_data[i] - raw_tag->shift()) * raw_tag->scale();
        higherhrnet_output.ind[j][p] = raw_ind_data[i];
        higherhrnet_output.val[j][p] =
            (raw_val_data[i] - raw_val->shift()) * raw_val->scale();

#if PE_DEBUG
        if (i < 5) {
            printf("[PE] raw[%ld] tag=%d val=%d ind=%ld -> tag=%.3f val=%.3f\n",
                   i,
                   raw_tag_data[i],
                   raw_val_data[i],
                   raw_ind_data[i],
                   higherhrnet_output.tag[j][p],
                   higherhrnet_output.val[j][p]);
        }
#endif
    }

    /* --------- postprocess --------- */
    std::pair<int, int> img_size = {288, 384};
    std::pair<int, int> img_w_pad = {0, 0};
    std::pair<int, int> img_h_pad = {0, 0};
    std::pair<int, int> input_image_size = {288, 384};
    std::pair<int, int> output_shape = {144, 192};

    auto result_ = postprocess_higherhrnet(
        higherhrnet_output,
        img_size, img_w_pad, img_h_pad,
        confidence_threshold, true,
        input_image_size, output_shape);

    auto keypoints_ = std::get<0>(result_);
    auto scores_    = std::get<1>(result_);
    auto boxes_     = std::get<2>(result_);

#if PE_DEBUG
    printf("[PE] postprocess output: persons=%lu\n",
           (unsigned long)keypoints_.size());
#endif

    uint32_t max_items =
        std::min((uint32_t)keypoints_.size(),
                 (uint32_t)MAX_DETECT_ITEM_NUM);

    for (uint32_t i = 0; i < max_items; ++i) {

    printf("[PE] person %ld score=%.3f box=[%.1f %.1f %.1f %.1f]\n",
               i,
               scores_[i],
               boxes_[i][0], boxes_[i][1],
               boxes_[i][2], boxes_[i][3]);

#if PE_DEBUG
        printf("[PE] person %ld score=%.3f box=[%.1f %.1f %.1f %.1f]\n",
               i,
               scores_[i],
               boxes_[i][0], boxes_[i][1],
               boxes_[i][2], boxes_[i][3]);
#endif

        if (scores_[i] < confidence_threshold) continue;

        bboxs[i].class_id = 0;
        bboxs[i].score = scores_[i];
        bboxs[i].x1 = MIN(MAX(0, boxes_[i][0]), 384);
        bboxs[i].y1 = MIN(MAX(0, boxes_[i][1]), 288);
        bboxs[i].x2 = MIN(MAX(0, boxes_[i][2]), 384);
        bboxs[i].y2 = MIN(MAX(0, boxes_[i][3]), 288);

        for (int j = 0; j < num_joints; ++j) {
            kps_group[i].data[j].x1 = keypoints_[i][j * 3 + 0];
            kps_group[i].data[j].y1 = keypoints_[i][j * 3 + 1];
            kps_group[i].data[j].score = keypoints_[i][j * 3 + 2];

#if PE_DEBUG
            if (j < 2) {
                printf("    kp[%d] x=%.1f y=%.1f s=%.2f\n",
                       j,
                       kps_group[i].data[j].x1,
                       kps_group[i].data[j].y1,
                       kps_group[i].data[j].score);
            }
#endif
        }

        g_pe_result.valid_num++;
    }

#if PE_DEBUG
    printf("[PE] valid_num=%d\n", g_pe_result.valid_num);
#endif

    return true;
}

uint32_t bbox_coordinate_x_scale_map(float x, uint32_t s_w, uint32_t t_w) {
    uint32_t x_ = 0;
    float s = (float)(t_w) / (float)(s_w);
    x_ = MIN((uint32_t)(x*s), t_w);
    return x_;
}

uint32_t bbox_coordinate_y_scale_map(float y, uint32_t s_h, uint32_t t_h) {
    uint32_t y_ = 0;
    float s = (float)(t_h) / (float)(s_h);
    y_ = MIN((uint32_t)(y*s), t_h);
    return y_;
}


}
