#include "ArducamIMX500SDK.h"
#include <algorithm>
#include <vector>
#include "stdio.h"
#include "string.h"

#define ALIGN_DOWN(size, align) ((size) & ~((align) - 1))
#define ALIGN_UP(size, align)   (ALIGN_DOWN((size) + (align) - 1, (align)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


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

bool parse_ap_params(const uint8_t* data, size_t data_len, DetectionResult* detection_result) {
    if (!data || !detection_result) {
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

    auto output_tensors = network->outputTensors();
    if (!output_tensors) {
        printf("outputTensors is null (schema mismatch or field missing)\n");
        return false;
    }

    if (output_tensors->size() < 4) {
        printf("OutputTensor num is insufficient: %lu\n", (unsigned long)output_tensors->size());
        return false;
    }

    data_offset += header.size_of_ap_parameter;
    if ((size_t)data_offset > data_len) {
        printf("output tensor data offset out of range: %lu / %u\n",
               data_offset, (unsigned)data_len);
        return false;
    }

    const uint8_t* output_tensor_data = data + data_offset;

    std::vector<const uint8_t*> output_tensor_ptrs;
    std::vector<uint32_t> output_tensor_sizes;
    output_tensor_ptrs.reserve(output_tensors->size());
    output_tensor_sizes.reserve(output_tensors->size());

    uint32_t output_data_offset = 0;

    for (uint32_t i = 0; i < output_tensors->size(); ++i) {
        auto t = output_tensors->Get(i);
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
            printf("%lu ", s);
            tensor_elements *= s;
        }
        printf("\n");

        printf("tensor_elements: %ld\n", tensor_elements);
        uint8_t bits_per_element = t->bitsPerElement();
        uint32_t tensor_bytes = (bits_per_element == 16) ? (tensor_elements * 2) : tensor_elements;
        uint32_t tensor_bytes_aligned = ALIGN_UP(tensor_bytes, 4);

        printf("tensor[%lu] data out of range: header_off(imx500_header_len + ap_params_header_len)=%lu off=%lu bytes=%lu aligned=%lu data_len=%u\n",
                   i, data_offset, output_data_offset, tensor_bytes, tensor_bytes_aligned, (unsigned)data_len);

        if ((size_t)data_offset + (size_t)output_data_offset + (size_t)tensor_bytes_aligned > data_len) {
            printf("tensor[%lu] data out of range: header_off(imx500_header_len + ap_params_header_len)=%lu off=%lu bytes=%lu aligned=%lu data_len=%u\n",
                   i, data_offset, output_data_offset, tensor_bytes, tensor_bytes_aligned, (unsigned)data_len);
            return false;
        }

        output_tensor_ptrs.push_back(output_tensor_data + output_data_offset);
        output_tensor_sizes.push_back(tensor_elements);
        output_data_offset += tensor_bytes_aligned;
    }

    const auto* bbox_tensor = output_tensors->Get(0);
    const auto* score_tensor = output_tensors->Get(1);
    const auto* class_tensor = output_tensors->Get(2);
    const auto* detect_num_tensor = output_tensors->Get(3);

    const auto* bbox_data = reinterpret_cast<const int16_t*>(output_tensor_ptrs[0]);
    const auto* score_data = reinterpret_cast<const uint8_t*>(output_tensor_ptrs[1]);
    const auto* class_data = reinterpret_cast<const int16_t*>(output_tensor_ptrs[2]);
    const auto* detect_num_data = reinterpret_cast<const int16_t*>(output_tensor_ptrs[3]);

    uint32_t bbox_elements = output_tensor_sizes[0];
    uint32_t bbox_stride = bbox_elements / 4;
    if (bbox_stride == 0) {
        printf("OutputTensor bbox stride is invalid: %lu\n", (unsigned long)bbox_elements);
        return false;
    }

    uint32_t score_elements = output_tensor_sizes[1];
    uint32_t class_elements = output_tensor_sizes[2];
    uint32_t detect_num = detect_num_data ? (uint32_t)detect_num_data[0] : 0;

    uint32_t max_items = std::min({bbox_stride, score_elements, class_elements, detect_num, (uint32_t)MAX_DETECT_ITEM_NUM});

    auto bboxs = detection_result->bboxs;
    detection_result->valid_num = 0;

    const float confidence_threshold = 0.1f;
    for (uint32_t i = 0; i < max_items; ++i) {
        float confidence = (static_cast<float>(score_data[i]) - score_tensor->shift()) * score_tensor->scale();
        if (confidence < confidence_threshold) continue;

        float xmin = (static_cast<float>(bbox_data[i]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float ymin = (static_cast<float>(bbox_data[i + bbox_stride]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float xmax = (static_cast<float>(bbox_data[i + bbox_stride * 2]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float ymax = (static_cast<float>(bbox_data[i + bbox_stride * 3]) - bbox_tensor->shift()) * bbox_tensor->scale();
        uint32_t class_id = (uint32_t)((static_cast<float>(class_data[i]) - class_tensor->shift()) * class_tensor->scale());

        bboxs->class_id = class_id;
        bboxs->score = confidence;
        bboxs->x1 = xmin;
        bboxs->y1 = ymin;
        bboxs->x2 = xmax;
        bboxs->y2 = ymax;

        printf("box[%lu]: xmin=%0.2f ymin=%0.2f xmax=%0.2f ymax=%0.2f cls_id=%lu score=%0.3f\n",
               (unsigned long)i, xmin, ymin, xmax, ymax, (unsigned long)class_id, confidence);

        bboxs++;
        detection_result->valid_num++;
    }

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
