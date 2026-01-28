#include "metadata_parser.h"
#include <algorithm>
#include <vector>
#include "stdio.h"
#include "string.h"
#include "g_config.h"

#define ALIGN_DOWN(size, align) ((size) & ~((align) - 1))
#define ALIGN_UP(size, align)   (ALIGN_DOWN((size) + (align) - 1, (align)))


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
    imx500_print_header(header); // for debug
}

extern "C" {
void parseApParams(const uint8_t* data) {
    // +-------------------------------------+
    // | ApParamsHeader(input_tensor_header) |
    // +-------------------------------------+
    // |         Input Tensor Data           |
    // +-------------------------------------+
    // | ApParamsHeader(output_tensor_header)|
    // +-------------------------------------+
    // |         Output Tensor Data          |
    // +-------------------------------------+
    uint32_t data_offset = 0;
    IMX500OutputHeader header;
    unpack_imx500_output_header(data, &header);
    data_offset += IMX500_HEADER_LEN;
    uint32_t total_input_tensor_data_size = 1;
    // 解析 ApParamsHeader(input_tensor_header) 并获取 Input Tensor Data 和 Output Tensor Data 大小
    // const apParams::fb::FBApParams* ap_parameter = apParams::fb::GetFBApParams(data+data_offset);
    // auto networks = ap_parameter->networks();
    // auto network = networks->Get(0);
    // auto input_tensors = network->inputTensors();
    // auto output_tensors = network->outputTensors();
    // data_offset += header.size_of_ap_parameter;

    // printf("InputTensor num: %ld\n", input_tensors->size());
    // for (int i = 0; i < input_tensors->size(); ++i) {
        // printf("InputTensor%d scale   %f\n", i, input_tensors->Get(i)->scale());
        // printf("InputTensor%d shift   %d\n", i, input_tensors->Get(i)->shift());
        // printf("InputTensor%d format  %d\n", i, input_tensors->Get(i)->format());
        // printf("InputTensor%d dim_num %d\n", i, input_tensors->Get(i)->numOfDimensions());
        // printf("InputTensor%d shape   [", i);
        // for(int j = 0; j < input_tensors->Get(i)->dimensions()->size(); ++j) {
        //     int dim = input_tensors->Get(i)->dimensions()->Get(j)->size();
        //     printf(" %d", dim);
        //     total_input_tensor_data_size *= dim;
        // }
        // printf(" ]\n");
    // }
    // data_offset += total_input_tensor_data_size;
    // printf("OutputTensor num: %ld\n", output_tensors->size());
    // for (int i = 0; i < output_tensors->size(); ++i) {
    //     printf("OutputTensor%d scale             %f\n", i, output_tensors->Get(i)->scale());
    //     printf("OutputTensor%d shift             %d\n", i, output_tensors->Get(i)->shift());
    //     printf("OutputTensor%d format            %d\n", i, output_tensors->Get(i)->format());
    //     printf("OutputTensor%d bits_per_element  %d\n", i, output_tensors->Get(i)->bitsPerElement());
    //     printf("OutputTensor%d dim_num           %d\n", i, output_tensors->Get(i)->numOfDimensions());
    //     printf("OutputTensor%d shape   [", i);

    //     for(int j = 0; j < output_tensors->Get(i)->dimensions()->size(); ++j) {
    //         int dim = output_tensors->Get(i)->dimensions()->Get(j)->size();
    //         printf(" %d", dim);
    //     }
    //     printf(" ]\n");
    // }
    // 解析 ApParamsHeader(output_tensor_header)
    const apParams::fb::FBApParams* ap_parameter = apParams::fb::GetFBApParams(data+data_offset);
    auto networks = ap_parameter->networks();
    auto network = networks->Get(0);
    auto input_tensors = network->inputTensors();
    auto output_tensors = network->outputTensors();
    data_offset += header.size_of_ap_parameter;

    if (output_tensors->size() < 4) {
        printf("OutputTensor num is insufficient: %ld\n", output_tensors->size());
        return;
    }

    const uint8_t* output_tensor_data = data + data_offset;
    std::vector<const uint8_t*> output_tensor_ptrs;
    std::vector<uint32_t> output_tensor_sizes;
    output_tensor_ptrs.reserve(output_tensors->size());
    output_tensor_sizes.reserve(output_tensors->size());

    uint32_t output_data_offset = 0;
    for (int i = 0; i < output_tensors->size(); ++i) {
        uint32_t tensor_elements = 1;
        for (int j = 0; j < output_tensors->Get(i)->dimensions()->size(); ++j) {
            tensor_elements *= output_tensors->Get(i)->dimensions()->Get(j)->size();
        }

        uint8_t bits_per_element = output_tensors->Get(i)->bitsPerElement();
        uint32_t tensor_bytes = bits_per_element == 16 ? tensor_elements * 2 : tensor_elements;
        output_tensor_ptrs.push_back(output_tensor_data + output_data_offset);
        output_tensor_sizes.push_back(tensor_elements);
        output_data_offset += ALIGN_UP(tensor_bytes, 4);
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
        printf("OutputTensor bbox stride is invalid: %lu\n", bbox_elements);
        return;
    }
    uint32_t score_elements = output_tensor_sizes[1];
    uint32_t class_elements = output_tensor_sizes[2];
    uint32_t detect_num = detect_num_data ? static_cast<uint32_t>(detect_num_data[0]) : 0;
    uint32_t max_items = std::min({bbox_stride, score_elements, class_elements, detect_num});

    printf("detect_num: %lu\n", detect_num);

    const float confidence_threshold = 0.1f;
    for (uint32_t i = 0; i < max_items; ++i) {
        float confidence = (static_cast<float>(score_data[i]) - score_tensor->shift()) * score_tensor->scale();
        if (confidence < confidence_threshold) {
            continue;
        }

        float xmin = (static_cast<float>(bbox_data[i]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float ymin = (static_cast<float>(bbox_data[i + bbox_stride]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float xmax = (static_cast<float>(bbox_data[i + bbox_stride * 2]) - bbox_tensor->shift()) * bbox_tensor->scale();
        float ymax = (static_cast<float>(bbox_data[i + bbox_stride * 3]) - bbox_tensor->shift()) * bbox_tensor->scale();
        uint32_t class_id = static_cast<uint32_t>((static_cast<float>(class_data[i]) - class_tensor->shift()) * class_tensor->scale());

        printf("box[%lu]: xmin=%0.2f ymin=%0.2f xmax=%0.2f ymax=%0.2f cls_id=%lu score=%0.3f\n",
               i, xmin, ymin, xmax, ymax, class_id, confidence);
    }
}
}
