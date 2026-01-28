#include "metadata_parser.h"
#include "stdio.h"
#include "string.h"
#include "g_config.h"

#define ALIGN_DOWN(size, align) (size & ~((align)-1))
#define ALIGN_UP(size, align) ALIGN_DOWN(size + align - 1, align)

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
    uint32_t total_output_tensor_data_size = 1;
    // 解析 ApParamsHeader(input_tensor_header) 并获取 Input Tensor Data 和 Output Tensor Data 大小
    const apParams::fb::FBApParams* ap_parameter = apParams::fb::GetFBApParams(data+data_offset);
    auto networks = ap_parameter->networks();
    auto network = networks->Get(0);
    auto input_tensors = network->inputTensors();
    auto output_tensors = network->outputTensors();
    data_offset += header.size_of_ap_parameter;

    // printf("InputTensor num: %ld\n", input_tensors->size());
    for (int i = 0; i < input_tensors->size(); ++i) {
        // printf("InputTensor%d scale   %f\n", i, input_tensors->Get(i)->scale());
        // printf("InputTensor%d shift   %d\n", i, input_tensors->Get(i)->shift());
        // printf("InputTensor%d format  %d\n", i, input_tensors->Get(i)->format());
        // printf("InputTensor%d dim_num %d\n", i, input_tensors->Get(i)->numOfDimensions());
        // printf("InputTensor%d shape   [", i);
        for(int j = 0; j < input_tensors->Get(i)->dimensions()->size(); ++j) {
            int dim = input_tensors->Get(i)->dimensions()->Get(j)->size();
            // printf(" %d", dim);
            total_input_tensor_data_size *= dim;
        }
        // printf(" ]\n");
    }
    data_offset += total_input_tensor_data_size;
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
    
    print_buf_hex(data+data_offset, 12);
    data_offset += IMX500_HEADER_LEN;
    // printf("\n");
    // 解析 ApParamsHeader(output_tensor_header)
    ap_parameter = apParams::fb::GetFBApParams(data+data_offset);
    networks = ap_parameter->networks();
    network = networks->Get(0);
    input_tensors = network->inputTensors();
    output_tensors = network->outputTensors();

    printf("InputTensor num: %ld\n", input_tensors->size());
    for (int i = 0; i < input_tensors->size(); ++i) {
        printf("InputTensor%d scale   %f\n", i, input_tensors->Get(i)->scale());
        printf("InputTensor%d shift   %d\n", i, input_tensors->Get(i)->shift());
        printf("InputTensor%d format  %d\n", i, input_tensors->Get(i)->format());
        printf("InputTensor%d dim_num %d\n", i, input_tensors->Get(i)->numOfDimensions());
        printf("InputTensor%d shape   [", i);
        for(int j = 0; j < input_tensors->Get(i)->dimensions()->size(); ++j) {
            int dim = input_tensors->Get(i)->dimensions()->Get(j)->size();
            printf(" %d", dim);
        }
        printf(" ]\n");
    }
    printf("OutputTensor num: %ld\n", output_tensors->size());
    for (int i = 0; i < output_tensors->size(); ++i) {
        printf("OutputTensor%d scale             %f\n", i, output_tensors->Get(i)->scale());
        printf("OutputTensor%d shift             %d\n", i, output_tensors->Get(i)->shift());
        printf("OutputTensor%d format            %d\n", i, output_tensors->Get(i)->format());
        printf("OutputTensor%d bits_per_element  %d\n", i, output_tensors->Get(i)->bitsPerElement());
        printf("OutputTensor%d dim_num           %d\n", i, output_tensors->Get(i)->numOfDimensions());
        printf("OutputTensor%d shape   [", i);

        for(int j = 0; j < output_tensors->Get(i)->dimensions()->size(); ++j) {
            int dim = output_tensors->Get(i)->dimensions()->Get(j)->size();
            printf(" %d", dim);
        }
        printf(" ]\n");
    }
}
}