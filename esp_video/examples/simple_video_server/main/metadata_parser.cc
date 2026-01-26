#include "metadata_parser.h"
#include "stdio.h"
#include "string.h"

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

void parseApParams(const uint8_t* data) {
    printf("parseApParams\n");
    const apParams::fb::FBApParams* ap_parameter = apParams::fb::GetFBApParams(data);
    auto networks = ap_parameter->networks();
    auto network = networks->Get(0);
    auto input_tensors = network->inputTensors();
    auto output_tensors = network->outputTensors();

    printf("InputTensor num: %ld\n", input_tensors->size());
    for (int i = 0; i < input_tensors->size(); ++i) {
        printf("InputTensor%d scale   %f\n", i, input_tensors->Get(i)->scale());
        printf("InputTensor%d shift   %d\n", i, input_tensors->Get(i)->shift());
        printf("InputTensor%d format  %d\n", i, input_tensors->Get(i)->format());
        printf("InputTensor%d dim_num %d\n", i, input_tensors->Get(i)->numOfDimensions());
        printf("InputTensor%d shape   [", i);
        for(int j = 0; j < input_tensors->Get(i)->dimensions()->size(); ++j) {
            printf(" %d", input_tensors->Get(i)->dimensions()->Get(j)->size());
        }
        printf(" ]\n");
    }
    printf("OutputTensor num: %ld\n", output_tensors->size());
    for (int i = 0; i < input_tensors->size(); ++i) {
        printf("OutputTensor%d scale             %f\n", i, output_tensors->Get(i)->scale());
        printf("OutputTensor%d shift             %d\n", i, output_tensors->Get(i)->shift());
        printf("OutputTensor%d format            %d\n", i, output_tensors->Get(i)->format());
        printf("OutputTensor%d bits_per_element  %d\n", i, output_tensors->Get(i)->bitsPerElement());
        printf("OutputTensor%d dim_num           %d\n", i, output_tensors->Get(i)->numOfDimensions());
        printf("OutputTensor%d shape   [", i);
        for(int j = 0; j < output_tensors->Get(i)->dimensions()->size(); ++j) {
            printf(" %d", output_tensors->Get(i)->dimensions()->Get(j)->size());
        }
        printf(" ]\n");
    }
    
}