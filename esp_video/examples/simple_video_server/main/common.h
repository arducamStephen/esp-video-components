/* Copyright [2024] arducam */
#ifndef ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_COMMON_H_
#define ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_COMMON_H_

#include "stdint.h"
#include "stdlib.h"

typedef struct {
    uint8_t valid_flag;
    uint8_t frame_count;
    uint16_t max_length_of_line;
    uint16_t size_of_ap_parameter;
    uint16_t network_ordinal;
    uint8_t indicator;
} IMX500OutputHeader;

typedef struct {
    uint8_t dim_num;               /* 维度数量 */
    uint32_t* shape;               /* shape 数组指针 */
    uint16_t shift;                /* 量化 shift */
    float scale;                   /* 量化 scale */
    uint8_t* data;                 /* 量化数据 */
    uint8_t type;                  /* 0 int8 | 1 float */
    uint8_t bits_per_element;
} Tensor;

typedef struct {
    float scale;
    int zero_point;
} QuantParam;

typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int   class_id;
} BBox;

typedef struct {
    BBox* bboxs;
    uint16_t valid_num;
} DetectionResult;

typedef struct {
    IMX500OutputHeader input_tensor_header;
    IMX500OutputHeader output_tensor_header;
    Tensor input_tensor;
    Tensor* output_tensors;
    uint16_t output_tensor_num;
} Metadata;

static inline void tensor_free(Tensor* t)
{
    if (!t) return;

    if (t->shape) {
        free(t->shape);
        t->shape = NULL;
    }

    if (t->data) {
        free(t->data);
        t->data = NULL;
    }

    t->dim_num = 0;
}

static inline void metadata_free(Metadata* meta)
{
    if (!meta) return;

    /* 1. 释放 input tensor 内部资源 */
    tensor_free(&meta->input_tensor);

    /* 2. 释放 output tensors */
    if (meta->output_tensors) {
        for (uint16_t i = 0; i < meta->output_tensor_num; i++) {
            tensor_free(&meta->output_tensors[i]);
        }

        free(meta->output_tensors);
        meta->output_tensors = NULL;
    }

    meta->output_tensor_num = 0;
}

#endif  // ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_COMMON_H_
