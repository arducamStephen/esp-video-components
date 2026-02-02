#ifndef ARDUCAM_IMX500_SDK_H_
#define ARDUCAM_IMX500_SDK_H_

#define VALID_DATA_OFFSET           0
#define IMX500_HEADER_LEN           12
#define MAX_DETECT_ITEM_NUM         10

#define METADATA_SIZE_REG           0x0701
#define DATA_READY_STATUS_REG       0x0705 
#define CAPTURE_METADATA_REG        0x0706
#define METADATA_SEND_SPI_MODE_REG  0x0707
#define BOOT_MODE_REG               0x0708
#define BOOT_STATUS_REG             0x0709
#define START_BOOT_REG              0x0710

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


#ifdef __cplusplus
#include "ApParams.h"
extern "C" {
#endif


void unpack_imx500_output_header(const uint8_t* data, IMX500OutputHeader* header);
bool parse_ap_params(const uint8_t* data, size_t data_len, DetectionResult* detection_result);
int32_t print_buf_hex(const uint8_t* buf, uint32_t len);
uint32_t bbox_coordinate_x_scale_map(float x, uint32_t s_w, uint32_t t_w);
uint32_t bbox_coordinate_y_scale_map(float y, uint32_t s_h, uint32_t t_h);

#ifdef __cplusplus
}
#endif

#endif  // ARDUCAM_IMX500_SDK_H_