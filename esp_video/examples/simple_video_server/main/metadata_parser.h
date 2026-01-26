#ifndef METADATA_PARSER_H_
#define METADATA_PARSER_H_

#include "common.h"

#ifdef __cplusplus
#include "ApParams.h"
extern "C" {
#endif


void unpack_imx500_output_header(const uint8_t* data, IMX500OutputHeader* header);
void parseApParams(const uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif  // METADATA_PARSER_H_