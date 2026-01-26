/* Copyright [2024] arducam */
#ifndef ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_DEVICE_IMX500_IMX500OUTPUTPARSER_H_
#define ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_DEVICE_IMX500_IMX500OUTPUTPARSER_H_

#include "stdint.h"
#ifdef __cplusplus
#include <string>
#include <memory>
#include <vector>
#include "ApParams.h"
#include "common.h"

namespace arducam {

struct IMX500OutputHeader {
    uint8_t valid_flag;
    uint8_t frame_count;
    uint16_t max_length_of_line;
    uint16_t size_of_ap_parameter;
    uint16_t network_ordinal;
    uint8_t indicator;
};
class IMX500OutputParser {
 public:
    ~IMX500OutputParser() {
        this->m_ap_parameter = nullptr;
    }
    void initialize();
    int32_t verify(const IMX500OutputHeader* header);
    void unpackHeader(const uint8_t* data, IMX500OutputHeader* header);
    std::vector<Tensor> extractOriginalOutputTensor(
        const uint8_t* data, const IMX500OutputHeader* header);
    std::vector<Tensor> revertModelOutput(const uint8_t* data);
    std::string getProductName() const;

 public:
    const apParams::fb::FBApParams* m_ap_parameter;
    constexpr static uint32_t HEADER_LEN = 12;

 private:
    std::vector<Tensor> m_output_tensor;

 protected:
    uint8_t* m_dnn_data_ptr;

};

}  // namespace arducam

#endif

#ifdef __cplusplus
extern "C" {
#endif

enum ApProperty {

};


#ifdef __cplusplus
}
#endif

#endif  // ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_DEVICE_IMX500_IMX500OUTPUTPARSER_H_
