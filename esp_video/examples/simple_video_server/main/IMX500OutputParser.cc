/* Copyright [2024] arducam */
#include <memory>
#include <string>
#include <cstring>
#include <numeric>
#include <functional>
#include <iostream>
#include "IMX500OutputParser.h"

#define ALIGN_DOWN(size, align) ((size) & (~((align)-1)))
#define ALIGN_UP(size, align) ALIGN_DOWN(size + align - 1, align)

namespace arducam {

    void IMX500OutputParser::initialize() {}

    std::string IMX500OutputParser::getProductName() const {
        return "IMX500OutputParser";
    }


    int32_t IMX500OutputParser::verify(const IMX500OutputHeader* header) {
        if (header->valid_flag != 0x01) return 1;
        return 0;
    }

    std::vector<Tensor> IMX500OutputParser::extractOriginalOutputTensor(
        const uint8_t* data,
        const IMX500OutputHeader* header) {
        this->m_dnn_data_ptr =
            const_cast<uint8_t*>(
                data + HEADER_LEN + header->size_of_ap_parameter);
        this->m_ap_parameter = apParams::fb::GetFBApParams(data + HEADER_LEN);
        // TODO(luwei): 添加m_ap_parameter为空的异常处理
        auto networks = this->m_ap_parameter->networks();
        // fpk 的多模型设计为不同时序切换模型，这里恒定为1个模型
        auto network = networks->Get(0);
        auto output_tensors = network->outputTensors();

        // if (this->m_output_tensor.capacity() < output_tensors->size()) {
        //     this->m_output_tensor.reserve(output_tensors->size()); // 提前分配足够的空间
        //     this->m_output_tensor.resize(output_tensors->size());
        // }
        this->m_output_tensor.resize(output_tensors->size());


        uint32_t data_offset = 0;
        for (int i = 0; i < output_tensors->size(); ++i) {
            this->m_output_tensor[i].scale = output_tensors->Get(i)->scale();
            this->m_output_tensor[i].shift = output_tensors->Get(i)->shift();
            this->m_output_tensor[i].dim_num =
                output_tensors->Get(i)->numOfDimensions();
            m_output_tensor[i].bits_per_element = output_tensors->Get(i)->bitsPerElement();
            // if (this->m_output_tensor[i].shape.capacity() < this->m_output_tensor[i].dim_num) {
            //     this->m_output_tensor[i].shape.reserve(this->m_output_tensor[i].dim_num); // 提前分配足够的空间
            //     this->m_output_tensor[i].shape.resize(this->m_output_tensor[i].dim_num);
            // }
            this->m_output_tensor[i].shape.resize(this->m_output_tensor[i].dim_num);
            for (int j = 0; j < this->m_output_tensor[i].shape.size(); ++j) {
                this->m_output_tensor[i].shape[j] =
                    output_tensors->Get(i)->dimensions()->Get(j)->size();
            }
            this->m_output_tensor[i].int8_data =
                reinterpret_cast<int8_t*>(this->m_dnn_data_ptr + data_offset);
            int output_tensor_size = std::accumulate(
                this->m_output_tensor[i].shape.begin(),
                this->m_output_tensor[i].shape.begin() + this->m_output_tensor[i].dim_num,
                1,
                std::multiplies<uint32_t>());

            // TODO(luwei): 此处错误需要中断
            switch (m_output_tensor[i].bits_per_element) {
            case 8:
                data_offset += ALIGN_UP(output_tensor_size, 4);
                break;
            case 16:
                data_offset += ALIGN_UP(output_tensor_size * 2, 4);
                break;
            default:
                data_offset += ALIGN_UP(output_tensor_size, 4);
                std::cout << "Invalid bits_per_element: " << (int32_t)m_output_tensor[i].bits_per_element << std::endl;
                break;
            }
        }
        return this->m_output_tensor;
    }

    void IMX500OutputParser::unpackHeader(
        const uint8_t* data,
        IMX500OutputHeader* header) {
        const IMX500OutputHeader* data_ =
            reinterpret_cast<const IMX500OutputHeader*>(data);
        memcpy(header, data_, sizeof(IMX500OutputHeader));
    }

    std::vector<Tensor> IMX500OutputParser::revertModelOutput(const uint8_t* data) {
        IMX500OutputHeader* header = reinterpret_cast<IMX500OutputHeader*>(
            malloc(sizeof(IMX500OutputHeader)));
        this->unpackHeader(data, header);
        this->verify(header);
        auto quantified_output_tensor = this->
            extractOriginalOutputTensor(data, header);
        free(header);
        return quantified_output_tensor;
    }

}  // namespace arducam


Metadata get_ap_property(uint8_t* data) {

}