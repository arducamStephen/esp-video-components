#include "imx500_tensor_parser.h"

#include <cmath>
#include <future>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// #include "libcamera/base/log.h"
#include "libcamera/base/span.h"
#include "metadata.h"
#include "ApParams.h"
#include "esp_log.h"

using namespace libcamera;
using namespace RPiController;

// LOG_DEFINE_CATEGORY(IMX500)

namespace RPiController {

/* Setup in the IMX500 driver */
constexpr unsigned int TensorStride = 1024;	//2560;

constexpr unsigned int DnnHeaderSize = 12;
constexpr unsigned int MipiPhSize = 0;
constexpr unsigned int InputSensorMaxWidth = 1280;
constexpr unsigned int InputSensorMaxHeight = 960;

enum TensorDataType {
	Signed = 0,
	Unsigned
};

struct DnnHeader {
	uint8_t frameValid;
	uint8_t frameCount;
	uint16_t maxLineLen;
	uint16_t apParamSize;
	uint16_t networkId;
	uint8_t tensorType;
};

struct OutputTensorApParams {
	uint8_t id;
	std::string name;
	std::string networkName;
	uint16_t numDimensions;
	uint8_t bitsPerElement;
	std::vector<Dimensions> vecDim;
	uint16_t shift;
	float scale;
	uint8_t format;
};

struct InputTensorApParams {
	uint8_t networkId;
	std::string networkName;
	uint16_t width;
	uint16_t height;
	uint16_t channel;
	uint16_t widthStride;
	uint16_t heightStride;
	uint8_t format;
};


static const char *TAG = "imx500";

Metadata dnnMetadata;

int parseHeader(DnnHeader &dnnHeader, std::vector<uint8_t> &apParams, const uint8_t *src)
{
	dnnHeader = *reinterpret_cast<const DnnHeader *>(src);

	// LOG(IMX500, Debug)
	// 	<< "Header: valid " << static_cast<bool>(dnnHeader.frameValid)
	// 	<< " count " << static_cast<int>(dnnHeader.frameCount)
	// 	<< " max len " << dnnHeader.maxLineLen
	// 	<< " ap param size " << dnnHeader.apParamSize
	// 	<< " network id " << dnnHeader.networkId
	// 	<< " tensor type " << static_cast<int>(dnnHeader.tensorType);
    // ESP_LOGI(TAG, "Header: valid %d count %d max len %u ap param size %u network id %u tensor type %d",
    //     static_cast<bool>(dnnHeader.frameValid),
    //     static_cast<int>(dnnHeader.frameCount),
    //     dnnHeader.maxLineLen,
    //     dnnHeader.apParamSize,
    //     dnnHeader.networkId,
    //     static_cast<int>(dnnHeader.tensorType));

	if (!dnnHeader.frameValid)
		return -1;

	apParams.resize(dnnHeader.apParamSize, 0);

	uint32_t i = DnnHeaderSize;
	unsigned int j = 0;
	// ESP_LOGI(TAG, "apParamsSize: %u", dnnHeader.apParamSize);
	for (j = 0; j < dnnHeader.apParamSize; j++) {
		if (i >= TensorStride) {
			ESP_LOGI(TAG, "i exceed TensorStride, resetting to 0");
			i = 0;
			src += TensorStride + MipiPhSize;
		}
		apParams[j] = src[i++];
	}

	// ESP_LOGI(TAG, "apParams size: %u, i=%d, j=%d", apParams.size(), i, j);
	// ESP_LOGI(TAG, "apParams contents:");
	// for (size_t i = 0; i < apParams.size(); i += 16) {
	// 	char line[3 * 16 + 1] = {0}; // Buffer to hold the formatted line
	// 	int pos = 0;
	// 	for (size_t j = 0; j < 16 && (i + j) < apParams.size(); ++j) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", apParams[i + j]);
	// 	}
	// 	ESP_LOGI(TAG, "%04X: %s", static_cast<unsigned int>(i), line);
	// }

	return 0;
}

int parseOutputApParams(std::vector<OutputTensorApParams> &outputApParams, std::vector<uint8_t> &apParams,
			const DnnHeader &dnnHeader)
{
	const apParams::fb::FBApParams *fbApParams;
	const apParams::fb::FBNetwork *fbNetwork;
	const apParams::fb::FBOutputTensor *fbOutputTensor;

	// std::vector<uint8_t> bytesToInsert = {0x00, 0x00, 0x80, 0x3b, 0x14, 0x00, 0x00, 0x00};
	// apParams.insert(apParams.begin() + 340, bytesToInsert.begin(), bytesToInsert.end());

	// Remove the last 8 bytes
	// if (apParams.size() >= 8) {
	// 	apParams.erase(apParams.end() - 8, apParams.end());
	// }

	// ESP_LOGI(TAG, "apParams size: %u", apParams.size());
	// ESP_LOGI(TAG, "apParams contents:");
	// for (size_t i = 0; i < apParams.size(); i += 16) {
	// 	char line[3 * 16 + 1] = {0}; // Buffer to hold the formatted line
	// 	int pos = 0;
	// 	for (size_t j = 0; j < 16 && (i + j) < apParams.size(); ++j) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", apParams[i + j]);
	// 	}
	// 	ESP_LOGI(TAG, "%04X: %s", static_cast<unsigned int>(i), line);
	// }

	fbApParams = apParams::fb::GetFBApParams(apParams.data());
	// LOG(IMX500, Debug) << "Networks size: " << fbApParams->networks()->size();
    // ESP_LOGI(TAG, "Output Networks size: %u", fbApParams->networks()->size());

	outputApParams.clear();

	for (unsigned int i = 0; i < fbApParams->networks()->size(); i++) {
		fbNetwork = (apParams::fb::FBNetwork *)(fbApParams->networks()->Get(i));
		if (fbNetwork->id() != dnnHeader.networkId)
			continue;

		// LOG(IMX500, Debug)
		// 	<< "Network: " << fbNetwork->type()->c_str()
		// 	<< ", i/p size: " << fbNetwork->inputTensors()->size()
		// 	<< ", o/p size: " << fbNetwork->outputTensors()->size();
        ESP_LOGI(TAG, "Output Network: %s, i/p size: %u, o/p size: %u",
            fbNetwork->type()->c_str(),
            fbNetwork->inputTensors()->size(),
            fbNetwork->outputTensors()->size());

		for (unsigned int j = 0; j < fbNetwork->outputTensors()->size(); j++) {
			OutputTensorApParams outApParam;

			fbOutputTensor = (apParams::fb::FBOutputTensor *)fbNetwork->outputTensors()->Get(j);

			outApParam.id = fbOutputTensor->id();
			outApParam.name = fbOutputTensor->name()->str();
			outApParam.networkName = fbNetwork->type()->str();
			outApParam.numDimensions = fbOutputTensor->numOfDimensions();

			for (unsigned int k = 0; k < fbOutputTensor->numOfDimensions(); k++) {
				Dimensions dim;
				dim.ordinal = fbOutputTensor->dimensions()->Get(k)->id();
				dim.size = fbOutputTensor->dimensions()->Get(k)->size();
				dim.serializationIndex = fbOutputTensor->dimensions()->Get(k)->serializationIndex();
				dim.padding = fbOutputTensor->dimensions()->Get(k)->padding();
				if (dim.padding != 0) {
					// LOG(IMX500, Error)
					// 	<< "Error in AP Params, Non-Zero padding for Dimension " << k;
                    ESP_LOGE(TAG, "Error in AP Params, Non-Zero padding for Dimension %u", k);
					return -1;
				}

				outApParam.vecDim.push_back(dim);
			}

			outApParam.bitsPerElement = fbOutputTensor->bitsPerElement();
			outApParam.shift = fbOutputTensor->shift();
			outApParam.scale = fbOutputTensor->scale();
			outApParam.format = fbOutputTensor->format();

			/* Add the element to vector */
			outputApParams.push_back(outApParam);
		}

		break;
	}

	// Log the contents of outputApParams
    ESP_LOGI(TAG, "outputApParams size: %u", outputApParams.size());
    // for (size_t i = 0; i < outputApParams.size(); ++i) {
    //     const auto &param = outputApParams[i];
    //     ESP_LOGI(TAG, "Output Tensor %zu:", i);
    //     ESP_LOGI(TAG, "  ID: %u", param.id);
    //     ESP_LOGI(TAG, "  Name: %s", param.name.c_str());
    //     ESP_LOGI(TAG, "  Network Name: %s", param.networkName.c_str());
    //     ESP_LOGI(TAG, "  Num Dimensions: %u", param.numDimensions);
    //     ESP_LOGI(TAG, "  Bits Per Element: %u", param.bitsPerElement);
    //     ESP_LOGI(TAG, "  Shift: %u", param.shift);
    //     ESP_LOGI(TAG, "  Scale: %f", param.scale);
    //     ESP_LOGI(TAG, "  Format: %u", param.format);

    //     ESP_LOGI(TAG, "  Dimensions:");
    //     for (size_t j = 0; j < param.vecDim.size(); ++j) {
    //         const auto &dim = param.vecDim[j];
    //         ESP_LOGI(TAG, "    Dimension %zu:", j);
    //         ESP_LOGI(TAG, "      Ordinal: %u", dim.ordinal);
    //         ESP_LOGI(TAG, "      Size: %u", dim.size);
    //         ESP_LOGI(TAG, "      Serialization Index: %u", dim.serializationIndex);
    //         ESP_LOGI(TAG, "      Padding: %u", dim.padding);
    //     }
	// }

	return 0;
}

int populateOutputTensorInfo(IMX500OutputTensorInfo &outputTensorInfo,
			     const std::vector<OutputTensorApParams> &outputApParams)
{
	/* Calculate total output size. */
	unsigned int totalOutSize = 0;
	for (auto const &ap : outputApParams) {
		unsigned int totalDimensionSize = 1;
		for (auto &dim : ap.vecDim) {
			if (totalDimensionSize >= std::numeric_limits<uint32_t>::max() / dim.size) {
				// LOG(IMX500, Error) << "Invalid totalDimensionSize";
                ESP_LOGE(TAG, "Invalid totalDimensionSize");
				return -1;
			}

			totalDimensionSize *= dim.size;
		}

		if (totalOutSize >= std::numeric_limits<uint32_t>::max() - totalDimensionSize) {
			// LOG(IMX500, Error) << "Invalid totalOutSize";
            ESP_LOGE(TAG, "Invalid totalOutSize");
			return -1;
		}

		totalOutSize += totalDimensionSize;
	}

	if (totalOutSize == 0) {
		// LOG(IMX500, Error) << "Invalid output tensor info (totalOutSize is 0)";
        ESP_LOGE(TAG, "Invalid output tensor info (totalOutSize is 0)");
		return -1;
	}

	// LOG(IMX500, Debug) << "Final output size: " << totalOutSize;
    // ESP_LOGI(TAG, "Final output size: %u", totalOutSize);

	if (totalOutSize >= std::numeric_limits<uint32_t>::max() / sizeof(float)) {
		// LOG(IMX500, Error) << "Invalid output tensor info";
        ESP_LOGE(TAG, "Invalid output tensor info");
		return -1;
	}

	outputTensorInfo.data = std::shared_ptr<float[]>(new float[totalOutSize]);
	unsigned int numOutputTensors = outputApParams.size();

	if (!numOutputTensors) {
		// LOG(IMX500, Error) << "Invalid numOutputTensors (0)";
        ESP_LOGE(TAG, "Invalid numOutputTensors (0)");
		return -1;
	}

	if (numOutputTensors >= std::numeric_limits<uint32_t>::max() / sizeof(uint32_t)) {
		// LOG(IMX500, Error) << "Invalid numOutputTensors";
        ESP_LOGE(TAG, "Invalid numOutputTensors");
		return -1;
	}

	outputTensorInfo.totalSize = totalOutSize;
	outputTensorInfo.numTensors = numOutputTensors;
	outputTensorInfo.networkName = outputApParams[0].networkName;
	outputTensorInfo.tensorDataNum.resize(numOutputTensors, 0);
	for (auto const &p : outputApParams) {
		outputTensorInfo.vecDim.push_back(p.vecDim);
		outputTensorInfo.numDimensions.push_back(p.vecDim.size());
	}

	return 0;
}

template<typename T>
float getVal8(const uint8_t *src, const OutputTensorApParams &param)
{
	T temp = (T)*src;
	float value = (temp - param.shift) * param.scale;
	return value;
}

template<typename T>
float getVal16(const uint8_t *src, const OutputTensorApParams &param)
{
	T temp = (((T) * (src + 1)) & 0xff) << 8 | (*src & 0xff);
	float value = (temp - param.shift) * param.scale;
	return value;
}

template<typename T>
float getVal32(const uint8_t *src, const OutputTensorApParams &param)
{
	T temp = (((T) * (src + 3)) & 0xff) << 24 | (((T) * (src + 2)) & 0xff) << 16 |
		 (((T) * (src + 1)) & 0xff) << 8 | (*src & 0xff);
	float value = (temp - param.shift) * param.scale;
	return value;
}

int parseOutputTensorBody(IMX500OutputTensorInfo &outputTensorInfo, const uint8_t *src,
			  const std::vector<OutputTensorApParams> &outputApParams,
			  const DnnHeader &dnnHeader)
{
	float *dst = outputTensorInfo.data.get();
	int ret = 0;

	if (outputTensorInfo.totalSize > (std::numeric_limits<uint32_t>::max() / sizeof(float))) {
		// LOG(IMX500, Error) << "totalSize is greater than maximum size";
        ESP_LOGE(TAG, "totalSize is greater than maximum size");
		return -1;
	}

	// Debug: Output the first 256 bytes of the src buffer
    // ESP_LOGI(TAG, "First %d bytes of src buffer:", 640);
    // for (int i = 0; i < 640; i += 16) {
    //     char line[3 * 16 + 1] = {0}; // Buffer to hold the formatted line
    //     int pos = 0;
    //     for (int j = 0; j < 16 && (i + j) < 640; ++j) {
    //         pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", src[i + j]);
    //     }
    //     ESP_LOGI(TAG, "%04X: %s", i, line);
    // }

	std::unique_ptr<float[]> tmpDst = std::make_unique<float[]>(outputTensorInfo.totalSize);
	std::vector<uint16_t> numLinesVec(outputApParams.size());
	std::vector<uint32_t> outSizes(outputApParams.size());
	std::vector<uint32_t> offsets(outputApParams.size());
	std::vector<const uint8_t *> srcArr(outputApParams.size());
	std::vector<std::vector<Dimensions>> serializedDims;
	std::vector<std::vector<Dimensions>> actualDims;

	const uint8_t *src1 = src;
	uint32_t offset = 0;
	std::vector<Dimensions> serializedDimT;
	std::vector<Dimensions> actualDimT;

	for (unsigned int tensorIdx = 0; tensorIdx < outputApParams.size(); tensorIdx++) {
		offsets[tensorIdx] = offset;
		srcArr[tensorIdx] = src1;
		uint32_t tensorDataNum = 0;

		const OutputTensorApParams &param = outputApParams.at(tensorIdx);
		uint32_t outputTensorSize = 0;
		uint32_t tensorOutSize = (param.bitsPerElement / 8);

		serializedDimT.resize(param.numDimensions);
		actualDimT.resize(param.numDimensions);

		for (int idx = 0; idx < param.numDimensions; idx++) {
			actualDimT[idx].size = param.vecDim.at(idx).size;
			serializedDimT[param.vecDim.at(idx).serializationIndex].size = param.vecDim.at(idx).size;

			tensorOutSize *= param.vecDim.at(idx).size;
			if (tensorOutSize >= std::numeric_limits<uint32_t>::max() / param.bitsPerElement / 8) {
				// LOG(IMX500, Error) << "Invalid output tensor info";
                ESP_LOGE(TAG, "Invalid output tensor info");
				return -1;
			}

			actualDimT[idx].serializationIndex = param.vecDim.at(idx).serializationIndex;
			serializedDimT[param.vecDim.at(idx).serializationIndex].serializationIndex =
				static_cast<uint8_t>(idx);
		}

		uint16_t numLines = std::ceil(tensorOutSize / static_cast<float>(dnnHeader.maxLineLen));
		outputTensorSize = tensorOutSize;
		numLinesVec[tensorIdx] = numLines;
		outSizes[tensorIdx] = tensorOutSize;

		serializedDims.push_back(serializedDimT);
		actualDims.push_back(actualDimT);

		src1 += numLines * TensorStride;
		tensorDataNum = (outputTensorSize / (param.bitsPerElement / 8));
		offset += tensorDataNum;
		outputTensorInfo.tensorDataNum[tensorIdx] = tensorDataNum;
		if (offset > outputTensorInfo.totalSize) {
			// LOG(IMX500, Error)
			// 	<< "Error in parsing output tensor offset " << offset << " > output_size";
            ESP_LOGE(TAG, "Error in parsing output tensor offset %u > output_size", offset);
			return -1;
		}
	}

	std::vector<uint32_t> idxs(outputApParams.size());
	for (unsigned int i = 0; i < idxs.size(); i++)
		idxs[i] = i;

	for (unsigned int i = 0; i < idxs.size(); i++) {
		for (unsigned int j = 0; j < idxs.size(); j++) {
			if (numLinesVec[idxs[i]] > numLinesVec[idxs[j]])
				std::swap(idxs[i], idxs[j]);
		}
	}

	std::vector<std::future<int>> futures;
	for (unsigned int ii = 0; ii < idxs.size(); ii++) {
		uint32_t idx = idxs[ii];
		futures.emplace_back(std::async(
			std::launch::async,
			[&tmpDst, &outSizes, &numLinesVec, &actualDims, &serializedDims,
			 &outputApParams, &dnnHeader, dst](int tensorIdx, const uint8_t *tsrc, int toffset) -> int {
				uint32_t outputTensorSize = outSizes[tensorIdx];
				uint16_t numLines = numLinesVec[tensorIdx];
				bool sortingRequired = false;

				const OutputTensorApParams &param = outputApParams[tensorIdx];
				const std::vector<Dimensions> &serializedDim = serializedDims[tensorIdx];
				const std::vector<Dimensions> &actualDim = actualDims[tensorIdx];

				for (unsigned i = 0; i < param.numDimensions; i++) {
					if (param.vecDim.at(i).serializationIndex != param.vecDim.at(i).ordinal)
						sortingRequired = true;
				}

				if (!outputTensorSize) {
					// LOG(IMX500, Error) << "Invalid output tensorsize (0)";
                    ESP_LOGE(TAG, "Invalid output tensorsize (0)");
					return -1;
				}

				/* Extract output tensor data */
				uint32_t elementIndex = 0;
				if (param.bitsPerElement == 8) {
					for (unsigned int i = 0; i < numLines; i++) {
						int lineIndex = 0;
						while (lineIndex < dnnHeader.maxLineLen) {
							if (param.format == TensorDataType::Signed)
								tmpDst[toffset + elementIndex] =
									getVal8<int8_t>(tsrc + lineIndex, param);
							else
								tmpDst[toffset + elementIndex] =
									getVal8<uint8_t>(tsrc + lineIndex, param);
							elementIndex++;
							lineIndex++;
							if (elementIndex == outputTensorSize)
								break;
						}
						tsrc += TensorStride;
						if (elementIndex == outputTensorSize)
							break;
					}
				} else if (param.bitsPerElement == 16) {
					for (unsigned int i = 0; i < numLines; i++) {
						int lineIndex = 0;
						while (lineIndex < dnnHeader.maxLineLen) {
							if (param.format == TensorDataType::Signed)
								tmpDst[toffset + elementIndex] =
									getVal16<int16_t>(tsrc + lineIndex, param);
							else
								tmpDst[toffset + elementIndex] =
									getVal16<uint16_t>(tsrc + lineIndex, param);
							elementIndex++;
							lineIndex += 2;
							if (elementIndex >= (outputTensorSize >> 1))
								break;
						}
						tsrc += TensorStride;
						if (elementIndex >= (outputTensorSize >> 1))
							break;
					}
				} else if (param.bitsPerElement == 32) {
					for (unsigned int i = 0; i < numLines; i++) {
						int lineIndex = 0;
						while (lineIndex < dnnHeader.maxLineLen) {
							if (param.format == TensorDataType::Signed)
								tmpDst[toffset + elementIndex] =
									getVal32<int32_t>(tsrc + lineIndex, param);
							else
								tmpDst[toffset + elementIndex] =
									getVal32<uint32_t>(tsrc + lineIndex, param);
							elementIndex++;
							lineIndex += 4;
							if (elementIndex >= (outputTensorSize >> 2))
								break;
						}
						tsrc += TensorStride;
						if (elementIndex >= (outputTensorSize >> 2))
							break;
					}
				}

				/*
				 * Sorting in order according to AP Params. Not supported if larger than 3D
				 * Preparation:
				 */
				if (sortingRequired) {
					constexpr unsigned int DimensionMax = 3;

					std::array<uint32_t, DimensionMax> loopCnt{ 1, 1, 1 };
					std::array<uint32_t, DimensionMax> coef{ 1, 1, 1 };
					for (unsigned int i = 0; i < param.numDimensions; i++) {
						if (i >= DimensionMax) {
							// LOG(IMX500, Error) << "numDimensions value is 3 or higher";
                            ESP_LOGE(TAG, "numDimensions value is 3 or higher");
							break;
						}

						loopCnt[i] = serializedDim.at(i).size;

						for (unsigned int j = serializedDim.at(i).serializationIndex; j > 0; j--)
							coef[i] *= actualDim.at(j - 1).size;
					}
					/* Sort execution */
					unsigned int srcIndex = 0;
					unsigned int dstIndex;
					for (unsigned int i = 0; i < loopCnt[DimensionMax - 1]; i++) {
						for (unsigned int j = 0; j < loopCnt[DimensionMax - 2]; j++) {
							for (unsigned int k = 0; k < loopCnt[DimensionMax - 3]; k++) {
								dstIndex = (coef[DimensionMax - 1] * i) +
									   (coef[DimensionMax - 2] * j) +
									   (coef[DimensionMax - 3] * k);
								dst[toffset + dstIndex] = tmpDst[toffset + srcIndex++];
							}
						}
					}
				} else {
					if (param.bitsPerElement == 8)
						memcpy(dst + toffset, tmpDst.get() + toffset,
						       outputTensorSize * sizeof(float));
					else if (param.bitsPerElement == 16)
						memcpy(dst + toffset, tmpDst.get() + toffset,
						       (outputTensorSize >> 1) * sizeof(float));
					else if (param.bitsPerElement == 32)
						memcpy(dst + toffset, tmpDst.get() + toffset,
						       (outputTensorSize >> 2) * sizeof(float));
					else {
						// LOG(IMX500, Error)
						// 	<< "Invalid bitsPerElement value =" << param.bitsPerElement;
                        ESP_LOGE(TAG, "Invalid bitsPerElement value = %u", param.bitsPerElement);
						return -1;
					}
				}

				return 0;
			},
			idx, srcArr[idx], offsets[idx]));
	}

	for (auto &f : futures)
		ret += f.get();

	return ret;
}

int parseInputApParams(InputTensorApParams &inputApParams, std::vector<uint8_t> &apParams,
		       const DnnHeader &dnnHeader)
{
	const apParams::fb::FBApParams *fbApParams;
	const apParams::fb::FBNetwork *fbNetwork;
	const apParams::fb::FBInputTensor *fbInputTensor;

	// std::vector<uint8_t> bytesToInsert = {0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00};
	// apParams.insert(apParams.begin() + 516, bytesToInsert.begin(), bytesToInsert.end());

	// Remove the last 8 bytes
	// if (apParams.size() >= 8) {
	// 	apParams.erase(apParams.end() - 8, apParams.end());
	// }

	// ESP_LOGI(TAG, "apParams size: %u", apParams.size());
	// ESP_LOGI(TAG, "apParams contents:");
	// for (size_t i = 0; i < apParams.size(); i += 16) {
	// 	char line[3 * 16 + 1] = {0}; // Buffer to hold the formatted line
	// 	int pos = 0;
	// 	for (size_t j = 0; j < 16 && (i + j) < apParams.size(); ++j) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", apParams[i + j]);
	// 	}
	// 	ESP_LOGI(TAG, "%04X: %s", static_cast<unsigned int>(i), line);
	// }

	fbApParams = apParams::fb::GetFBApParams(apParams.data());
	// LOG(IMX500, Debug) << "Networks size: " << fbApParams->networks()->size();
    // ESP_LOGI(TAG, "Inpupt Networks size: %u", fbApParams->networks()->size());

	for (unsigned int i = 0; i < fbApParams->networks()->size(); i++) {
		fbNetwork = reinterpret_cast<const apParams::fb::FBNetwork *>(fbApParams->networks()->Get(i));
		if (fbNetwork->id() != dnnHeader.networkId)
			continue;

		// LOG(IMX500, Debug)
		// 	<< "Network: " << fbNetwork->type()->c_str()
		// 	<< ", i/p size: " << fbNetwork->inputTensors()->size()
		// 	<< ", o/p size: " << fbNetwork->outputTensors()->size();
        ESP_LOGI(TAG, "Input Network: %s, i/p size: %u, o/p size: %u", fbNetwork->type()->c_str(), 
                        fbNetwork->inputTensors()->size(), fbNetwork->outputTensors()->size());

		inputApParams.networkName = fbNetwork->type()->str();
		fbInputTensor =
			reinterpret_cast<const apParams::fb::FBInputTensor *>(fbNetwork->inputTensors()->Get(0));

		// LOG(IMX500, Debug)
		// 	<< "Input Tensor shift: " << fbInputTensor->shift()
		// 	<< ", Scale: scale: " << fbInputTensor->scale()
		// 	<< ", Format: " << static_cast<int>(fbInputTensor->format());
        // ESP_LOGI(TAG, "Input Tensor shift: %d, Scale: %f, Format: %d", fbInputTensor->shift(), 
        //                 fbInputTensor->scale(), static_cast<int>(fbInputTensor->format()));

		if (fbInputTensor->dimensions()->size() != 3) {
			// LOG(IMX500, Error) << "Invalid number of dimensions in InputTensor";
            ESP_LOGE(TAG, "number of dimensions in InputTensor is %d", fbInputTensor->dimensions()->size());
            ESP_LOGE(TAG, "Invalid number of dimensions in InputTensor");
			return -1;
		}

		for (unsigned int j = 0; j < fbInputTensor->dimensions()->size(); j++) {
			switch (fbInputTensor->dimensions()->Get(j)->serializationIndex()) {
			case 0:
				inputApParams.width = fbInputTensor->dimensions()->Get(j)->size();
				inputApParams.widthStride =
					inputApParams.width + fbInputTensor->dimensions()->Get(j)->padding();
				break;
			case 1:
				inputApParams.height = fbInputTensor->dimensions()->Get(j)->size();
				inputApParams.heightStride =
					inputApParams.height + fbInputTensor->dimensions()->Get(j)->padding();
				break;
			case 2:
				inputApParams.channel = fbInputTensor->dimensions()->Get(j)->size();
				break;
			default:
				// LOG(IMX500, Error) << "Invalid dimension in InputTensor " << j;
                ESP_LOGE(TAG, "Invalid number of dimensions in InputTensor, index=%d",
								fbInputTensor->dimensions()->Get(j)->serializationIndex());
				break;
			}
		}
	}

	return 0;
}

int parseInputTensorBody(IMX500InputTensorInfo &inputTensorInfo, const uint8_t *src,
			 const InputTensorApParams &inputApParams, const DnnHeader &dnnHeader)
{
	if ((inputApParams.width > InputSensorMaxWidth) || (inputApParams.height > InputSensorMaxHeight) ||
	    ((inputApParams.channel != 1) && (inputApParams.channel != 3) && (inputApParams.channel != 4))) {
		// LOG(IMX500, Error)
		// 	<< "Invalid input tensor size w: " << inputApParams.width
		// 	<< " h: " << inputApParams.height
		// 	<< " c: " << inputApParams.channel;
        ESP_LOGE(TAG, "Invalid input tensor size w: %u h: %u c: %u",
            inputApParams.width, inputApParams.height, inputApParams.channel);
		return -1;
	}

	unsigned int outSize = inputApParams.width * inputApParams.height * inputApParams.channel;
	unsigned int outSizePadded = inputApParams.widthStride * inputApParams.heightStride * inputApParams.channel;
	unsigned int numLines = std::ceil(outSizePadded / static_cast<float>(dnnHeader.maxLineLen));
	inputTensorInfo.data = std::shared_ptr<uint8_t[]>(new uint8_t[outSize]);

	unsigned int diff = 0, outLineIndex = 0, pixelIndex = 0, heightIndex = 0, size = 0, left = 0;
	unsigned int wPad = inputApParams.widthStride - inputApParams.width;
	unsigned int hPad = inputApParams.heightStride - inputApParams.height;

	for (unsigned int line = 0; line < numLines; line++) {
		for (unsigned int lineIndex = diff; lineIndex < dnnHeader.maxLineLen; lineIndex += size) {
			if (outLineIndex == inputApParams.width) { /* Skip width padding pixels */
				outLineIndex = 0;
				heightIndex++;
				lineIndex += wPad;
				if (lineIndex >= dnnHeader.maxLineLen) {
					diff = lineIndex - dnnHeader.maxLineLen;
					break;
				} else
					diff = 0;
			}

			if (heightIndex == inputApParams.height) { /* Skip height padding pixels */
				lineIndex += hPad * inputApParams.widthStride;
				heightIndex = 0;
				if (lineIndex >= dnnHeader.maxLineLen) {
					diff = lineIndex - dnnHeader.maxLineLen;
					while (diff >= dnnHeader.maxLineLen) {
						diff -= dnnHeader.maxLineLen;
						src += TensorStride;
						line++;
					}
					break;
				} else
					diff = 0;
			}

			if (((pixelIndex == inputApParams.width * inputApParams.height) ||
			     (pixelIndex == inputApParams.width * inputApParams.height * 2) ||
			     (pixelIndex == inputApParams.width * inputApParams.height * 3))) {
				if (pixelIndex == outSize)
					break;
			}

			if (left > 0) {
				size = left;
				left = 0;
			} else if (pixelIndex + inputApParams.width >= outSize) {
				size = outSize - pixelIndex;
			} else if (lineIndex + inputApParams.width >= dnnHeader.maxLineLen) {
				size = dnnHeader.maxLineLen - lineIndex;
				left = inputApParams.width - size;
			} else {
				size = inputApParams.width;
			}

			memcpy(&inputTensorInfo.data[pixelIndex], src + lineIndex, size);
			pixelIndex += size;
			outLineIndex += size;
		}

		if (pixelIndex == outSize)
			break;

		src += TensorStride;
	}

	inputTensorInfo.size = outSize;
	inputTensorInfo.width = inputApParams.width;
	inputTensorInfo.height = inputApParams.height;
	inputTensorInfo.channels = inputApParams.channel;
	inputTensorInfo.widthStride = inputApParams.widthStride;
	inputTensorInfo.heightStride = inputApParams.heightStride;
	inputTensorInfo.networkName = inputApParams.networkName;

	// ESP_LOGI(TAG, "parseInputTensorBody size=%d, width=%d, height=%d, channel=%d, widthStride=%d, heightStride=%d, networkName=%s",
	// 	inputTensorInfo.size, inputTensorInfo.width, inputTensorInfo.height, inputTensorInfo.channels,
	// 	inputTensorInfo.widthStride, inputTensorInfo.heightStride, inputTensorInfo.networkName.c_str());

	return 0;
}

} /* namespace */

int RPiController::imx500ParseInputTensor(IMX500InputTensorInfo &inputTensorInfo,
					  libcamera::Span<const uint8_t> inputTensor)
{
	DnnHeader dnnHeader;
	std::vector<uint8_t> apParams;
	InputTensorApParams inputApParams{};

	const uint8_t *src = inputTensor.data();
	int ret = parseHeader(dnnHeader, apParams, src);
	if (ret) {
		// LOG(IMX500, Error) << "Header param parsing failed!";
        ESP_LOGE(TAG, "Header param parsing failed!");
		return ret;
	}

	if (dnnHeader.tensorType != TensorType::InputTensor) {
		// LOG(IMX500, Error) << "Invalid input tensor type in AP params!";
        ESP_LOGE(TAG, "Invalid input tensor type in AP params!");
		return -1;
	}

	ret = parseInputApParams(inputApParams, apParams, dnnHeader);
	if (ret) {
		// LOG(IMX500, Error) << "AP param parsing failed!";
        ESP_LOGE(TAG, "AP param parsing failed!"); 
		return ret;
	}

	ret = parseInputTensorBody(inputTensorInfo, src + TensorStride, inputApParams, dnnHeader);
	if (ret) {
		// LOG(IMX500, Error) << "Input tensor body parsing failed!";
        ESP_LOGE(TAG, "Input tensor body parsing failed!");
		return ret;
	}

	return 0;
}

int RPiController::imx500ParseOutputTensor(IMX500OutputTensorInfo &outputTensorInfo,
					   Span<const uint8_t> outputTensor)
{
	DnnHeader dnnHeader;
	std::vector<uint8_t> apParams;
	std::vector<OutputTensorApParams> outputApParams;

	const uint8_t *src = outputTensor.data();
	int ret = parseHeader(dnnHeader, apParams, src);
	if (ret) {
		// LOG(IMX500, Error) << "Header param parsing failed!";
        ESP_LOGE(TAG, "Header param parsing failed!");
		return ret;
	}

	if (dnnHeader.tensorType != TensorType::OutputTensor) {
		// LOG(IMX500, Error) << "Invalid output tensor type in AP params!";
        ESP_LOGE(TAG, "Invalid output tensor type in AP params!");
		return -1;
	}

	ret = parseOutputApParams(outputApParams, apParams, dnnHeader);
	if (ret) {
		// LOG(IMX500, Error) << "AP param parsing failed!";
        ESP_LOGE(TAG, "AP param parsing failed!");
		return ret;
	}

	ret = populateOutputTensorInfo(outputTensorInfo, outputApParams);
	if (ret) {
		// LOG(IMX500, Error) << "Failed to populate OutputTensorInfo!";
        ESP_LOGE(TAG, "Failed to populate OutputTensorInfo!");
		return ret;
	}

	ret = parseOutputTensorBody(outputTensorInfo, src + TensorStride, outputApParams, dnnHeader);
	if (ret) {
		// LOG(IMX500, Error) << "Output tensor body parsing failed!";
        ESP_LOGE(TAG, "Output tensor body parsing failed!");
		return ret;
	}

	return 0;
}

std::unordered_map<TensorType, IMX500Tensors> RPiController::imx500SplitTensors(Span<const uint8_t> tensors)
{
	const DnnHeader *outputHeader;
	DnnHeader inputHeader;
	std::unordered_map<TensorType, IMX500Tensors> offsets;

	/*
	 * Structure of the IMX500 DNN output:
	 * Line 0: KPI params
	 * Line [1, x): Input tensor
	 * Line [x, N-1): Output tensor
	 * Line N-1: PQ params
	 */
	offsets[TensorType::Kpi].offset = 0;

	// const uint8_t *src = tensors.data() + 2544;	//TensorStride;
	const uint8_t *src = tensors.data() + TensorStride;

    // ESP_LOGI(TAG, "First 256 bytes of src buffer:");
    // for (int i = 0; i < 256; i += 16) {
    //     char line[3 * 16 + 16] = {0};
    //     int pos = 0;
    //     for (int j = 0; j < 16 && (i + j) < 256; ++j) {
    //         pos += sprintf(&line[pos], "%02X ", src[i + j]);
    //     }
    //     ESP_LOGI(TAG, "%04X: %s", i, line);
    // }


	inputHeader = *reinterpret_cast<const DnnHeader *>(src);
	if (inputHeader.tensorType != TensorType::InputTensor) {
		// LOG(IMX500, Debug) << "Input tensor is invalid, arborting.";
        ESP_LOGI(TAG, "Input tensor is invalid, aborting.");
		return {};
	}

    // ESP_LOGI(TAG, "inputHeader: valid %d, count %d, maxLineLen %u, apParamSize %u, networkId %u, tensorType %d",
    // static_cast<bool>(inputHeader.frameValid),
    // static_cast<int>(inputHeader.frameCount),
    // inputHeader.maxLineLen,
    // inputHeader.apParamSize,
    // inputHeader.networkId,
    // static_cast<int>(inputHeader.tensorType));

	offsets[TensorType::InputTensor].offset = TensorStride;
	offsets[TensorType::InputTensor].valid = inputHeader.frameValid;
	// LOG(IMX500, Debug)
	// 	<< "Found input tensor at offset: " << offsets[TensorType::InputTensor].offset
	// 	<< ", valid: " << static_cast<unsigned int>(offsets[TensorType::InputTensor].valimx500SplitTensorsid);
    ESP_LOGI(TAG, "Found input tensor at offset: %u, valid: %u",
        offsets[TensorType::InputTensor].offset,
        static_cast<unsigned int>(offsets[TensorType::InputTensor].valid));

	// src += 307376;    //TensorStride;
	src += TensorStride;

	while (src < tensors.data() + tensors.size()) {
		outputHeader = reinterpret_cast<const DnnHeader *>(src);

        // ESP_LOGI(TAG, "outputHeader: valid %d, count %d, maxLineLen %u, apParamSize %u, networkId %u, tensorType %d",
        //     static_cast<bool>(outputHeader->frameValid),
        //     static_cast<int>(outputHeader->frameCount),
        //     outputHeader->maxLineLen,
        //     outputHeader->apParamSize,
        //     outputHeader->networkId,
        //     static_cast<int>(outputHeader->tensorType));

		if (outputHeader->frameCount == inputHeader.frameCount &&
		    outputHeader->apParamSize == inputHeader.apParamSize &&
		    outputHeader->maxLineLen == inputHeader.maxLineLen &&
		    outputHeader->tensorType == TensorType::OutputTensor) {
			offsets[TensorType::OutputTensor].offset = src - tensors.data();
			offsets[TensorType::OutputTensor].valid = outputHeader->frameValid;
			// LOG(IMX500, Debug)
			// 	<< "Found output tensor at offset: " << offsets[TensorType::OutputTensor].offset
			// 	<< ", valid: " << static_cast<unsigned int>(offsets[TensorType::OutputTensor].valid);
            ESP_LOGI(TAG, "Found output tensor at offset: %u, valid: %u",
                offsets[TensorType::OutputTensor].offset,
                static_cast<unsigned int>(offsets[TensorType::OutputTensor].valid));
			break;
		}
		src += TensorStride;
	}

	return offsets;
}

// void RPiController::parseInferenceData(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
void RPiController::imx500ParseInferenceData(libcamera::Span<const uint8_t> buffer)
{
	/* Inference data comes after 2 lines of embedded data. */
	constexpr unsigned int StartLine = 2;
	// size_t bytesPerLine = (mode_.width * mode_.bitdepth) >> 3;
	size_t bytesPerLine = (1024 * 8) >> 3;
	// if (hwConfig_.cfeDataBufferStrided)
	if (true)
		bytesPerLine = (bytesPerLine + 15) & ~15;

	if (buffer.size() <= StartLine * bytesPerLine)
		return;

    std::unique_ptr<uint8_t[]> savedInputTensor_;
	/* Check if an input tensor is needed - this is sticky! */
	// bool enableInputTensor = false;
	// metadata.get("cnn.enable_input_tensor", enableInputTensor);

	/* Cache the DNN metadata for fast parsing. */
	// unsigned int tensorBufferSize = buffer.size() - (StartLine * bytesPerLine);
	unsigned int tensorBufferSize = buffer.size();
	std::unique_ptr<uint8_t[]> cache = std::make_unique<uint8_t[]>(tensorBufferSize);
	// memcpy(cache.get(), buffer.data() + StartLine * bytesPerLine, tensorBufferSize);
	memcpy(cache.get(), buffer.data(), tensorBufferSize);
	Span<const uint8_t> tensors(cache.get(), tensorBufferSize);

	std::unordered_map<TensorType, IMX500Tensors> offsets = RPiController::imx500SplitTensors(tensors);
	auto itIn = offsets.find(TensorType::InputTensor);
	auto itOut = offsets.find(TensorType::OutputTensor);

	if (itIn != offsets.end() && itOut != offsets.end()) {
		const unsigned int inputTensorOffset = itIn->second.offset;
		const unsigned int outputTensorOffset = itOut->second.offset;
		const unsigned int inputTensorSize = outputTensorOffset - inputTensorOffset;
		Span<const uint8_t> inputTensor;

		if (itIn->second.valid) {
			if (itOut->second.valid) {
				/* Valid input and output tensor, get the span directly from the current cache. */
				// ESP_LOGI(TAG, "inputTensorOffset=%d, outputTensorOffset=%d, inputTensorSize=%d",
				// 			inputTensorOffset, outputTensorOffset, inputTensorSize);
				inputTensor = Span<const uint8_t>(cache.get() + inputTensorOffset,
								  inputTensorSize);
			} else {
				/*
				 * Invalid output tensor with valid input tensor.
				 * This is likely because the DNN takes longer than
				 * a frame time to generate the output tensor.
				 *
				 * In such cases, we don't process the input tensor,
				 * but simply save it for when the next output
				 * tensor is valid. This way, we ensure that both
				 * valid input and output tensors are in lock-step.
				 */
				savedInputTensor_ = std::make_unique<uint8_t[]>(inputTensorSize);
				memcpy(savedInputTensor_.get(), cache.get() + inputTensorOffset,
				       inputTensorSize);
			}
		} else if (itOut->second.valid && savedInputTensor_) {
			/*
			 * Invalid input tensor with valid output tensor. This is
			 * likely because the DNN takes longer than a frame time
			 * to generate the output tensor.
			 *
			 * In such cases, use the previously saved input tensor
			 * if possible.
			 */
			inputTensor = Span<const uint8_t>(savedInputTensor_.get(), inputTensorSize);
		}

		if (inputTensor.size()) {
			IMX500InputTensorInfo inputTensorInfo;
			// ESP_LOGI(TAG, "inputTensor size=%d", inputTensor.size());
			if (!imx500ParseInputTensor(inputTensorInfo, inputTensor)) {
				CnnInputTensorInfo exported{};
				exported.width = inputTensorInfo.width;
				exported.height = inputTensorInfo.height;
				exported.numChannels = inputTensorInfo.channels;
				strncpy(exported.networkName, inputTensorInfo.networkName.c_str(),
					sizeof(exported.networkName));
				exported.networkName[sizeof(exported.networkName) - 1] = '\0';
				dnnMetadata.set("cnn.input_tensor_info", exported);
				dnnMetadata.set("cnn.input_tensor", std::move(inputTensorInfo.data));
				dnnMetadata.set("cnn.input_tensor_size", inputTensorInfo.size);
			}

			/* We can now safely clear the saved input tensor. */
			savedInputTensor_.reset();
		}
	}

	if (itOut != offsets.end() && itOut->second.valid) {
		unsigned int outputTensorOffset = itOut->second.offset;
		Span<const uint8_t> outputTensor(cache.get() + outputTensorOffset,
						 tensorBufferSize - outputTensorOffset);

		IMX500OutputTensorInfo outputTensorInfo;
		if (!imx500ParseOutputTensor(outputTensorInfo, outputTensor)) {
			CnnOutputTensorInfo exported{};
			if (outputTensorInfo.numTensors < MaxNumTensors) {
				exported.numTensors = outputTensorInfo.numTensors;
                // ESP_LOGI(TAG, "outputTensor numTensors=%d", exported.numTensors);
				for (unsigned int i = 0; i < exported.numTensors; i++) {
					exported.info[i].tensorDataNum = outputTensorInfo.tensorDataNum[i];
					exported.info[i].numDimensions = outputTensorInfo.numDimensions[i];
					for (unsigned int j = 0; j < exported.info[i].numDimensions; j++)
						exported.info[i].size[j] = outputTensorInfo.vecDim[i][j].size;
				}
			} else {
				// LOG(IPARPI, Debug)
				// 	<< "IMX500 output tensor info export failed, numTensors > MaxNumTensors";
                ESP_LOGI(TAG, "IMX500 output tensor info export failed, numTensors > MaxNumTensors");
			}
			strncpy(exported.networkName, outputTensorInfo.networkName.c_str(),
				sizeof(exported.networkName));
			exported.networkName[sizeof(exported.networkName) - 1] = '\0';
			dnnMetadata.set("cnn.output_tensor_info", exported);
			dnnMetadata.set("cnn.output_tensor", std::move(outputTensorInfo.data));
			dnnMetadata.set("cnn.output_tensor_size", outputTensorInfo.totalSize);

			auto itKpi = offsets.find(TensorType::Kpi);
			if (itKpi != offsets.end()) {
				constexpr unsigned int DnnRuntimeOffset = 9;
				constexpr unsigned int DspRuntimeOffset = 10;
				CnnKpiInfo kpi;

				uint8_t *k = cache.get() + itKpi->second.offset;
				kpi.dnnRuntime = k[4 * DnnRuntimeOffset + 3] << 24 |
						 k[4 * DnnRuntimeOffset + 2] << 16 |
						 k[4 * DnnRuntimeOffset + 1] << 8 |
						 k[4 * DnnRuntimeOffset];
				kpi.dspRuntime = k[4 * DspRuntimeOffset + 3] << 24 |
						 k[4 * DspRuntimeOffset + 2] << 16 |
						 k[4 * DspRuntimeOffset + 1] << 8 |
						 k[4 * DspRuntimeOffset];
				dnnMetadata.set("cnn.kpi_info", kpi);
			}
		}
	}
}


extern "C" 
void imx500_parse_inference_data(const uint8_t *buffer, size_t buffer_size)
{
    // ESP_LOGI(TAG, "imx500_parse_inference_data called with buffer size: %d", buffer_size);

    libcamera::Span<const uint8_t> dnnData(buffer, buffer_size);

	// Parse the inference data
    ESP_LOGI(TAG, "imx500 parse inference data...");
    RPiController::imx500ParseInferenceData(dnnData);
	
	// Parse the inference data
    ESP_LOGI(TAG, "imx500 ojbect detection process...");
	RPiController::imx500ObjectDetectionProcess(RPiController::dnnMetadata);

	// RPiController::imx500ObjectDetectDraw();
}

extern "C"
void imx500_draw_detect_object(uint16_t *buffer, uint16_t width, uint16_t height)
{
	RPiController::imx500ObjectDetectDraw(buffer, width, height);
}
