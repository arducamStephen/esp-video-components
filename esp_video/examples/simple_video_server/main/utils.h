/* Copyright [2025] arducam */
#ifndef ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_MODEL_MODEL_ID_7_HIGHERHRNET_RPI_UTILS_H_
#define ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_MODEL_MODEL_ID_7_HIGHERHRNET_RPI_UTILS_H_

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>
#include <tuple>
#include <cstdint>

namespace arducam {

using std::vector;
using std::array;
using std::pair;
using std::make_pair;
using std::tuple;

// 网络输出结构体
struct NetworkOutputs {
    vector<vector<float>> tag; // 尺寸: [num_joints][max_num_people]
    vector<vector<int32_t>> ind;   // 尺寸: [num_joints][max_num_people]
    vector<vector<float>> val; // 尺寸: [num_joints][max_num_people]
};

// 默认关节点排序（共 17 个关节点）  
static const vector<int32_t> default_joint_order = { 0, 1, 2, 3, 4, 5, 6, 11, 12, 7, 8, 9, 10, 13, 14, 15, 16 };

tuple<vector<vector<float>>, vector<float>, vector<vector<float>>>
postprocess_higherhrnet(const NetworkOutputs& outputs,
    const pair<int32_t, int32_t>& img_size,
    const pair<int32_t, int32_t>& img_w_pad,
    const pair<int32_t, int32_t>& img_h_pad,
    float detection_threshold,
    bool network_postprocess,
    const pair<int32_t, int32_t>& input_image_size,
    const pair<int32_t, int32_t>& output_shape,
    int32_t num_joints = 17,
    const vector<int32_t>& joint_order = default_joint_order,
    int32_t max_num_people = 30,
    bool ignore_too_much = false,
    bool use_detection_val = true,
    float tag_threshold = 1.0f);

}  // arducam

#endif  // ARDUCAM_AI_POSTPROCESS_INCLUDE_ARDUCAM_MODEL_MODEL_ID_7_HIGHERHRNET_RPI_UTILS_H_
