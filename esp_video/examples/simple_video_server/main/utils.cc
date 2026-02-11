#include "munkres.hpp"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>
#include <tuple>
#include <cstdint>

using std::vector;
using std::array;
using std::pair;
using std::make_pair;
using std::tuple;

static std::vector<std::pair<int32_t, int32_t>> hungarian(const std::vector<std::vector<float>>& cost) {
    int32_t n = cost.size();
    int32_t m = cost[0].size();
    int32_t N = std::max(n, m);
    // 构造 (N+1)x(N+1) 代价矩阵，所有下标从 1..N
    std::vector<std::vector<float>> a(N + 1, std::vector<float>(N + 1, 0.0));
    for (int32_t i = 1; i <= n; ++i)
        for (int32_t j = 1; j <= m; ++j)
            a[i][j] = cost[i - 1][j - 1];

    std::vector<float> u(N + 1, 0.0), v(N + 1, 0.0);
    std::vector<int32_t> p(N + 1, 0), way(N + 1, 0);

    for (int32_t i = 1; i <= N; ++i) {
        p[0] = i;
        int32_t j0 = 0;
        std::vector<float> minv(N + 1, std::numeric_limits<float>::infinity());
        std::vector<bool> used(N + 1, false);

        do {
            used[j0] = true;
            int32_t i0 = p[j0], j1 = 0;
            float delta = std::numeric_limits<float>::infinity();
            for (int32_t j = 1; j <= N; ++j) {
                if (!used[j]) {
                    float cur = a[i0][j] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int32_t j = 0; j <= N; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int32_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    // 收集匹配结果：p[j] 行匹配到列 j
    std::vector<std::pair<int32_t, int32_t>> result;
    result.reserve(std::min(n, m));
    for (int32_t j = 1; j <= N; ++j) {
        int32_t i = p[j];
        if (i >= 1 && i <= n && j <= m) {
            result.emplace_back(i - 1, j - 1);
        }
    }
    return result;
}

std::vector<std::pair<int32_t, int32_t>> py_max_match(
    const std::vector<std::vector<float>>& scores) {
    Munkres munk;
    auto res = munk.compute(scores);
    return res;
}

//———————————————————————————————————————————————————————————————————————————————
// 用于保存某个候选关键点的信息
struct JointDetection {
    float x;
    float y;
    float score;
    float tag;
};

//———————————————————————————————————————————————————————————————————————————————
// matchByTag 函数
// 注意：此处假设传入的 tag 与 val 矩阵为转置存储，即尺寸为 [num_joints][max_num_people]，
//      而 loc 矩阵仍为 [max_num_people][num_joints]
pair<vector<vector<array<float, 4>>>, vector<float>>
matchByTag(const vector<vector<float>>& transposed_tag,
    const vector<vector<array<int32_t, 2>>>& loc,
    const vector<vector<float>>& transposed_val,
    int32_t num_joints,
    const vector<int32_t>& joint_order,
    float detection_threshold,
    int32_t max_num_people,
    bool ignore_too_much,
    bool use_detection_val,
    float tag_threshold) {
    vector<vector<array<float, 4>>> persons;
    vector<vector<float>> person_tags;

    // 遍历每个关节（按照 joint_order 顺序），注意这里 joint 的索引为 transposed 版中的行索引
    for (int32_t i = 0; i < num_joints; i++) {
        int32_t joint_idx = joint_order[i];
        vector<JointDetection> detections;
        // 遍历所有候选（内层：人数，对应转置矩阵的列）
        for (size_t p = 0; p < transposed_tag[0].size(); p++) {
            if (transposed_val[joint_idx][p] > detection_threshold) {
                JointDetection det;
                // 从 loc 矩阵中获取 (x, y) 坐标，loc 的排列为 [max_num_people][num_joints]
                det.x = loc[p][joint_idx][0];
                det.y = loc[p][joint_idx][1];
                det.score = transposed_val[joint_idx][p];
                det.tag = transposed_tag[joint_idx][p];
                detections.push_back(det);
            }
        }
        if (detections.empty())
            continue;

        // 第一关节或尚未创建人体记录时，直接将每个检测作为新人体
        if (i == 0 || persons.empty()) {
            for (auto& det : detections) {
                vector<array<float, 4>> person(num_joints, { 0, 0, 0, 0 });
                person[joint_idx] = { det.x, det.y, det.score, det.tag };
                persons.push_back(person);
                person_tags.push_back({ det.tag });
            }
        }
        else {
            int32_t num_persons = persons.size();
            int32_t num_detections = detections.size();

            // 如果已达到最大检测人数且 ignore_too_much 为 true，则跳过当前关节匹配
            if (ignore_too_much && num_persons >= max_num_people)
                continue;

            // 计算已有人体的平均 tag
            vector<float> avg_tags(num_persons, 0.0f);
            for (int32_t p = 0; p < num_persons; p++) {
                float sum = 0.0f;
                for (float t : person_tags[p]) {
                    sum += t;
                }
                avg_tags[p] = sum / person_tags[p].size();
            }

            // 构造 cost_matrix 与 diff_matrix（尺寸：num_detections x num_persons）
            vector<vector<float>> cost_matrix(num_detections, vector<float>(num_persons, 0));
            vector<vector<float>> diff_matrix(num_detections, vector<float>(num_persons, 0));
            for (int32_t d = 0; d < num_detections; d++) {
                for (int32_t p = 0; p < num_persons; p++) {
                    float diff = std::fabs(detections[d].tag - avg_tags[p]);
                    diff_matrix[d][p] = diff;
                    if (use_detection_val) {
                        int32_t rounded = static_cast<int32_t>(std::round(diff));
                        cost_matrix[d][p] = rounded * 100.0f - detections[d].score;
                    }
                    else {
                        cost_matrix[d][p] = diff;
                    }
                }
            }

            vector<vector<float>> cost;
            if (num_detections > num_persons) {
                cost.assign(num_detections, vector<float>(num_detections, 1e10f));
                for (int32_t d = 0; d < num_detections; ++d) {
                    for (int32_t p = 0; p < num_persons; ++p) {
                        cost[d][p] = cost_matrix[d][p];
                    }
                }
            }
            else {
                // 如果你的 hungarian 支持 D×P 矩阵，就直接用：
                cost = cost_matrix;
                // 否则，也可以 pad 行 (pad 到 P×P)，
                // 但要保证行为 Python 的 rectangular match 逻辑。
            }

            vector<pair<int32_t, int32_t>> assignments = py_max_match(cost);

            vector<bool> detection_assigned(num_detections, false);
            for (auto& assign : assignments) {
                int32_t d = assign.first;
                int32_t p = assign.second;
                if (d < num_detections && p < num_persons && diff_matrix[d][p] < tag_threshold) {
                    // 将检测 d 分配给已有人体 p
                    persons[p][joint_idx] = { detections[d].x, detections[d].y, detections[d].score, detections[d].tag };
                    person_tags[p].push_back(detections[d].tag);
                    detection_assigned[d] = true;
                }
                else {
                    // 匹配代价过大，新建一个人体记录
                    vector<array<float, 4>> person(num_joints, { 0, 0, 0, 0 });
                    person[joint_idx] = { detections[d].x, detections[d].y, detections[d].score, detections[d].tag };
                    persons.push_back(person);
                    person_tags.push_back({ detections[d].tag });
                    detection_assigned[d] = true;
                }
            }
            // 对于未匹配的检测，新建人体记录
            for (int32_t d = 0; d < num_detections; d++) {
                if (!detection_assigned[d]) {
                    vector<array<float, 4>> person(num_joints, { 0, 0, 0, 0 });
                    person[joint_idx] = { detections[d].x, detections[d].y, detections[d].score, detections[d].tag };
                    persons.push_back(person);
                    person_tags.push_back({ detections[d].tag });
                }
            }

        }
    }

    // 计算每个人体的平均得分（仅统计得分大于 0 的关节）
    vector<float> scores;
    for (auto& person : persons) {
        float sum = 0.0f;
        for (int32_t j = 0; j < num_joints; j++) {
            if (person[j][2] > 0) {
                sum += person[j][2];
            }
        }
        float avg = sum / num_joints;
        scores.push_back(avg);
    }

    return make_pair(persons, scores);
}

//———————————————————————————————————————————————————————————————————————————————
// 包装函数：由于 batch 维度为 1，直接调用 matchByTag
pair<vector<vector<array<float, 4>>>, vector<float>>
match(const vector<vector<float>>& transposed_tag,
    const vector<vector<array<int32_t, 2>>>& loc,
    const vector<vector<float>>& transposed_val,
    int32_t num_joints,
    const vector<int32_t>& joint_order,
    float detection_threshold,
    int32_t max_num_people,
    bool ignore_too_much,
    bool use_detection_val,
    float tag_threshold) {
    return matchByTag(transposed_tag, loc, transposed_val, num_joints, joint_order,
        detection_threshold, max_num_people, ignore_too_much, use_detection_val, tag_threshold);
}

//———————————————————————————————————————————————————————————————————————————————
// 后处理主函数
// 注意：本函数内部基于转置存储格式，所以获取人数数目取自 outputs.tag[0].size()
// 参数说明：
//   img_size: 原图尺寸 (height, width)
//   img_w_pad, img_h_pad: 分别为左右、上下 pad（第一项为左/上偏移，第二项为右/下）
//   input_image_size: 网络输入尺寸 (height, width)
//   output_shape: 网络输出特征图尺寸 (height, width)
tuple<vector<vector<float>>, vector<float>, vector<vector<float>>>
postprocess_higherhrnet(const HigherHRNetOutput& outputs,
    const pair<int32_t, int32_t>& img_size,
    const pair<int32_t, int32_t>& img_w_pad,
    const pair<int32_t, int32_t>& img_h_pad,
    float detection_threshold,
    bool network_postprocess,
    const pair<int32_t, int32_t>& input_image_size,
    const pair<int32_t, int32_t>& output_shape,
    int32_t num_joints,
    const vector<int32_t>& joint_order,
    int32_t max_num_people,
    bool ignore_too_much,
    bool use_detection_val,
    float tag_threshold) {
    // 由于数据转置存储，人数取 outputs.tag[0].size()
    int32_t maxN = outputs.tag.empty() ? 0 : outputs.tag[0].size();

    // 根据 outputs.ind 计算关键点的 (x, y) 坐标（从扁平化索引反算）
    // 这里 outputs.ind 为转置存储，所以访问应为 outputs.ind[j][i]
    vector<vector<array<int32_t, 2>>> loc(maxN, vector<array<int32_t, 2>>(num_joints));
    for (int32_t i = 0; i < maxN; i++) {
        for (int32_t j = 0; j < num_joints; j++) {
            int32_t idx = outputs.ind[j][i];
            // 输出热图尺寸为 output_shape = (height, width)
            int32_t x = idx % output_shape.second;
            int32_t y = idx / output_shape.second;
            loc[i][j] = { x, y };
        }
    }

    // 调用匹配函数：由于匹配函数内部假设 tag 与 val 为转置存储，
    // 直接传入 outputs.tag 与 outputs.val 即可
    auto match_result = match(outputs.tag, loc, outputs.val,
        num_joints, joint_order,
        detection_threshold, max_num_people,
        ignore_too_much, use_detection_val, tag_threshold);
    vector<vector<array<float, 4>>> persons = match_result.first;
    vector<float> scores = match_result.second;

    // 坐标缩放（与 Python 逻辑对应）  
    // 第一步：根据 input_image_size 与 output_shape 得到初步缩放因子  
    float scale_y = static_cast<float>(input_image_size.first) / output_shape.first;
    float scale_x = static_cast<float>(input_image_size.second) / output_shape.second;
    // 第二步：去除 pad 后，再按照原图与预处理图的比例进行缩放  
    for (auto& person : persons) {
        for (auto& joint : person) {
            joint[0] *= scale_x;
            joint[1] *= scale_y;
            // 去除预处理时左上角 pad
            joint[0] -= img_w_pad.first;
            joint[1] -= img_h_pad.first;
            // 根据 resized 后图像尺寸与原图比例进行最后缩放
            int32_t resized_w = input_image_size.second - (img_w_pad.first + img_w_pad.second);
            int32_t resized_h = input_image_size.first - (img_h_pad.first + img_h_pad.second);
            float s_x = static_cast<float>(img_size.second) / resized_w;
            float s_y = static_cast<float>(img_size.first) / resized_h;
            joint[0] *= s_x;
            joint[1] *= s_y;
        }
    }

    // 生成输出：平铺每个人体的关键点 (x,y,score) 与边框 [x_min, y_min, x_max, y_max]
    vector<vector<float>> out_keypoints;
    vector<float> out_scores;
    vector<vector<float>> out_bbox;
    for (size_t p = 0; p < persons.size(); p++) {
        vector<float> kp;
        kp.reserve(num_joints * 3);
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        for (int32_t j = 0; j < num_joints; j++) {
            float x = persons[p][j][0];
            float y = persons[p][j][1];
            float s = persons[p][j][2];
            kp.push_back(x);
            kp.push_back(y);
            kp.push_back(s);
            if (s > 0) {
                min_x = std::min(min_x, x);
                min_y = std::min(min_y, y);
                max_x = std::max(max_x, x);
                max_y = std::max(max_y, y);
            }
        }
        if (min_x == std::numeric_limits<float>::max())
            continue;
        out_keypoints.push_back(kp);
        out_scores.push_back(scores[p]);
        out_bbox.push_back({ min_x, min_y, max_x, max_y });
    }
    return std::make_tuple(out_keypoints, out_scores, out_bbox);
}
