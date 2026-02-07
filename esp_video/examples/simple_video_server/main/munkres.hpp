#ifndef MUNKRES_HPP
#define MUNKRES_HPP

#include <vector>
#include <limits>
#include <cstdint>
#include <stdexcept>
#include <algorithm>

struct HigherHRNetOutput {
    std::vector<std::vector<float>> tag;
    std::vector<std::vector<int32_t>> ind;
    std::vector<std::vector<float>> val;
};

static const std::vector<int32_t> default_joint_order = { 0, 1, 2, 3, 4, 5, 6, 11, 12, 7, 8, 9, 10, 13, 14, 15, 16 };

std::tuple<std::vector<std::vector<float>>, std::vector<float>, std::vector<std::vector<float>>>
postprocess_higherhrnet(const HigherHRNetOutput& outputs,
    const std::pair<int32_t, int32_t>& img_size,
    const std::pair<int32_t, int32_t>& img_w_pad,
    const std::pair<int32_t, int32_t>& img_h_pad,
    float detection_threshold,
    bool network_postprocess,
    const std::pair<int32_t, int32_t>& input_image_size,
    const std::pair<int32_t, int32_t>& output_shape,
    int32_t num_joints = 17,
    const std::vector<int32_t>& joint_order = default_joint_order,
    int32_t max_num_people = 30,
    bool ignore_too_much = false,
    bool use_detection_val = true,
    float tag_threshold = 1.0f);

class Munkres {
public:
    using Matrix = std::vector<std::vector<float>>;
    using Pair = std::pair<int32_t, int32_t>;

    // Compute optimal assignment for cost_matrix
    // Returns list of (row, col) pairs
    std::vector<Pair> compute(const Matrix& cost_matrix) {
        // Pad to square
        C = padMatrix(cost_matrix, 0.0);
        n = C.size();
        original_rows = cost_matrix.size();
        original_cols = cost_matrix[0].size();
        row_covered.assign(n, false);
        col_covered.assign(n, false);
        marked.assign(n, std::vector<int32_t>(n, 0));
        path.assign(n * 2, std::vector<int32_t>(2, 0));
        Z0_r = Z0_c = 0;

        int32_t step = 1;
        while (step > 0 && step < 7) {
            switch (step) {
            case 1: step = step1(); break;
            case 2: step = step2(); break;
            case 3: step = step3(); break;
            case 4: step = step4(); break;
            case 5: step = step5(); break;
            case 6: step = step6(); break;
            default: step = 7; break;
            }
        }

        // Collect results
        std::vector<Pair> results;
        for (int32_t i = 0; i < original_rows; ++i) {
            for (int32_t j = 0; j < original_cols; ++j) {
                if (marked[i][j] == 1) {
                    results.emplace_back(i, j);
                }
            }
        }
        return results;
    }

private:
    int32_t n;
    int32_t original_rows;
    int32_t original_cols;
    Matrix C;
    std::vector<bool> row_covered;
    std::vector<bool> col_covered;
    std::vector<std::vector<int32_t>> marked;
    std::vector<std::vector<int32_t>> path;
    int32_t Z0_r, Z0_c;

    // Pad non-square matrix to square by adding zero rows/cols
    Matrix padMatrix(const Matrix& matrix, float pad_value) {
        int32_t rows = matrix.size();
        int32_t cols = matrix[0].size();
        int32_t m = std::max(rows, cols);
        Matrix newMat(m, std::vector<float>(m, pad_value));
        for (int32_t i = 0; i < rows; ++i) {
            for (int32_t j = 0; j < cols; ++j) {
                newMat[i][j] = matrix[i][j];
            }
        }
        return newMat;
    }

    int32_t step1() {
        // For each row, subtract minimal value
        for (int32_t i = 0; i < n; ++i) {
            float minval = std::numeric_limits<float>::infinity();
            for (int32_t j = 0; j < n; ++j) {
                if (C[i][j] < minval) minval = C[i][j];
            }
            for (int32_t j = 0; j < n; ++j) {
                C[i][j] -= minval;
            }
        }
        return 2;
    }

    int32_t step2() {
        // Star zeros
        for (int32_t i = 0; i < n; ++i) {
            for (int32_t j = 0; j < n; ++j) {
                if (C[i][j] == 0 && !row_covered[i] && !col_covered[j]) {
                    marked[i][j] = 1;
                    row_covered[i] = true;
                    col_covered[j] = true;
                    break;
                }
            }
        }
        clearCovers();
        return 3;
    }

    int32_t step3() {
        int32_t count = 0;
        for (int32_t i = 0; i < n; ++i) {
            for (int32_t j = 0; j < n; ++j) {
                if (marked[i][j] == 1 && !col_covered[j]) {
                    col_covered[j] = true;
                    ++count;
                }
            }
        }
        return (count >= n) ? 7 : 4;
    }

    int32_t step4() {
        int32_t row = 0, col = 0;
        bool done = false;
        int32_t step = 0;
        while (!done) {
            auto zr = findAZero(row, col);
            row = zr.first;
            col = zr.second;
            if (row < 0) {
                done = true;
                step = 6;
            }
            else {
                marked[row][col] = 2; // prime
                int32_t starCol = findStarInRow(row);
                if (starCol >= 0) {
                    row_covered[row] = true;
                    col_covered[starCol] = false;
                    col = starCol;
                }
                else {
                    done = true;
                    Z0_r = row;
                    Z0_c = col;
                    step = 5;
                }
            }
        }
        return step;
    }

    int32_t step5() {
        int32_t count = 0;
        path[count][0] = Z0_r;
        path[count][1] = Z0_c;
        bool done = false;
        while (!done) {
            int32_t r = findStarInCol(path[count][1]);
            if (r >= 0) {
                ++count;
                path[count][0] = r;
                path[count][1] = path[count - 1][1];
            }
            else {
                done = true;
            }
            if (!done) {
                int32_t c = findPrimeInRow(path[count][0]);
                ++count;
                path[count][0] = path[count - 1][0];
                path[count][1] = c;
            }
        }
        convertPath(count);
        clearCovers();
        erasePrimes();
        return 3;
    }

    int32_t step6() {
        float minval = findSmallest();
        for (int32_t i = 0; i < n; ++i) {
            for (int32_t j = 0; j < n; ++j) {
                if (row_covered[i]) C[i][j] += minval;
                if (!col_covered[j]) C[i][j] -= minval;
            }
        }
        return 4;
    }

    std::pair<int32_t, int32_t> findAZero(int32_t startRow, int32_t startCol) {
        for (int32_t i = startRow; i < n; ++i) {
            for (int32_t j = (i == startRow ? startCol : 0); j < n; ++j) {
                if (C[i][j] == 0 && !row_covered[i] && !col_covered[j]) {
                    return { i, j };
                }
            }
        }
        return { -1, -1 };
    }

    int32_t findStarInRow(int32_t row) {
        for (int32_t j = 0; j < n; ++j) {
            if (marked[row][j] == 1) return j;
        }
        return -1;
    }

    int32_t findStarInCol(int32_t col) {
        for (int32_t i = 0; i < n; ++i) {
            if (marked[i][col] == 1) return i;
        }
        return -1;
    }

    int32_t findPrimeInRow(int32_t row) {
        for (int32_t j = 0; j < n; ++j) {
            if (marked[row][j] == 2) return j;
        }
        return -1;
    }

    void convertPath(int32_t count) {
        for (int32_t i = 0; i <= count; ++i) {
            int32_t r = path[i][0];
            int32_t c = path[i][1];
            marked[r][c] = (marked[r][c] == 1) ? 0 : 1;
        }
    }

    void clearCovers() {
        std::fill(row_covered.begin(), row_covered.end(), false);
        std::fill(col_covered.begin(), col_covered.end(), false);
    }

    void erasePrimes() {
        for (int32_t i = 0; i < n; ++i) {
            for (int32_t j = 0; j < n; ++j) {
                if (marked[i][j] == 2) marked[i][j] = 0;
            }
        }
    }

    float findSmallest() {
        float minval = std::numeric_limits<float>::infinity();
        for (int32_t i = 0; i < n; ++i) {
            if (!row_covered[i]) {
                for (int32_t j = 0; j < n; ++j) {
                    if (!col_covered[j] && C[i][j] < minval) {
                        minval = C[i][j];
                    }
                }
            }
        }
        return minval;
    }
};

#endif // MUNKRES_HPP
