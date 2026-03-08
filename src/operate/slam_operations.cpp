#include "slam_operations.h"
#include "filesystem"
#include "fstream"

namespace slam_utility {

bool SlamOperation::GetFilesNameInDirectory(const std::string &dir, std::vector<std::string> &filenames) {
    namespace fs = std::filesystem;
    filenames.clear();
    if (dir.empty()) {
        return false;
    }
    try {
        for (const auto &entry: fs::directory_iterator(dir)) {
            // Ignore '.' and '..' (filesystem does not return these, but just in case)
            const auto &path = entry.path();
            std::string filename = path.filename().string();
            if (filename == "." || filename == "..") {
                continue;
            }
            filenames.emplace_back(path.string());
        }
        return true;
    } catch (const fs::filesystem_error &) {
        return false;
    }
}

bool SlamOperation::GetFilesNameInDirectoryRecursively(const std::string &dir, std::vector<std::string> &filenames) {
    namespace fs = std::filesystem;
    filenames.clear();
    if (dir.empty()) {
        return false;
    }
    try {
        for (const auto &entry: fs::recursive_directory_iterator(dir)) {
            // Ignore '.' and '..' (filesystem does not return these, but just in case)
            const auto &path = entry.path();
            std::string filename = path.filename().string();
            if (filename == "." || filename == "..") {
                continue;
            }
            filenames.emplace_back(path.string());
        }
        return true;
    } catch (const fs::filesystem_error &) {
        return false;
    }
}

bool SlamOperation::SaveMatrixToTxtFile(const Mat &matrix, const std::string &file_path) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
        return false;
    }

    for (int32_t i = 0; i < matrix.rows(); ++i) {
        for (int32_t j = 0; j < matrix.cols(); ++j) {
            ofs << matrix(i, j) << " ";
        }
        ofs << "\n";
    }

    ofs.close();
    return true;
}

bool SlamOperation::LoadMatrixFromTxtFile(const std::string &file_path, Mat &matrix) {
    std::ifstream ifs(file_path);
    RETURN_FALSE_IF(!ifs.is_open());

    // Load data from txt file.
    std::vector<std::vector<float>> matrix_data;
    std::string line;
    while (std::getline(ifs, line)) {
        std::stringstream ss(line);
        std::vector<float> row;
        float value;
        while (ss >> value) {
            row.push_back(value);
        }
        matrix_data.push_back(row);
    }
    ifs.close();

    // Load data to matrix.
    matrix.resize(matrix_data.size(), matrix_data.front().size());
    for (uint32_t row = 0; row < matrix_data.size(); ++row) {
        for (uint32_t col = 0; col < matrix_data[row].size(); ++col) {
            matrix(row, col) = matrix_data[row][col];
        }
    }
    return true;
}

bool SlamOperation::IsEndWith(const std::string &raw_string, const std::string &sub_string) {
    RETURN_FALSE_IF(sub_string.length() > raw_string.length());
    return raw_string.substr(raw_string.length() - sub_string.length()) == sub_string;
}

bool SlamOperation::IsEndWith(const std::vector<std::string> &all_raw_string, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    for (const auto &raw_str: all_raw_string) {
        RETURN_FALSE_IF(!IsEndWith(raw_str, sub_string));
    }
    return true;
}

bool SlamOperation::IsEndWith(const std::vector<std::string> &all_raw_string, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_raw_string.size() != all_sub_string.size());
    for (uint32_t i = 0; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsEndWith(all_raw_string[i], all_sub_string[i]));
    }
    return true;
}

bool SlamOperation::IsEndWith(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    for (uint32_t i = begin_idx; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsEndWith(all_raw_string[i], sub_string));
    }
    return true;
}

bool SlamOperation::IsEndWith(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_sub_string.empty());
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    RETURN_FALSE_IF(begin_idx + all_sub_string.size() > all_raw_string.size());
    for (uint32_t i = 0; i < all_sub_string.size(); ++i) {
        RETURN_FALSE_IF(!IsEndWith(all_raw_string[i + begin_idx], all_sub_string[i]));
    }
    return true;
}

bool SlamOperation::IsBeginWith(const std::string &raw_string, const std::string &sub_string) {
    RETURN_FALSE_IF(sub_string.length() > raw_string.length());
    return raw_string.substr(0, sub_string.length()) == sub_string;
}

bool SlamOperation::IsBeginWith(const std::vector<std::string> &all_raw_string, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    for (const auto &raw_str: all_raw_string) {
        RETURN_FALSE_IF(!IsBeginWith(raw_str, sub_string));
    }
    return true;
}

bool SlamOperation::IsBeginWith(const std::vector<std::string> &all_raw_string, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_raw_string.size() != all_sub_string.size());
    for (uint32_t i = 0; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsBeginWith(all_raw_string[i], all_sub_string[i]));
    }
    return true;
}

bool SlamOperation::IsBeginWith(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    for (uint32_t i = begin_idx; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsBeginWith(all_raw_string[i], sub_string));
    }
    return true;
}

bool SlamOperation::IsBeginWith(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_sub_string.empty());
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    RETURN_FALSE_IF(begin_idx + all_sub_string.size() > all_raw_string.size());
    for (uint32_t i = 0; i < all_sub_string.size(); ++i) {
        RETURN_FALSE_IF(!IsBeginWith(all_raw_string[i + begin_idx], all_sub_string[i]));
    }
    return true;
}

bool SlamOperation::IsContained(const std::string &raw_string, const std::string &sub_string) { return raw_string.find(sub_string) != std::string::npos; }

bool SlamOperation::IsContained(const std::vector<std::string> &all_raw_string, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    for (const auto &raw_str: all_raw_string) {
        RETURN_FALSE_IF(!IsContained(raw_str, sub_string));
    }
    return true;
}

bool SlamOperation::IsContained(const std::vector<std::string> &all_raw_string, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_raw_string.size() != all_sub_string.size());
    for (uint32_t i = 0; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsContained(all_raw_string[i], all_sub_string[i]));
    }
    return true;
}

bool SlamOperation::IsContained(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::string &sub_string) {
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    for (uint32_t i = begin_idx; i < all_raw_string.size(); ++i) {
        RETURN_FALSE_IF(!IsContained(all_raw_string[i], sub_string));
    }
    return true;
}

bool SlamOperation::IsContained(const std::vector<std::string> &all_raw_string, const uint32_t begin_idx, const std::vector<std::string> &all_sub_string) {
    RETURN_FALSE_IF(all_sub_string.empty());
    RETURN_FALSE_IF(all_raw_string.empty());
    RETURN_FALSE_IF(begin_idx >= all_raw_string.size());
    RETURN_FALSE_IF(begin_idx + all_sub_string.size() > all_raw_string.size());
    for (uint32_t i = 0; i < all_sub_string.size(); ++i) {
        RETURN_FALSE_IF(!IsContained(all_raw_string[i + begin_idx], all_sub_string[i]));
    }
    return true;
}

std::string SlamOperation::ReplaceBy(const std::string &raw_string, const std::string &sub_string, const std::string &replace_string) {
    std::string result = raw_string;
    auto pos = result.find(sub_string);
    while (pos != std::string::npos) {
        result.replace(pos, sub_string.length(), replace_string);
        pos = result.find(sub_string, pos + replace_string.length());
    }
    return result;
}

bool SlamOperation::IsDirectory(const std::string &path) { return std::filesystem::is_directory(path); }

std::string SlamOperation::GetLastSubStringAfter(const std::string &raw_string, const std::string &divider) {
    // Split raw_string using divider and return the last substring.
    // If divider is not found, return raw_string itself.
    if (divider.empty()) {
        return raw_string;
    }
    size_t pos = raw_string.rfind(divider);
    if (pos == std::string::npos) {
        return raw_string;
    }
    return raw_string.substr(pos + divider.length());
}

}  // namespace slam_utility
