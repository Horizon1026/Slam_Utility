#include "yaml_parser.h"
#include "slam_log_reporter.h"
#include "fstream"
#include "sstream"
#include "algorithm"
#include "cctype"
#include "stdexcept"

namespace SLAM_UTILITY {

YamlParser::ConfigMap YamlParser::ParseFile(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return ConfigMap{};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return ParseString(buffer.str());
}

YamlParser::ConfigMap YamlParser::ParseString(const std::string &content) {
    ConfigMap config_map;
    std::string current_key;
    std::vector<std::string> lines;

    // Divide the content into lines.
    std::istringstream iss(content);
    std::string line;
    while (std::getline(iss, line)) {
        lines.push_back(line);
    }

    // Parse each line with support for multi-line arrays.
    for (uint32_t i = 0; i < lines.size(); ++i) {
        if (IsMultiLineArrayStart(lines[i])) {
            // Handle multi-line array parsing.
            std::vector<std::string> array_lines;
            uint32_t end_line = FindMultiLineArrayEnd(lines, i);

            // Collect all lines for the multi-line array.
            for (uint32_t j = i; j <= end_line; ++j) {
                array_lines.push_back(lines[j]);
            }

            // Parse the multi-line array.
            ParseMultiLineArray(array_lines, current_key, config_map);

            // Skip the processed lines.
            i = end_line;
        } else {
            ParseLine(lines[i], current_key, config_map);
        }
    }

    return config_map;
}

void YamlParser::ParseLine(const std::string &line, std::string &current_key, ConfigMap &config_map) {
    std::string trimmed_line = Trim(line);

    // Skip empty lines and comment lines.
    if (IsEmpty(trimmed_line) || trimmed_line[0] == '#') {
        return;
    }

    // Skip lines that start multi-line arrays (they will be handled separately).
    if (IsMultiLineArrayStart(trimmed_line)) {
        return;
    }

    uint32_t indent_level = GetIndentLevel(line);

    // Root node (no indentation).
    if (indent_level == 0) {
        if (IsKeyValueLine(trimmed_line)) {
            uint32_t colon_pos = trimmed_line.find(':');
            if (colon_pos != std::string::npos) {
                current_key = Trim(trimmed_line.substr(0, colon_pos));
                std::string value = Trim(trimmed_line.substr(colon_pos + 1));

                if (!IsEmpty(value)) {
                    if (IsArrayLine(value)) {
                        config_map[current_key] = ParseArray(value);
                    } else {
                        config_map[current_key] = {value};
                    }
                } else {
                    config_map[current_key] = {value}; // Store the empty value
                }
            }
        } else if (trimmed_line.back() == ':') {
            // Only key name, no value.
            current_key = Trim(trimmed_line.substr(0, trimmed_line.length() - 1));
            config_map[current_key] = std::vector<std::string>{};
        }
    }
    // Sub-node (with indentation).
    else if (indent_level > 0 && !IsEmpty(current_key)) {
        if (IsKeyValueLine(trimmed_line)) {
            uint32_t colon_pos = trimmed_line.find(':');
            if (colon_pos != std::string::npos) {
                std::string sub_key = Trim(trimmed_line.substr(0, colon_pos));
                std::string value = Trim(trimmed_line.substr(colon_pos + 1));

                std::string full_key = current_key + ":" + sub_key;

                if (!IsEmpty(value)) {
                    if (IsArrayLine(value)) {
                        config_map[full_key] = ParseArray(value);
                    } else {
                        config_map[full_key] = {value};
                    }
                } else {
                    config_map[full_key] = {value}; // Store the empty value
                }
            }
        } else if (trimmed_line.back() == ':') {
            // Only sub-key name, no value.
            std::string sub_key = Trim(trimmed_line.substr(0, trimmed_line.length() - 1));
            std::string full_key = current_key + ":" + sub_key;
            config_map[full_key] = std::vector<std::string>{};
        }
    }
}

std::vector<std::string> YamlParser::ParseArray(const std::string &array_str) {
    std::vector<std::string> result;

    // Remove array marker.
    std::string content = array_str;
    if (content.front() == '[' && content.back() == ']') {
        content = content.substr(1, content.length() - 2);
    }

    // Divide array elements.
    std::istringstream iss(content);
    std::string element;
    while (std::getline(iss, element, ',')) {
        std::string trimmed_element = Trim(element);
        if (!IsEmpty(trimmed_element)) {
            result.push_back(trimmed_element);
        }
    }

    return result;
}

uint32_t YamlParser::GetIndentLevel(const std::string &line) {
    uint32_t count = 0;
    for (char c : line) {
        if (c == ' ' || c == '\t') {
            count++;
        } else {
            break;
        }
    }
    return count;
}

bool YamlParser::IsArrayLine(const std::string &str) {
    return str.front() == '[' && str.back() == ']';
}

bool YamlParser::IsKeyValueLine(const std::string &str) {
    return str.find(':') != std::string::npos;
}

bool YamlParser::IsMultiLineArrayStart(const std::string &line) {
    std::string trimmed = Trim(line);
    // Check if the line ends with '[' indicating the start of a multi-line array
    return !trimmed.empty() && trimmed.back() == '[';
}

uint32_t YamlParser::FindMultiLineArrayEnd(const std::vector<std::string> &lines, uint32_t start_line) {
    for (uint32_t i = start_line; i < lines.size(); ++i) {
        std::string trimmed = Trim(lines[i]);
        // Skip comment lines
        if (!trimmed.empty() && trimmed[0] == '#') {
            continue;
        }
        // Check if the line contains ']' indicating the end of the array
        if (trimmed.find(']') != std::string::npos) {
            return i;
        }
    }
    // If no closing bracket found, return the last line
    return lines.size() - 1;
}

void YamlParser::ParseMultiLineArray(const std::vector<std::string> &array_lines,
                                     std::string &current_key, ConfigMap &config_map) {
    if (array_lines.empty()) return;

    // Extract the key from the first line
    std::string first_line = array_lines[0];
    uint32_t colon_pos = first_line.find(':');
    if (colon_pos == std::string::npos) return;

    std::string key = Trim(first_line.substr(0, colon_pos));

    // Determine the full key based on context
    std::string full_key;
    if (!current_key.empty()) {
        // We're in a nested context, so append to current_key
        full_key = current_key + ":" + key;
    } else {
        // We're at root level
        full_key = key;
    }

    // Build the complete array string by concatenating all lines
    std::string array_content;
    for (const auto& line : array_lines) {
        std::string trimmed = Trim(line);
        // Skip comment lines
        if (!trimmed.empty() && trimmed[0] == '#') {
            continue;
        }
        // Skip empty lines
        if (IsEmpty(trimmed)) {
            continue;
        }
        array_content += trimmed + " ";
    }

    // Clean up the array content and parse it
    array_content = Trim(array_content);
    // Remove the key part from the first line
    uint32_t first_bracket = array_content.find('[');
    if (first_bracket != std::string::npos) {
        array_content = array_content.substr(first_bracket);
    }

    // Parse the array
    config_map[full_key] = ParseArray(array_content);
}

bool YamlParser::ToBool(const std::string &str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

    if (lower_str == "true" || lower_str == "1" || lower_str == "yes" || lower_str == "on") {
        return true;
    } else if (lower_str == "false" || lower_str == "0" || lower_str == "no" || lower_str == "off") {
        return false;
    }

    throw std::invalid_argument("[YamlParser] Failed to convert string to bool type: " + str);
}

int YamlParser::ToInt(const std::string &str) {
    try {
        return std::stoi(str);
    } catch (const std::exception& e) {
        throw std::invalid_argument("[YamlParser] Failed to convert string to int type: " + str);
    }
}

float YamlParser::ToFloat(const std::string &str) {
    try {
        return std::stof(str);
    } catch (const std::exception& e) {
        throw std::invalid_argument("[YamlParser] Failed to convert string to float type: " + str);
    }
}

double YamlParser::ToDouble(const std::string &str) {
    try {
        return std::stod(str);
    } catch (const std::exception& e) {
        throw std::invalid_argument("[YamlParser] Failed to convert string to double type: " + str);
    }
}

bool YamlParser::IsEmpty(const std::string &str) {
    return str.empty() || std::all_of(str.begin(), str.end(), ::isspace);
}

std::string YamlParser::Trim(const std::string &str) {
    uint32_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return "";
    }

    uint32_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

void YamlParser::PrintConfigMap(const ConfigMap &config_map) {
    for (const auto &[key, values] : config_map) {
        if (values.empty()) {
            continue;
        }

        if (values.size() == 1) {
            ReportInfo(key << " : " << values[0]);
            continue;
        }

        ReportInfo(key << " : [");
        for (const auto &value : values) {
            ReportInfo("    " << value);
        }
        ReportInfo("]");
    }
}

} // namespace SLAM_UTILITY
