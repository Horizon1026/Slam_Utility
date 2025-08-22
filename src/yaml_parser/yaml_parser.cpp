#include "yaml_parser.h"
#include "slam_log_reporter.h"
#include "fstream"
#include "sstream"
#include "algorithm"
#include "cctype"
#include "stdexcept"

namespace SLAM_UTILITY {

YamlParser::ConfigMap YamlParser::ParseFile(const std::string &file_path) {
    std::string content;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ReportError("[YamlParser] Failed to open file: " << file_path);
        return {};
    }
    // Load full content of the file.
    std::stringstream buffer;
    buffer << file.rdbuf();
    content = buffer.str();
    file.close();
    return ParseString(content);
}

YamlParser::ConfigMap YamlParser::ParseString(const std::string &content) {
    ConfigMap config_map;
    std::istringstream stream(content);
    std::string current_line;
    std::string string_to_be_parsed = "";
    std::vector<std::string> root_keys;
    bool need_to_pop_root_key = false;
    while (std::getline(stream, current_line)) {
        // Remove comments. If the line is totally commented, skip it.
        current_line = RemoveComments(current_line);
        if (current_line.empty()) {
            continue;
        }

        // Check if the line is a valid string to be parsed.
        // Be careful about '[' and ']' inside two quotes '"'.
        string_to_be_parsed += current_line;
        const std::string string_without_quotes = RemoveStringInQuotes(string_to_be_parsed);
        if (!HasPairsOfBrackets(string_without_quotes)) {
            continue;
        }

        // Get level of the key.
        const uint32_t key_level = GetKeyLevel(string_to_be_parsed);

        // Remove meaningless characters.
        string_to_be_parsed = RemoveMeaninglessCharacters(string_to_be_parsed);
        if (string_to_be_parsed.empty()) {
            continue;
        }

        // Get the key and value.
        const std::string key = GetKey(string_to_be_parsed);
        if (key.empty()) {
            string_to_be_parsed = "";
            continue;
        }

        // Process root keys according to the key level.
        if (need_to_pop_root_key) {
            while (key_level < root_keys.size()) {
                root_keys.pop_back();
            }
            need_to_pop_root_key = false;
        }

        // Add current key to the root keys.
        root_keys.emplace_back(key);

        // Get the value.
        const std::string value = GetValue(string_to_be_parsed);
        if (value.empty()) {
            string_to_be_parsed = "";
            continue;
        } else {
            need_to_pop_root_key = true;
        }

        // Add key and value to the config map.
        const std::string full_key = GetRootKey(root_keys);
        if (value[0] == '[') {
            // Array value.
            const std::vector<std::string> array_values = GetArrayValue(value);
            config_map[full_key] = array_values;
        } else if (value[0] == '"') {
            // String value.
            config_map[full_key] = {GetStringInQuotes(value)};
        } else {
            // Single value.
            config_map[full_key] = {GetStringInQuotes(value)};
        }

        // Prepare for the next line.
        if (!root_keys.empty()) {
            root_keys.pop_back();
        }
        string_to_be_parsed = "";
    }
    return config_map;
}

void YamlParser::PrintConfigMap(const ConfigMap &config_map) {
    for (const auto &[key, values] : config_map) {
        if (values.empty()) {
            continue;
        }

        if (values.size() == 1) {
            ReportInfo(key << ": " << values[0]);
            continue;
        }

        ReportInfo(key << ": [");
        for (const auto &value : values) {
            ReportInfo("    " << value);
        }
        ReportInfo("]");
    }
}

bool YamlParser::HasPairsOfBrackets(const std::string &str) {
    int32_t num_of_left_brackets = 0;
    int32_t num_of_right_brackets = 0;
    for (const char &c : str) {
        if (c == '[') {
            ++num_of_left_brackets;
        } else if (c == ']') {
            ++num_of_right_brackets;
        }
    }
    return num_of_left_brackets == num_of_right_brackets;
}

std::string YamlParser::RemoveComments(const std::string &str) {
    std::string result = "";
    for (const char &c : str) {
        if (c == '#') {
            break;
        }
        result += c;
    }
    return result;
}
std::string YamlParser::RemoveMeaninglessCharacters(const std::string &str) {
    std::string result = "";
    for (const char &c : str) {
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            continue;
        }
        result += c;
    }
    return result;
}

std::string YamlParser::RemoveStringInQuotes(const std::string &str) {
    std::string result = "";
    bool is_in_quotes = false;
    for (const char &c : str) {
        if (c == '"') {
            is_in_quotes = !is_in_quotes;
            continue;
        }
        if (is_in_quotes) {
            continue;
        }
        result += c;
    }
    return result;
}

std::string YamlParser::GetStringInQuotes(const std::string &str) {
    if (str.empty()) {
        return "";
    }
    if (str[0] != '"') {
        return str;
    }
    return str.substr(1, str.size() - 2);
}

std::string YamlParser::GetStringInBrackets(const std::string &str) {
    std::string result = "";
    bool is_in_brackets = false;
    bool is_in_quotes = false;
    for (const char &c : str) {
        if (c == '"') {
            is_in_quotes = !is_in_quotes;
        }
        if (c == '[' && !is_in_quotes) {
            is_in_brackets = true;
            continue;
        }
        if (c == ']' && !is_in_quotes) {
            is_in_brackets = false;
            continue;
        }
        if (is_in_brackets) {
            result += c;
        }
    }
    if (is_in_brackets) {
        return "";
    }
    return result;
}

std::string YamlParser::GetKey(const std::string &str) {
    std::string result = "";
    for (const char &c : str) {
        if (c == ':') {
            break;
        }
        result += c;
    }
    return result;
}

std::string YamlParser::GetValue(const std::string &str) {
    std::string result = "";
    bool is_key_name = true;
    for (const char &c : str) {
        if (is_key_name && c == ':') {
            is_key_name = false;
            continue;
        }
        if (is_key_name) {
            continue;
        }
        result += c;
    }
    return result;
}

std::string YamlParser::GetRootKey(const std::vector<std::string> &root_keys) {
    std::string result = "";
    for (const auto &key : root_keys) {
        result += key + ":";
    }

    if (!result.empty()) {
        result.pop_back();
    }
    return result;
}

uint32_t YamlParser::GetKeyLevel(const std::string &str) {
    uint32_t result = 0;
    for (const char &c : str) {
        if (c == ' ') {
            ++result;
        }
        if (c == ':') {
            break;
        }
    }
    return result / 4;
}

std::vector<std::string> YamlParser::GetArrayValue(const std::string &str) {
    const std::string string_in_brackets = GetStringInBrackets(str);
    std::vector<std::string> result;
    std::string current_value = "";
    for (const char &c : string_in_brackets) {
        if (c == ',') {
            result.emplace_back(GetStringInQuotes(current_value));
            current_value = "";
            continue;
        }
        current_value += c;
    }
    if (!current_value.empty()) {
        result.emplace_back(GetStringInQuotes(current_value));
    }
    return result;
}

} // namespace SLAM_UTILITY
