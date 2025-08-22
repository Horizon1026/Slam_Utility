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
    std::string full_key = "";
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

        // Remove meaningless characters.
        string_to_be_parsed = RemoveMeaninglessCharacters(string_to_be_parsed);
        if (string_to_be_parsed.empty()) {
            continue;
        }
        ReportDebug(string_to_be_parsed);
        const std::string string_in_brackets = GetStringInBrackets(string_to_be_parsed);
        ReportDebug(string_in_brackets);

        // TODO:

        // Clear the string after parsing.
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

bool YamlParser::IsComment(const std::string &str) {
    if (str.empty()) {
        return false;
    }
    return str.front() == '#';
}

} // namespace SLAM_UTILITY
