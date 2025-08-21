#ifndef _SLAM_UTILITY_YAML_PARSER_H_
#define _SLAM_UTILITY_YAML_PARSER_H_

#include "string"
#include "vector"
#include "unordered_map"
#include "memory"

namespace SLAM_UTILITY {

/**
 * @brief YAML file parser class.
 *
 * Support parsing single value configuration, array configuration, root node and sub-node relationship.
 * The parsing result is stored in an unordered_map, where the key is the configuration item name (root node: sub-node format),
 * and the value is a vector of string parameters.
 */
class YamlParser {
public:
    using ConfigMap = std::unordered_map<std::string, std::vector<std::string>>;

    /**
     * @brief Parse YAML file.
     * @param file_path YAML file path.
     * @return Parsing result, return empty map if parsing fails.
     */
    static ConfigMap ParseFile(const std::string &file_path);

    /**
     * @brief Parse YAML string content.
     * @param content YAML string content.
     * @return Parsing result, return empty map if parsing fails.
     */
    static ConfigMap ParseString(const std::string &content);

    /**
     * @brief Convert string to bool type.
     * @param str Input string.
     * @return Converted bool value.
     */
    static bool ToBool(const std::string &str);

    /**
     * @brief Convert string to int type.
     * @param str Input string.
     * @return Converted int value.
     */
    static int ToInt(const std::string &str);

    /**
     * @brief Convert string to float type.
     * @param str Input string.
     * @return Converted float value.
     */
    static float ToFloat(const std::string &str);

    /**
     * @brief Convert string to double type.
     * @param str Input string.
     * @return Converted double value.
     */
    static double ToDouble(const std::string &str);

    /**
     * @brief Print the configuration mapping table.
     * @param config_map Configuration mapping table.
     */
    static void PrintConfigMap(const ConfigMap &config_map);

private:

    /**
     * @brief Check if the string is empty or only contains whitespace characters.
     * @param str Input string.
     * @return True if the string is empty or only contains whitespace characters.
     */
    static bool IsEmpty(const std::string &str);

    /**
     * @brief Remove leading and trailing whitespace characters from the string.
     * @param str Input string.
     * @return String with leading and trailing whitespace characters removed.
     */
    static std::string Trim(const std::string &str);

    /**
     * @brief Parse single line of YAML content.
     * @param line Single line content.
     * @param current_key Current key name.
     * @param config_map Configuration mapping table.
     */
    static void ParseLine(const std::string &line, std::string &current_key, ConfigMap &config_map);

    /**
     * @brief Parse array value.
     * @param array_str Array string.
     * @return Parsed string vector.
     */
    static std::vector<std::string> ParseArray(const std::string &array_str);

    /**
     * @brief Get indent level.
     * @param line Input line.
     * @return Indent level (number of spaces).
     */
    static uint32_t GetIndentLevel(const std::string &line);

    /**
     * @brief Check if the line is an array line.
     * @param line Input line.
     * @return True if the line is an array line.
     */
    static bool IsArrayLine(const std::string &line);

    /**
     * @brief Check if the line is a key-value pair line.
     * @param line Input line.
     * @return True if the line is a key-value pair line.
     */
    static bool IsKeyValueLine(const std::string &line);

    /**
     * @brief Check if the line starts a multi-line array.
     * @param line Input line.
     * @return True if the line starts a multi-line array.
     */
    static bool IsMultiLineArrayStart(const std::string &line);

    /**
     * @brief Find the end line of a multi-line array.
     * @param lines All lines of the YAML content.
     * @param start_line Index of the line that starts the array.
     * @return Index of the line that ends the array.
     */
    static uint32_t FindMultiLineArrayEnd(const std::vector<std::string> &lines, uint32_t start_line);

    /**
     * @brief Parse a multi-line array.
     * @param array_lines Lines that make up the multi-line array.
     * @param current_key Current key name.
     * @param config_map Configuration mapping table.
     */
    static void ParseMultiLineArray(const std::vector<std::string> &array_lines,
                                   std::string &current_key, ConfigMap &config_map);
};

} // namespace SLAM_UTILITY

#endif // _SLAM_UTILITY_YAML_PARSER_H_
