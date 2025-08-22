#ifndef _SLAM_UTILITY_YAML_PARSER_H_
#define _SLAM_UTILITY_YAML_PARSER_H_

#include "string"
#include "vector"
#include "unordered_map"
#include "memory"

namespace SLAM_UTILITY {

class YamlParser final {

public:
    using ConfigMap = std::unordered_map<std::string, std::vector<std::string>>;

public:
    YamlParser() = default;
    ~YamlParser() = default;

    static ConfigMap ParseFile(const std::string &file_path);
    static ConfigMap ParseString(const std::string &content);
    static void PrintConfigMap(const ConfigMap &config_map);

private:
    static bool HasPairsOfBrackets(const std::string &str);
    static std::string RemoveComments(const std::string &str);
    static std::string RemoveMeaninglessCharacters(const std::string &str);
    static std::string RemoveStringInQuotes(const std::string &str);
    static std::string GetStringInBrackets(const std::string &str);
    static bool IsComment(const std::string &str);
    static bool ToBool(const std::string &str);
    static int ToInt(const std::string &str);
    static float ToFloat(const std::string &str);
    static double ToDouble(const std::string &str);
    static bool IsEmpty(const std::string &str);

};

}

#endif // _SLAM_UTILITY_YAML_PARSER_H_
