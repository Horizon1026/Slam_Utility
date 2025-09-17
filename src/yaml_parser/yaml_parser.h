#ifndef _SLAM_UTILITY_YAML_PARSER_H_
#define _SLAM_UTILITY_YAML_PARSER_H_

#include "string"
#include "vector"
#include "unordered_map"
#include "memory"

namespace SLAM_UTILITY {

/* Class YamlParser Declaration. */
class YamlParser final {

public:
    using ConfigMap = std::unordered_map<std::string, std::vector<std::string>>;

public:
    YamlParser() = default;
    ~YamlParser() = default;

    static ConfigMap ParseFile(const std::string &file_path);
    static ConfigMap ParseString(const std::string &content);
    static void PrintConfigMap(const ConfigMap &config_map);

    static bool ToBool(const std::string &str);
    static uint8_t ToUint8(const std::string &str);
    static uint16_t ToUint16(const std::string &str);
    static uint32_t ToUint32(const std::string &str);
    static uint64_t ToUint64(const std::string &str);
    static int8_t ToInt8(const std::string &str);
    static int16_t ToInt16(const std::string &str);
    static int32_t ToInt32(const std::string &str);
    static int64_t ToInt64(const std::string &str);
    static float ToFloat(const std::string &str);
    static double ToDouble(const std::string &str);

private:
    static bool HasPairsOfBrackets(const std::string &str);
    static std::string RemoveComments(const std::string &str);
    static std::string RemoveMeaninglessCharacters(const std::string &str);
    static std::string RemoveStringInQuotes(const std::string &str);
    static std::string GetStringInQuotes(const std::string &str);
    static std::string GetStringInBrackets(const std::string &str);
    static std::string GetKey(const std::string &str);
    static std::string GetValue(const std::string &str);
    static std::string GetRootKey(const std::vector<std::string> &root_keys);
    static uint32_t GetKeyLevel(const std::string &str);
    static std::vector<std::string> GetArrayValue(const std::string &str);

};

}

#endif // _SLAM_UTILITY_YAML_PARSER_H_
