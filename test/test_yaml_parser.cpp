#include "yaml_parser.h"
#include "slam_log_reporter.h"
#include "iostream"
#include "iomanip"

using namespace SLAM_UTILITY;

void DemonstrateTypeConversion(const std::string &value) {
    std::cout << "Raw value: " << value << std::endl;

    try {
        bool bool_val = YamlParser::ToBool(value);
        std::cout << "  -> bool: " << std::boolalpha << bool_val << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  -> bool: Failed - " << e.what() << std::endl;
    }

    try {
        int int_val = YamlParser::ToInt(value);
        std::cout << "  -> int: " << int_val << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  -> int: Failed - " << e.what() << std::endl;
    }

    try {
        float float_val = YamlParser::ToFloat(value);
        std::cout << "  -> float: " << float_val << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  -> float: Failed - " << e.what() << std::endl;
    }

    try {
        double double_val = YamlParser::ToDouble(value);
        std::cout << "  -> double: " << double_val << std::endl;
    } catch (const std::exception& e) {
        std::cout << "  -> double: Failed - " << e.what() << std::endl;
    }

    std::cout << std::endl;
}

int main() {
    ReportColorWarn(">> Test YAML parser.");

    // Test file parsing
    ReportColorWarn("1. Test file parsing:");
    auto config_map = YamlParser::ParseFile("../examples/example.yaml");

    if (config_map.empty()) {
        ReportError("Failed to parse YAML file.");
        return -1;
    }

    YamlParser::PrintConfigMap(config_map);

    // Test string parsing.
    ReportColorWarn("2. Test string parsing:");
    std::string yaml_content = R"(
        test_key: test_value
        array_test: [1, 2, 3, 4]
        nested:
        sub_key: sub_value
    )";

    auto string_config = YamlParser::ParseString(yaml_content);
    YamlParser::PrintConfigMap(string_config);

    // Test type conversion.
    ReportColorWarn("3. Test type conversion:");

    std::vector<std::string> test_values = {
        "true", "false", "123", "3.14", "hello", "0", "1.0"
    };

    for (const auto& value : test_values) {
        DemonstrateTypeConversion(value);
    }

    // Test specific configuration item access.
    ReportColorWarn("4. Test specific configuration item access:");

    if (config_map.find("app_name") != config_map.end()) {
        ReportInfo("Application name: " << config_map["app_name"][0]);
    }

    if (config_map.find("camera:model") != config_map.end()) {
        ReportInfo("Camera model: " << config_map["camera:model"][0]);
    }

    if (config_map.find("feature_types") != config_map.end()) {
        ReportInfo("Feature types number: " << config_map["feature_types"].size());
        ReportInfo("First feature type: " << config_map["feature_types"][0]);
    }

    ReportColorWarn("Test completed!");
    return 0;
}
