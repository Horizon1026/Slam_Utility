#include "iomanip"
#include "iostream"
#include "slam_log_reporter.h"
#include "yaml_parser.h"

using namespace SLAM_UTILITY;


int main() {
    ReportColorWarn(">> Test YAML parser.");

    YamlParser::ConfigMap config_map = YamlParser::ParseFile("../examples/example.yaml");
    YamlParser::PrintConfigMap(config_map);

    return 0;
}
