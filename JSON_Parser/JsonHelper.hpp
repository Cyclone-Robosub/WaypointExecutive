#pragma once

#include <string>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

nlohmann::json load_json_from_file(const std::string& filepath);
bool validate_with_detailed_errors(const nlohmann::json &input_json, const std::string &schema_filepath);
