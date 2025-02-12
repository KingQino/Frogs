//
// Created by Yinghao Qin on 12/02/2025.
//

#include "command_line.hpp"
#include <stdexcept>

CommandLine::CommandLine(int argc, char* argv[]) {
    for (int i = 1; i < argc; i += 2) {
        if (i + 1 < argc) { // Ensure there's a value after the key
            std::string key = argv[i];
            std::string value = argv[i + 1];

            // Remove leading '-' or '--' if present
            if (key[0] == '-') key.erase(0, 1);
            if (key[0] == '-') key.erase(0, 1);

            arguments[key] = value;
        } else {
            std::cerr << "Warning: Missing value for argument '" << argv[i] << "'" << std::endl;
        }
    }
}

void CommandLine::parse_parameters(Parameters& params) const {
    params.algorithm = get_string("algorithm", params.algorithm);
    params.instance = get_string("instance", params.instance);
    params.enable_logging = get_bool("enable_logging", params.enable_logging);
    params.stop_criteria = get_int("stop_criteria", params.stop_criteria);
    params.enable_multithreading = get_bool("enable_multithreading", params.enable_multithreading);
    params.seed = get_int("seed", params.seed);
    params.nb_granular = get_int("nb_granular", params.nb_granular);
}

// Display help message
void CommandLine::display_help() {
    std::cout << "--------------------------------------------------- Parameters Instruction  ---------------------------------------------------" << std::endl;
    std::cout << "Usage: ./Run [options]\n"
              << "Options:\n"
              << "  -algorithm [name]           : Algorithm name (e.g., Ma, Lahc)\n"
              << "  -instance [filename]        : Problem instance filename\n"
              << "  -enable_logging [0|1]       : Enable logging (default: 0)\n"
              << "  -stop_criteria [0|1|2]      : Stopping criteria (default: 0)\n"
              << "  -enable_multithreading [0|1]: Enable multi-threading (default: 1)\n"
              << "  -seed [int]                 : Random seed (default: 0)\n"
              << "  -nb_granular [int]          : Granular search parameter (default: 20)\n";
    std::cout << "-------------------------------------------------------------------------------------------------------------------------------" << std::endl;
}

int CommandLine::get_int(const std::string& key, int default_value) const {
    auto it = arguments.find(key);
    if (it != arguments.end()) {
        try {
            return std::stoi(it->second);
        } catch (const std::exception&) {
            std::cerr << "Error: Invalid integer value for argument '" << key << "'. Using default: " << default_value << std::endl;
        }
    }
    return default_value;
}

double CommandLine::get_double(const std::string& key, double default_value) const {
    auto it = arguments.find(key);
    if (it != arguments.end()) {
        try {
            return std::stod(it->second);
        } catch (const std::exception&) {
            std::cerr << "Error: Invalid double value for argument '" << key << "'. Using default: " << default_value << std::endl;
        }
    }
    return default_value;
}

std::string CommandLine::get_string(const std::string& key, const std::string& default_value) const {
    auto it = arguments.find(key);
    return (it != arguments.end()) ? it->second : default_value;
}

bool CommandLine::get_bool(const std::string& key, bool default_value) const {
    auto it = arguments.find(key);
    if (it != arguments.end()) {
        std::string val = it->second;
        return (val == "1" || val == "true" || val == "yes");
    }
    return default_value;
}

void CommandLine::print_arguments() const {
    std::cout << "Parsed Command-Line Arguments:\n";
    for (const auto& arg : arguments) {
        std::cout << "  " << arg.first << " = " << arg.second << std::endl;
    }
}
