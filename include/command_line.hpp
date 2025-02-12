//
// Created by Yinghao Qin on 12/02/2025.
//

#ifndef FROGS_COMMAND_LINE_HPP
#define FROGS_COMMAND_LINE_HPP

#include "parameters.hpp"
#include <unordered_map>
#include <vector>


class CommandLine {
private:
    std::unordered_map<std::string, std::string> arguments;

public:
    // Constructor: Parses command-line arguments
    CommandLine(int argc, char* argv[]);

    // Override default parameters based on command-line arguments
    void parse_parameters(Parameters& params) const;

    // Display help message
    static void display_help() ;

    // Methods to get argument values
    [[nodiscard]] int get_int(const std::string& key, int default_value) const;
    [[nodiscard]] double get_double(const std::string& key, double default_value) const;
    [[nodiscard]] std::string get_string(const std::string& key, const std::string& default_value) const;
    [[nodiscard]] bool get_bool(const std::string& key, bool default_value) const;

    // Debug: Print all parsed arguments
    void print_arguments() const;
};

#endif //FROGS_COMMAND_LINE_HPP
