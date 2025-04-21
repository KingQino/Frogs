//
// Created by Yinghao Qin on 12/02/2025.
//

#ifndef FROGS_PARAMETERS_HPP
#define FROGS_PARAMETERS_HPP

#include<iostream>
#include<string>

using namespace std;

enum class Algorithm { CBMA, LAHC};

struct Parameters {
    // Running parameters
    Algorithm algorithm;        // Algorithm name
    string instance;            // Problem instance name
    bool enable_logging;        // Enable logging
    int stop_criteria;          // Stopping criteria (e.g., max evaluations used)
    bool enable_multithreading; // Enable multi-threading
    int seed;                   // Random seed

    // Algorithm parameters
    int nb_granular;            // Granular search parameter
    bool is_hard_constraint;    // Hard constraint
    bool is_duration_constraint;// Whether to consider duration constraint
    int history_length;         // LAHC history length
    int max_neigh_attempts;     // Maximum attempts for neighbourhood exploration

    // experimental parameters
    int runtime_multiplier;     // Runtime multiplier


    // Constructor: Initializes default values
    Parameters() :
            algorithm(Algorithm::LAHC),
            instance("E-n22-k4.evrp"),
            enable_logging(false),
            stop_criteria(0),
            enable_multithreading(false),
            seed(0) {

        nb_granular = 20;
        is_hard_constraint = true;
        is_duration_constraint = false;
        history_length = 5'000;
        runtime_multiplier = 1;
        max_neigh_attempts = 10'000;
    }
};

#endif //FROGS_PARAMETERS_HPP
