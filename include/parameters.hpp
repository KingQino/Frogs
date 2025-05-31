//
// Created by Yinghao Qin on 12/02/2025.
//

#ifndef FROGS_PARAMETERS_HPP
#define FROGS_PARAMETERS_HPP

#include<iostream>
#include<string>

using namespace std;

enum class Algorithm { CBMA, LAHC, SGA};

struct Parameters {
    // Running parameters
    string kDataPath;
    string kStatsPath;
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
    int max_search_depth;       // LAHC max_search_depth determines how many times the neighbourhood (random) will be explored
    int max_neigh_attempts;     // Maximum attempts for neighbourhood exploration
    double low_opt_trigger_threshold; // LAHC: The threshold to trigger the lower-level optimisation
    double T0;                  // Initial temperature for simulated annealing (if applicable)
    double alpha;               // Cooling rate for simulated annealing (if applicable)
    int min_win;                // Minimum window size for recent deltas
    int max_win;                // Maximum window size for recent deltas
    double win_k;               // k value for dynamic window size calculation

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

        kDataPath = "../data/";
        kStatsPath = "../stats";
        nb_granular = 20;
        is_hard_constraint = true;
        is_duration_constraint = false;
        history_length = 5'000;
        max_search_depth = 200;
        low_opt_trigger_threshold = 0.3;
        T0 = 30.0;
        alpha = 0.98;
        min_win = 20;
        max_win = 500;
        win_k = 0.5;
        max_neigh_attempts = 10'000;
        runtime_multiplier = 1;
    }
};

#endif //FROGS_PARAMETERS_HPP
