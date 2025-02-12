//
// Created by Yinghao Qin on 12/02/2025.
//

#ifndef FROGS_PARAMETERS_HPP
#define FROGS_PARAMETERS_HPP

#include<iostream>
#include<string>

using namespace std;

typedef struct tParameters {
    // Running parameters
    string algorithm;           // Algorithm name
    string instance;            // Problem instance name
    bool enable_logging;        // Enable logging
    int stop_criteria;          // Stopping criteria (e.g., max evaluations used)
    bool enable_multithreading; // Enable multi-threading
    int seed;                   // Random seed

    // Algorithm parameters
    int nb_granular;            // Granular search parameter

    // Constructor: Initializes default values
    tParameters() :
            algorithm("Lahc"),
            instance("E-n22-k4.evrp"),
            enable_logging(false),
            stop_criteria(0),
            enable_multithreading(false),
            seed(0) {

        nb_granular = 20;
    }
} Parameters;

#endif //FROGS_PARAMETERS_HPP
