//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_HEURISTIC_INTERFACE_HPP
#define FROGS_HEURISTIC_INTERFACE_HPP

#include <iostream>
#include <random>
#include <chrono>
#include <utility>

using namespace std;

class HeuristicInterface {
public:
    string name;
    uniform_real_distribution<double> uniform_real_dist;

    // Constructor to initialize member variables
    HeuristicInterface(string heuristic_name, int seed_value)
            : name(std::move(heuristic_name)),
              uniform_real_dist(0.0, 1.0) {

    }

    virtual void run() = 0;
    virtual void initialize_heuristic() = 0;
    virtual void run_heuristic() = 0;
    [[nodiscard]] virtual bool stop_criteria_max_evals() const = 0;
    [[nodiscard]] virtual bool stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const = 0;
    [[nodiscard]] virtual bool stop_criteria_obj_convergence(double current_best_obj) const = 0;

    virtual ~HeuristicInterface() = default;
};

#endif //FROGS_HEURISTIC_INTERFACE_HPP
