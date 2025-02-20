//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_HEURISTIC_INTERFACE_HPP
#define FROGS_HEURISTIC_INTERFACE_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <utility>

using namespace std;

class HeuristicInterface {
public:
    string name;
    Case* instance;
    Preprocessor* preprocessor;
    std::default_random_engine random_engine;
    uniform_real_distribution<double> uniform_real_dist;

    // Constructor to initialize member variables
    HeuristicInterface(string heuristic_name, Case* instance, Preprocessor* preprocessor)
            : name(std::move(heuristic_name)),
              instance(instance),
              preprocessor(preprocessor),
              random_engine(preprocessor->params.seed),
              uniform_real_dist(0.0, 1.0) {

    }

    virtual void run() = 0;
    virtual void initialize_heuristic() = 0;
    virtual void run_heuristic() = 0;
    [[nodiscard]] virtual bool stop_criteria_max_evals() const {
        return instance->get_evals() >= preprocessor->max_evals_;
    }

    [[nodiscard]] virtual bool stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const {
        return duration.count() >= preprocessor->max_exec_time_;
    }

    [[nodiscard]] virtual bool stop_criteria_obj_convergence(const double current_best_obj) const {
        static int no_improvement_count = 0; // consecutive no-change count
        static double prev_best_obj = std::numeric_limits<double>::max(); // previous best objective, initialized to infinity

        // If change is small, increment count; otherwise, reset
        if (double obj_change = std::abs(current_best_obj - prev_best_obj); obj_change < MY_EPSILON) {
            no_improvement_count++;
        } else {
            no_improvement_count = 0;
        }

        // update previous objective value
        prev_best_obj = current_best_obj;

        // check if max no-change count is reached
        return no_improvement_count >= preprocessor->max_no_improvement_count_;
    }

    virtual ~HeuristicInterface() = default;
};

#endif //FROGS_HEURISTIC_INTERFACE_HPP
