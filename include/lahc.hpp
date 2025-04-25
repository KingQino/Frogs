//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_LAHC_HPP
#define FROGS_LAHC_HPP

#include "case.hpp"
#include "initializer.hpp"
#include "leader_array.hpp"
#include "follower.hpp"
#include "solution.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Lahc final : public HeuristicInterface, public StatsInterface {
public:
    static const std::string ALGORITHM;

    bool enable_logging;
    int stop_criteria;

    long boundary_no_low_opt;                   // The number of iterations without lower-level optimisation
    long iter;                                  // Iteration counter I
    long idle_iter;                             // Idle iteration counter
    long history_length;                        // LAHC history length Lh
    double num_moves_per_history;               // the number of algorithm moves per history length iteration
    double ratio_successful_moves;              // the ratio of successful moves per history length iteration
    vector<double> history_list;                // Lahc history list L, it holds the objetive values
    std::unique_ptr<Solution> global_best;      // Global best solution found so far
    Indicators history_list_metrics;            // The statistical info of the history list
    Solution* current;                          // Current solution s
    double global_best_upper_so_far;            // The best solution found so far

    Initializer* initializer;
    LeaderArray* leader;
    Follower* follower;

public:
    Lahc(int seed, Case *instance, Preprocessor* preprocessor);
    ~Lahc() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

};

#endif //FROGS_LAHC_HPP
