//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_LAHC_HPP
#define FROGS_LAHC_HPP

#include "case.hpp"
#include "Split.h"
#include "leader_lahc.hpp"
#include "leader_array.hpp"
#include "follower.hpp"
#include "individual.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Lahc final : public HeuristicInterface, public StatsInterface {
public:
    static const std::string ALGORITHM;

    bool enable_logging;
    int stop_criteria;

    long iter;                                  // Iteration counter I
    long idle_iter;                             // Idle iteration counter
    long history_length;                        // LAHC history length Lh
    vector<double> history_list;                // Lahc history list L, it holds the objetive values
    std::unique_ptr<Individual> global_best;    // Global best solution found so far
    Indicators history_list_metrics;            // The statistical info of the history list
    Individual* current;                        // Current solution s

    Split* split;
//    LeaderLahc* leader;
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
