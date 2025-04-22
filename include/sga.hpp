//
// Created by Yinghao Qin on 09/04/2025.
//

#ifndef FROGS_SGA_HPP
#define FROGS_SGA_HPP

#include "case.hpp"
#include "initializer.hpp"
#include "leader_sga.hpp"
#include "follower.hpp"
#include "individual.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Sga final : public HeuristicInterface, public StatsInterface {
private:
    std::vector<std::vector<int>> elites;
    std::vector<std::vector<int>> immigrants;
    std::vector<std::vector<int>> offspring;
public:
    static const std::string ALGORITHM;

    bool enable_logging;
    int stop_criteria;

    int pop_size;
    int gen;
    vector<unique_ptr<Individual>> population;
    unique_ptr<Individual> global_best;      // Global best solution found so far
    double global_best_upper_so_far{};            // The best solution found so far

    uniform_int_distribution<int> uniform_int_dis;// Uniform distribution for random integers
    double mut_ind_prob; // Probability of mutation for each individual
    int max_neigh_attempts;

    vector<double> data_logging1;
    vector<double> data_logging2;
    Indicators pop_cost_metrics;
    Indicators pop_cost_metrics_after_impro; // The statistical info of the population cost metrics

    Initializer* initializer;
    std::vector<std::unique_ptr<LeaderSga>> leaders;
    std::vector<std::unique_ptr<Follower>> followers;
    std::vector<std::unique_ptr<PartialSolution>> partial_sols;

    Sga(int seed, Case *instance, Preprocessor *preprocessor);
    ~Sga() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

    void cx_partially_matched(vector<int>& parent1, vector<int>& parent2);
    void mut_shuffle_indexes(vector<int>& chromosome, double ind_pb);
};


#endif //FROGS_SGA_HPP
