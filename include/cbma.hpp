//
// Created by Yinghao Qin on 05/04/2025.
//

#ifndef FROGS_CBMA_HPP
#define FROGS_CBMA_HPP

#include "case.hpp"
#include "initializer.hpp"
#include "leader_cbma.hpp"
#include "follower.hpp"
#include "individual.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Cbma final : public HeuristicInterface, public StatsInterface {
//private:
//    std::vector<std::vector<int>> elites;
//    std::vector<std::vector<int>> offspring;
//    vector<int> indices;

public:
    static const std::string ALGORITHM;

    bool enable_logging;
    int stop_criteria;

    vector<shared_ptr<Individual>> population;
    std::unique_ptr<Individual> global_best;      // Global best solution found so far
    std::unique_ptr<Individual> iter_best;
    double global_best_upper_so_far{};            // The best solution found so far

    int pop_size;
    double elite_ratio;
    double immigrants_ratio;
    double crossover_prob;
    double mutation_prob;
    double mut_ind_prob;

    int gen; // iteration num
    double gammaL; // confidence ratio of local search: 调大可以增加local search的解的个数
    double gammaR; // confidence ratio of recharging: 调小可以增加recharging的解的个数
    int delta;  // confidence interval
    deque<double> P; // list for confidence intervals of local search
    double r; // confidence interval is used to judge whether an upper-level sub-solution should make the charging process

    Indicators before_up_opt;
    Indicators after_up_opt;
    Indicators after_low_opt;

    Initializer* initializer;
    LeaderCbma* leader;
    Follower* follower;


    Cbma(int seed, Case *instance, Preprocessor* preprocessor);
    ~Cbma() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

    static shared_ptr<Individual> select_best_upper_individual(const vector<shared_ptr<Individual>>& pop);
    static shared_ptr<Individual> select_best_lower_individual(const vector<shared_ptr<Individual>>& pop);
    vector<vector<int>> select_random(const vector<vector<int>>& chromosomes, int k);
    void cx_partially_matched(vector<int>& parent1, vector<int>& parent2);
    void mut_shuffle_indexes(vector<int>& chromosome, double ind_pb);

    static vector<double> get_fitness_vector_from_upper_group(const vector<shared_ptr<Individual>>& group) ;
    static vector<double> get_fitness_vector_from_lower_group(const vector<shared_ptr<Individual>>& group) ;
};

#endif //FROGS_CBMA_HPP
