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
#include <deque>

using namespace std;

enum HistoryTag { PERTURB, SEARCH, STUCK };

const int MAX_LOCAL_STEPS = 2'000;

class Cbma final : public HeuristicInterface, public StatsInterface {
private:
    vector<vector<int>> promising_seqs;
    vector<vector<int>> average_seqs;
    std::vector<std::vector<int>> offspring;
    vector<shared_ptr<Individual>> S1, S2, S3;

    // vector<Individual> population;                // Population of individuals
    vector<vector<int>> elites;                   // Elite
    vector<vector<int>> non_elites;               // Non-elite
    vector<vector<int>> immigrants;               // Immigrants
    // vector<vector<int>> offspring;                // Offspring produced by elites, non-elites, and immigrants

    // int gen;                                      // evolutionary generation
    // int pop_size;                                 // Population size
    int chromosome_length;                        // Length of the chromosome, i.e., the number of customers
    double mut_prob;                              // Mutation probability
    // double mut_ind_prob;                          // Mutation individual probability

    /* memory optimisation tricks */
    vector<vector<int>> temp_dumb_routes;
    vector<int> temp_child1;
    vector<int> temp_child2;
    std::unordered_map<int, int> temp_cx_map1;
    std::unordered_map<int, int> temp_cx_map2;
    vector<Individual> temp_best_individuals;
    vector<tuple<double, HistoryTag, int>> temp_history_list;
    std::deque<double> temp_recent_moves_pool;
public:
    static const std::string ALGORITHM;

    bool enable_logging;
    int stop_criteria;

    vector<shared_ptr<Individual>> population;
    std::unique_ptr<Individual> global_best;      // Global best solution found so far
    std::unique_ptr<Individual> iter_best;
    double global_best_upper_so_far{};            // The best solution found so far

    // bool enable_logging;                          // Whether to enable logging
    // int stop_criteria;                            // Stop criteria for the algorithm
    int max_neigh_attempts;                       // Maximum number of attempts for neighbourhood exploration
    int max_perturbation_strength;                // Maximum perturbation strength

    int pop_size;
    double elite_ratio;
    double immigrants_ratio;
    double crossover_prob;
    double mutation_prob;
    double mut_ind_prob;
    // unique_ptr<Individual> global_best;           // Global best solution found so far
    // double global_best_upper_so_far;              // The best solution found so far

    int gen; // iteration num
    double gammaL; // confidence ratio of local search: 调大可以增加local search的解的个数
    double gammaR; // confidence ratio of recharging: 调小可以增加recharging的解的个数
    int delta;  // confidence interval
    deque<double> P; // list for confidence intervals of local search
    double r; // confidence interval is used to judge whether an upper-level sub-solution should make the charging process

    Indicators before_up_opt;
    Indicators after_up_opt;
    Indicators after_low_opt;

    // Initializer* initializer;
    // LeaderCbma* leader;
    // Follower* follower;

    std::unique_ptr<Initializer> initializer;     // Used to generate initial solutions
    vector<std::unique_ptr<LeaderCbma>> leaders;  // Upper-level decisions, including perturbation and local search
    vector<std::unique_ptr<Follower>> followers;  // Lower-level decisions, including charging insertions
    vector<unique_ptr<PartialSolution>> partial_sols; // Partial solutions are used to coordinate the leader and follower

    // statistics
    // Indicators stats_greedy_local_opt;            // Statistics after a fully greedy local search
    // Indicators stats_neigh_explore;               // Statistics after neighbourhood exploration

    Cbma(int seed, Case *instance, Preprocessor* preprocessor);
    ~Cbma() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;

    int get_luby(int j) const;
    int neighbourhood_explore(int individual_index, int& luby_index, Individual& temp_best, Individual& ind,
                              vector<tuple<double, HistoryTag, int>>& history_list);


    static shared_ptr<Individual> select_best_upper_individual(const vector<shared_ptr<Individual>>& pop);
    static shared_ptr<Individual> select_best_lower_individual(const vector<shared_ptr<Individual>>& pop);
    vector<vector<int>> select_random(const vector<vector<int>>& chromosomes, int k);
    void cx_partially_matched(vector<int>& parent1, vector<int>& parent2);
    void mut_shuffle_indexes (vector<int>& chromosome, double ind_pb);

    static vector<double> get_fitness_vector_from_upper_group(const vector<shared_ptr<Individual>>& group) ;
    static vector<double> get_fitness_vector_from_lower_group(const vector<shared_ptr<Individual>>& group) ;

    static string tag_to_str(HistoryTag tag);
    static size_t get_dynamic_window(double gap_ratio, double min_win, double max_win, double k);
    static bool stuck_in_local_optima(const deque<double>& improvements, size_t window_size, double epsilon,
                                      double strong_delta_thresh);
    static void save_vector_to_csv(const std::vector<std::tuple<double, HistoryTag, int>>& history_list,
                                   const string& filename);
};

#endif //FROGS_CBMA_HPP
