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

class Cbma final : public HeuristicInterface, public StatsInterface {
private:
    std::vector<std::vector<int>> elites;
    std::vector<std::vector<int>> non_elites;
    std::vector<std::vector<int>> immigrants;
    std::vector<std::vector<int>> offspring;
    vector<int> indices;

    int chromosome_length;
    vector<vector<int>> temp_dumb_routes;
    vector<int> temp_child1;
    vector<int> temp_child2;
    std::unordered_map<int, int> temp_cx_map1;
    std::unordered_map<int, int> temp_cx_map2;
    vector<Individual> temp_best_individuals;
    std::vector<std::pair<double, HistoryTag>> temp_history_list;
    mutable std::deque<double> recent_improvements_pool;
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

    int max_neigh_attempts;
    int max_chain_length;

    Indicators after_local_opt;
    Indicators after_neighbour_explore;

    Initializer* initializer;
    std::vector<std::unique_ptr<LeaderCbma>> leaders;
    std::vector<std::unique_ptr<Follower>> followers;
    std::vector<std::unique_ptr<PartialSolution>> partial_sols;

    int get_luby(int j) const;

    Cbma(int seed, Case *instance, Preprocessor* preprocessor);
    ~Cbma() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    void open_log_for_evolution() override;
    void close_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void save_log_for_solution() override;
    static size_t get_dynamic_window(double gap, size_t base_window = 10, size_t min_win = 5, size_t max_win = 150);
    static bool stuck_in_local_optima(const std::deque<double>& improvements, double epsilon, size_t window_size,
                                      double strong_delta_thresh);
    int perform_neighbourhood_explore(
            int individual_index,
            int& luby_index,
            Individual& temp_best,
            std::shared_ptr<Individual>& ind,
            std::vector<std::pair<double, HistoryTag>>& history_list,
            bool debug);
    static std::string tag_to_str(HistoryTag tag);
    static void save_vector_to_csv(const std::vector<std::pair<double, HistoryTag>>& history_list, const std::string& filename) ;

    static shared_ptr<Individual> select_best_upper_individual(const vector<shared_ptr<Individual>>& pop);
    static shared_ptr<Individual> select_best_lower_individual(const vector<shared_ptr<Individual>>& pop);
    vector<vector<int>> select_random(const vector<vector<int>>& chromosomes, int k);
    void cx_partially_matched(vector<int>& parent1, vector<int>& parent2);
    void mut_shuffle_indexes(vector<int>& chromosome, double ind_pb);

    static vector<double> get_fitness_vector_from_upper_group(const vector<shared_ptr<Individual>>& group) ;
    static vector<double> get_fitness_vector_from_lower_group(const vector<shared_ptr<Individual>>& group) ;
};

#endif //FROGS_CBMA_HPP
