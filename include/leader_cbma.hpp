//
// Created by Yinghao Qin on 22/04/2025.
//

#ifndef FROGS_LEADER_CBMA_HPP
#define FROGS_LEADER_CBMA_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include "solution.hpp"
#include <functional>  // For std::function
#include <unordered_set>


class LeaderCbma {
private:
    mutable int* temp_r1 = nullptr;
    mutable int* temp_r2 = nullptr;
    mutable int temp_buffer_size = 0;

    void prepare_temp_buffers(int required_size) const;
public:
    Case* instance;
    Preprocessor* preprocessor;
    std::mt19937& random_engine;   // Random number generator
    uniform_int_distribution<int> uniform_int_dis;// Uniform distribution for random integers

    int route_cap;
    int node_cap;
    int** routes;
    int num_routes;
    int* num_nodes_per_route;
    int* demand_sum_per_route;
    double upper_cost;
    int moves_count;
    vector<int> move_indices;
    unordered_set<pair<int, int>, PairHash> route_pairs;
    int k_active_moves;
    uniform_int_distribution<int> k_active_moves_dist;
    vector<int> active_moves;

    void run(Individual* ind);
    void run_plus(Individual* ind);
    void load_individual(Individual* ind);
    void export_individual(Individual* ind) const;
    LeaderCbma(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~LeaderCbma();

    void clean();
    void clean_empty_routes(int r1, int r2); // clean possible empty routes after move
    static void moveItoJ(int* route, int a, int b);

    // Operators for CBMA
    void two_opt_for_route(int* route, int length);
    void two_opt_for_sol();
    bool two_opt_star_for_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    void two_opt_star_for_sol();
    bool node_relocation_for_route(int* route, int length);
    void node_relocation_for_sol();


    /* Local search until it can no longer improve */
    [[nodiscard]] static bool is_accepted_impro(const double& change);
    // wrapper function - search until no improvement
    bool perform_intra_move_impro(const std::function<bool(int*, int)>& move_func) const;
    bool perform_inter_move_impro(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    // basic operators - neighbourhood size O(n^2)
    bool move1_intra_impro(int* route, int length); // if U is ahead of V, then move U to the behind of V; otherwise, move U to the ahead of V
    bool move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move2_intra_impro(int* route, int length);
    bool move2_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move3_intra_impro(int* route, int length);
    bool move3_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move4_intra_impro(int* route, int length);
    bool move4_inter_impro(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move5_intra_impro(int* route, int length);
    bool move5_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move6_intra_impro(int* route, int length);
    bool move6_inter_impro(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move7_intra_impro(int* route, int length);
    bool move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);


    friend ostream& operator<<(ostream& os, const LeaderCbma& leader);
};

#endif //FROGS_LEADER_CBMA_HPP
