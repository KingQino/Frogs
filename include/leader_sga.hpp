//
// Created by Yinghao Qin on 13/04/2025.
//

#ifndef FROGS_LEADER_SGA_HPP
#define FROGS_LEADER_SGA_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <functional>  // For std::function
#include <unordered_set>


class LeaderSga {
private:
    mutable int* temp_r1 = nullptr;
    mutable int* temp_r2 = nullptr;
    mutable int temp_buffer_size = 0;

    mutable std::vector<int> temp_candidates;

    void prepare_temp_buffers(int required_size) const;
public:
    Case* instance;
    Preprocessor* preprocessor;
    std::mt19937& random_engine;   // Random number generator
    uniform_int_distribution<int> uniform_int_dis;// Uniform distribution for random integers

    PartialSolution* partial_sol;

    int route_cap;
    int node_cap;
    int** routes;
    int num_routes;
    int* num_nodes_per_route;
    int* demand_sum_per_route;
    int max_search_depth;
    double upper_cost;
    double border_cost;
    int moves_count;
    vector<int> move_indices;
    unordered_set<pair<int, int>, PairHash> route_pairs;


    void local_improve(Individual* ind);
    bool neighbour_explore(const double& border_val, PartialSolution* partial_ind);
    void run(Individual* ind);
    void load_individual(Individual* ind);
    void export_individual(Individual* ind) const;
    LeaderSga(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~LeaderSga();


    void clean();
    void clean_empty_routes(int r1, int r2); // clean possible empty routes after move
    static void moveItoJ(int* route, int a, int b);

    /* Local search until it can no longer improve */
    [[nodiscard]] static bool is_accepted_impro(const double& change);
    // wrapper function - search until no improvement
    bool perform_intra_move_impro(const std::function<bool(int*, int)>& move_func) const;
    bool perform_inter_move_impro(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    bool perform_inter_move_with_empty_impro(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
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
    bool move1_inter_with_empty_route_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);


    /* Neighbourhood exploration attempts all possible neighbour solutions - it jumps to any neighbouring solution whose cost is below a given threshold */
    [[nodiscard]] bool is_accepted_neigh(const double& change) const;
    // wrapper function - random single route or route pair
    bool perform_intra_move_neigh(const std::function<bool(int*, int)>& move_func);
    bool perform_inter_move_neigh(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    // basic operators - neighbourhood size O(n), random node u
    bool move1_intra_neigh(int* route, int length); // if U is ahead of V, then move U to the behind of V; otherwise, move U to the ahead of V
    bool move1_inter_neigh(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move4_intra_neigh(int* route, int length);
    bool move4_inter_neigh(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move7_intra_neigh(int* route, int length);
    bool move8_inter_neigh(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter_neigh(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);


    friend ostream& operator<<(ostream& os, const LeaderSga& leader);
};


#endif //FROGS_LEADER_SGA_HPP
