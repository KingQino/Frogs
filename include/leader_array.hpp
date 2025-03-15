//
// Created by Yinghao Qin on 24/02/2025.
//

#ifndef FROGS_LEADER_ARRAY_HPP
#define FROGS_LEADER_ARRAY_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include "solution.hpp"
#include <functional>  // For std::function

class LeaderArray {
public:
    Case* instance;
    Preprocessor* preprocessor;
    std::default_random_engine random_engine;   // Random number generator
    uniform_int_distribution<int> uniform_int_dis;// Uniform distribution for random integers

    int route_cap;
    int node_cap;
    int** routes;
    int num_routes;
    int* num_nodes_per_route;
    int* demand_sum_per_route;
    int max_search_depth;
    double upper_cost;
    double history_cost;

    void clean();
    void run(Individual* ind);
    bool neighbour_explore(const double& history_val);
    void load_individual(Individual* ind);
    void export_individual(Individual* ind) const;
    void load_solution(Solution* sol);
    void export_solution(Solution* sol) const;
    LeaderArray(int seed_val, Case* instance, Preprocessor* preprocessor);
    ~LeaderArray();

    void clean_empty_routes(int r1, int r2); // clean possible empty routes after move
    bool perform_intra_move(const std::function<bool(int*, int)>& move_func);
    bool perform_inter_move(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);
    bool perform_inter_move_with_empty_route(const std::function<bool(int*, int*, int&, int&, int&, int&)>& move_func);

    [[nodiscard]] bool is_accepted(const double& change) const;
    bool move1_intra(int* route, int length); // if U is ahead of V, then move U to the behind of V; otherwise, move U to the ahead of V
    bool move1_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move1_inter_with_empty_route(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move4_intra(int* route, int length);
    bool move4_inter(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool move7_intra(int* route, int length);
    bool move8_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move8_inter_with_empty_route(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool move9_inter_with_empty_route(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);

    static void moveItoJ(int* route, int a, int b);

    friend ostream& operator<<(ostream& os, const LeaderArray& leader);
};

#endif //FROGS_LEADER_ARRAY_HPP
