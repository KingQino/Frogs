//
// Created by Yinghao Qin on 24/02/2025.
//

#ifndef FROGS_LEADER_ARRAY_HPP
#define FROGS_LEADER_ARRAY_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include "solution.hpp"

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
    void neighbour_explore(const double& history_val);
    void load_individual(Individual* ind);
    void export_individual(Individual* ind) const;
    void load_solution(Solution* sol);
    void export_solution(Solution* sol) const;
    LeaderArray(int seed_val, Case* instance, Preprocessor* preprocessor);
    ~LeaderArray();


    static void moveItoJ(int* route, int a, int b);
    [[nodiscard]] bool is_accepted(const double& change) const;
    bool two_opt_for_single_route(int* route, int length);
    bool two_opt_intra_for_individual();
    bool two_opt_star_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, int* temp_r1, int* temp_r2);
    bool two_opt_inter_for_individual();
    bool node_relocation_for_single_route(int* route, int length);
    bool node_relocation_intra_for_individual(); // three-arcs exchange, intra-route
    bool node_relocation_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2);
    bool node_relocation_inter_for_individual(); // three-arcs exchange, inter-route
    bool node_exchange_for_single_route(int* route, int length);
    bool node_exchange_intra_for_individual(); // four-arcs exchange, intra-route
    bool node_exchange_between_two_routes(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2);
    bool node_exchange_inter_for_individual(); // four-arcs exchange, inter-route

    friend ostream& operator<<(ostream& os, const LeaderArray& leader);
};

#endif //FROGS_LEADER_ARRAY_HPP
