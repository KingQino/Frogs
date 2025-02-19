//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_FOLLOWER_HPP
#define FROGS_FOLLOWER_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include <list>

#define INFEASIBLE 1'000'000'000L


class Follower {
public:

    Case* instance;
    Preprocessor* preprocessor;

    /* Auxiliary data structures to run the follower (i.e., lower optimisation) algorithm */
    int num_routes;                        // Number of routes
    int** lower_routes;
    int*  lower_num_nodes_per_route;
    double lower_cost;

    double insert_station_by_simple_enum(int* repaired_route, int& repaired_length);
    double insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const;
    void recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost, int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance);
    void run(Individual* ind);
    void load_individual(Individual* ind);
    void export_individual(Individual* ind) const;
    Follower(Case* instance, Preprocessor* preprocessor);
    ~Follower();

    friend ostream& operator<<(ostream& os, const Follower& follower);
};

#endif //FROGS_FOLLOWER_HPP
