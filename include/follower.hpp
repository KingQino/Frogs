//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_FOLLOWER_HPP
#define FROGS_FOLLOWER_HPP

#include "case.hpp"
#include "preprocessor.hpp"
#include "individual.hpp"
#include "solution.hpp"
#include <list>
#include <stack>

#define INFEASIBLE 1'000'000'000L

// This structure is used in the "enumerate stations"
struct State {
    int m_len{}, n_len{}, i{}, stationIdx{}; // Current state variables
};

// This struct is used to store the charging station information for the given route
struct ChargingMeta {
    double cost{}; // cost after applying recharging decision
    int num_stations{}; // num of stations
    vector<int> chosen_pos; // chosen position
    vector<int> chosen_sta; // chosen station
};

class Follower {
public:

    Case* instance;
    Preprocessor* preprocessor;

    /* Auxiliary data structures to run the follower (i.e., lower optimisation) algorithm */
    int route_cap;
    int node_cap;
    int num_routes;                        // Number of routes
    int** lower_routes;
    int*  lower_num_nodes_per_route;
    double lower_cost;

    double insert_station_by_simple_enum(int* repaired_route, int& repaired_length);
    double insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const;
    void recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost, int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance);
    double insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const;
    ChargingMeta try_enumerate_n_stations_to_route(int m_len, int n_len, int* chosen_sta, int* chosen_pos, double& cost,
                                                   int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance) const;


    void clean();
    void refine(Individual* ind);
    void run(Individual* ind);
    void load_individual(const Individual* ind);
    void export_individual(Individual* ind) const;
    void run(Solution* sol);
    void load_solution(const Solution* sol);
    void export_solution(Solution* sol) const;
    Follower(Case* instance, Preprocessor* preprocessor);
    ~Follower();

    friend ostream& operator<<(ostream& os, const Follower& follower);
};

#endif //FROGS_FOLLOWER_HPP
