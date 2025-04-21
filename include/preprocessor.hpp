//
// Created by Yinghao Qin on 12/02/2025.
//

#ifndef FROGS_PREPROCESSOR_HPP
#define FROGS_PREPROCESSOR_HPP

#include "case.hpp"
#include "parameters.hpp"
#include "CircleSector.h"
#include <random>


#define MY_EPSILON 0.000'01       // Precision parameter, used to avoid numerical instabilities

// Hash function for pairs of route indices
struct PairHash
{
    size_t operator() (pair<int, int> const & a_pair) const {
        return a_pair.first * 256 + a_pair.second;
    }
};

struct Customer {
    int id{};                     // Index of the customer
    double coord_x{};             // Coordinate X
    double coord_y{};             // Coordinate Y
    double service_duration{};    // Service duration
    int demand{};                 // Demand
    int polar_angle{};            // Polar angle of the customer around the depot, measured in degrees and truncated for convenience
};

class Preprocessor {
    static const int MAX_EVALUATION_FACTOR;

public:
    const Case& c;
    const Parameters& params;

    // Stop criteria
    double max_evals_{};            // the max number of evaluations can be used
    int max_exec_time_{};           // the max execution time of the algorithm
    int max_no_improvement_count_{};// the max number of iterations without improvement

    int nb_granular_{};             // Granular search parameter, limits the neighbourhood size of each solution in the local search move
    int max_demand_{};              // the max customer demand among all customers
    int total_demand_{};            // the total customer demand
    int route_cap_{};               // the capacity of routes, 3 * the number of vehicles
    int node_cap_{};                // the capacity of node per route, the number of customers + 1
    double max_cruise_distance_{};  // the max cruise distance the vehicle can travel without recharging the battery
    double max_distance_{};         // the longest arc distance between any two nodes
    double penalty_capacity_{};	    // Penalty for one unit of capacity excess (adapted through the search)
    double penalty_duration_{};		// Penalty for one unit of duration excess (adapted through the search)
    bool is_duration_constraint_{}; // Whether to consider duration constraint

    vector<int> customer_ids_;      // the id of customers
    vector<int> station_ids_;       // the id of charging stations
    vector<Customer> customers_;    // the information list of customers

    vector<vector<int>> sorted_nearby_customers_;   // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {index 1: [5,3,2,6], index 2: [], ...}
    vector<vector<int>> correlated_vertices_;       // Neighborhood restrictions: For each client, list of nearby customers
    vector<vector<int>> best_station_;              // For each pair of customers, the best station to visit, i.e., the station that minimizes the extra cost

    Preprocessor(const Case& c, const Parameters& params);

    [[nodiscard]] int get_best_station(int from, int to) const;
    [[nodiscard]] int get_best_and_feasible_station(int from, int to, double max_dis) const; // the station within allowed max distance from "from", and min dis[from][s]+dis[to][s]

};


#endif //FROGS_PREPROCESSOR_HPP
