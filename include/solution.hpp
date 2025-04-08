//
// Created by Yinghao Qin on 05/03/2025.
//

#ifndef FROGS_SOLUTION_HPP
#define FROGS_SOLUTION_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <random>
#include <algorithm>
#include "case.hpp"
#include "preprocessor.hpp"

using namespace std;

class Solution {
public:
    Case* instance{};
    Preprocessor* preprocessor{};

    int route_cap{};
    int node_cap{};

    int** routes;                     // the CVRP solution
    int num_routes{};                 // the number of routes for the solution
    int* num_nodes_per_route{};       // the node number of each route
    int* demand_sum_per_route{};      // the demand sum of all customers of each route
    double upper_cost{};              // the upper cost of the solution
    double lower_cost{};              // the lower cost of the solution

    Solution();
    Solution(const Solution& sol);
    Solution(Case* instance, Preprocessor* preprocessor);
    Solution(Case* instance, Preprocessor* preprocessor, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route);
    ~Solution();

    [[nodiscard]] vector<int> get_chromosome() const;
};

#endif //FROGS_SOLUTION_HPP
