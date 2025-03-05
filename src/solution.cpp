//
// Created by Yinghao Qin on 05/03/2025.
//
#include "solution.hpp"

Solution::Solution() {
    this->lower_cost = numeric_limits<double>::max();
    this->route_cap = 0;  // Fix: Initialize route_cap to avoid undefined behavior
    this->routes = nullptr;
}

Solution::Solution(const Solution &sol) {
    this->route_cap = sol.route_cap;
    this->node_cap = sol.node_cap;
    this->num_routes = sol.num_routes;
    this->upper_cost = sol.upper_cost;
    this->lower_cost = sol.lower_cost;
    this->routes = new int *[sol.route_cap];
    for (int i = 0; i < sol.route_cap; ++i) {
        this->routes[i] = new int[sol.node_cap];
        memcpy(this->routes[i], sol.routes[i], sizeof(int) * sol.node_cap);
    }
    this->num_nodes_per_route = new int[sol.route_cap];
    memcpy(this->num_nodes_per_route, sol.num_nodes_per_route, sizeof(int) * sol.route_cap);
    this->demand_sum_per_route = new int[sol.route_cap];
    memcpy(this->demand_sum_per_route, sol.demand_sum_per_route, sizeof(int) * sol.route_cap);
}

Solution::Solution(Case *instance, Preprocessor *preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    this->route_cap = preprocessor->route_cap_;
    this->node_cap = preprocessor->node_cap_;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);
    this->num_routes = 0;
    this->upper_cost = 0.;
    this->lower_cost = 0.;
}

Solution::Solution(Case* instance, Preprocessor* preprocessor, const vector<vector<int>> &routes, double upper_cost, const vector<int> &demand_sum_per_route)
: Solution(instance, preprocessor) {
    this->upper_cost = upper_cost;
    this->num_routes = static_cast<int>(routes.size());
    for (int i = 0; i < this->num_routes; ++i) {
        this->num_nodes_per_route[i] = static_cast<int>(routes[i].size());
        for (int j = 0; j < this->num_nodes_per_route[i]; ++j) {
            this->routes[i][j] = routes[i][j];
        }
    }
    for (int i = 0; i < demand_sum_per_route.size(); ++i) {
        this->demand_sum_per_route[i] = demand_sum_per_route[i];
    }
}

Solution::~Solution() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->routes[i];
    }
    delete[] this->routes;
    delete[] this->num_nodes_per_route;
    delete[] this->demand_sum_per_route;
}