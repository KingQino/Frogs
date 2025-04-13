//
// Created by Yinghao Qin on 15/02/2025.
//

#include "individual.hpp"
#include <random>
#include <algorithm>

Individual::Individual() {
    this->lower_cost = numeric_limits<double>::max();
    this->route_cap = 0;  // Fix: Initialize route_cap to avoid undefined behavior
    this->routes = nullptr;
    this->lower_routes = nullptr;
}

Individual::Individual(const Individual &ind) {
    this->instance = ind.instance;
    this->preprocessor = ind.preprocessor;

    this->route_cap = ind.route_cap;
    this->node_cap = ind.node_cap;
    this->num_routes = ind.num_routes;
    this->upper_cost = ind.upper_cost;
    this->lower_cost = ind.lower_cost;
    this->routes = new int *[ind.route_cap];
    this->lower_routes = new int *[ind.route_cap];
    for (int i = 0; i < ind.route_cap; ++i) {
        this->routes[i] = new int[ind.node_cap];
        memcpy(this->routes[i], ind.routes[i], sizeof(int) * ind.node_cap);
        this->lower_routes[i] = new int[ind.node_cap];
        memcpy(this->lower_routes[i], ind.lower_routes[i], sizeof(int) * ind.node_cap);
    }
    this->num_nodes_per_route = new int[ind.route_cap];
    memcpy(this->num_nodes_per_route, ind.num_nodes_per_route, sizeof(int) * ind.route_cap);
    this->lower_num_nodes_per_route = new int[ind.route_cap];
    memcpy(this->lower_num_nodes_per_route, ind.lower_num_nodes_per_route, sizeof(int) * ind.route_cap);
    this->demand_sum_per_route = new int[ind.route_cap];
    memcpy(this->demand_sum_per_route, ind.demand_sum_per_route, sizeof(int) * ind.route_cap);
}

Individual::Individual(Case* instance, Preprocessor *preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    this->route_cap = preprocessor->route_cap_;
    this->node_cap = preprocessor->node_cap_;
    this->routes = new int *[route_cap];
    this->lower_routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
        this->lower_routes[i] = new int[node_cap];
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->lower_num_nodes_per_route = new int[route_cap];
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);
    this->num_routes = 0;
    this->upper_cost = 0.;
    this->lower_cost = 0.;
}


Individual::Individual(Case* instance, Preprocessor* preprocessor, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route)
: Individual(instance, preprocessor) {
    this->upper_cost = upper_cost;
    this->num_routes = static_cast<int>(routes.size());
    for (int i = 0; i < this->num_routes; ++i) {
        this->num_nodes_per_route[i] = static_cast<int>(routes[i].size());
        memcpy(this->routes[i], routes[i].data(), sizeof(int) * this->num_nodes_per_route[i]);
    }
    for (int i = 0; i < demand_sum_per_route.size(); ++i) {
        this->demand_sum_per_route[i] = demand_sum_per_route[i];
    }

    this->lower_cost = numeric_limits<double>::max();
}

Individual::~Individual() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] routes[i];
        delete[] lower_routes[i];
    }
    delete[] routes;
    delete[] lower_routes;
    delete[] num_nodes_per_route;
    delete[] lower_num_nodes_per_route;
    delete[] demand_sum_per_route;
}

std::ostream& operator<<(std::ostream& os, const Individual& ind) {
    os << "Individual Details:\n";
    os << "Route Capacity: " << ind.route_cap << "\n";
    os << "Node Capacity: " << ind.node_cap << "\n";
    os << "Number of Routes: " << ind.num_routes << "\n";
    os << "Upper Cost: " << ind.upper_cost << "\n";

    os << "Number of Nodes per route (upper): ";
    for (int i = 0; i < ind.route_cap; ++i) {
        os << ind.num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < ind.route_cap; ++i) {
        os << ind.demand_sum_per_route[i] << " ";
    }
    os << "\n";

    os << "Upper Routes: \n";
    for (int i = 0; i < ind.num_routes; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < ind.num_nodes_per_route[i]; ++j) {
            os << ind.routes[i][j] << " ";
        }
        os << "\n";
    }

    os << "Number of Nodes per route (lower): ";
    for (int i = 0; i < ind.route_cap; ++i) {
        os << ind.lower_num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Lower Routes: \n";
    for (int i = 0; i < ind.num_routes; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < ind.lower_num_nodes_per_route[i]; ++j) {
            os << ind.lower_routes[i][j] << " ";
        }
        os << "\n";
    }

    return os;
}