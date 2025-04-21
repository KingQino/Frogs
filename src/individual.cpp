//
// Created by Yinghao Qin on 15/02/2025.
//

#include "individual.hpp"
#include <random>
#include <algorithm>

PartialSolution::PartialSolution() {
    this->move_type = -1;
    this->idx1 = -1;
    this->idx2 = -1;
    this->route1 = nullptr;
    this->route2 = nullptr;
    this->length1 = 0;
    this->length2 = 0;
    this->is_empty1 = true;
    this->is_empty2 = true;

    this->num_routes = 0;
}

void PartialSolution::clean() {
    this->move_type = -1;
    this->idx1 = -1;
    this->idx2 = -1;
    this->route1 = nullptr;
    this->route2 = nullptr;
    this->length1 = 0;
    this->length2 = 0;
    this->is_empty1 = true;
    this->is_empty2 = true;

    this->num_routes = 0;
}

void PartialSolution::set_intra_route(int idx, int* route, int length) {
    this->move_type = 0;

    this->idx1 = idx;
    this->route1 = route;
    this->length1 = length;
    this->is_empty1 = false;
}

void PartialSolution::set_inter_route(int r1, int *rout1, int len1, bool emp1, int r2, int *rout2, int len2,
                                      bool emp2) {
    this->move_type = 1;

    this->idx1 = r1;
    this->route1 = rout1;
    this->length1 = len1;
    this->is_empty1 = emp1;

    this->idx2 = r2;
    this->route2 = rout2;
    this->length2 = len2;
    this->is_empty2 = emp2;
}

std::ostream &operator<<(std::ostream &os, const PartialSolution &partial_sol) {
    os << "PartialSolution: move_type = " << partial_sol.move_type << "\n";
    os << "num_routes = " << partial_sol.num_routes << "\n";
    os << "idx1 = " << partial_sol.idx1
       << ", length1 = " << partial_sol.length1 << ", is_empty1 = " << partial_sol.is_empty1
       << ", idx2 = " << partial_sol.idx2 << ", length2 = " << partial_sol.length2
       << ", is_empty2 = " << partial_sol.is_empty2;
    return os;
}


Individual::Individual() {
    this->lower_cost = numeric_limits<double>::max();
    this->route_cap = 0;  // Fix: Initialize route_cap to avoid undefined behavior
    this->routes = nullptr;
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
    for (int i = 0; i < ind.route_cap; ++i) {
        this->routes[i] = new int[ind.node_cap];
        memcpy(this->routes[i], ind.routes[i], sizeof(int) * ind.node_cap);
    }
    this->num_nodes_per_route = new int[ind.route_cap];
    memcpy(this->num_nodes_per_route, ind.num_nodes_per_route, sizeof(int) * ind.route_cap);
    this->demand_sum_per_route = new int[ind.route_cap];
    memcpy(this->demand_sum_per_route, ind.demand_sum_per_route, sizeof(int) * ind.route_cap);
}

Individual::Individual(Case* instance, Preprocessor *preprocessor) {
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


Individual::Individual(Case* instance, Preprocessor* preprocessor, const vector<vector<int>>& routes, double upper_cost,
                       const vector<int>& demand_sum_per_route)
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
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;
}

vector<int> Individual::get_chromosome() const {
    vector<int> chromosome; // num of customers
    chromosome.reserve(instance->num_customer_); // Preallocate memory for efficiency
    for (int i = 0; i < num_routes; ++i) {
        for (int j = 1; j < num_nodes_per_route[i] - 1; ++j) {
            chromosome.push_back(routes[i][j]);
        }
    }

    return chromosome;
}

std::ostream& operator<<(std::ostream& os, const Individual& ind) {
    os << "Individual Details:\n";
    os << "Route Capacity: " << ind.route_cap << "\n";
    os << "Node Capacity: " << ind.node_cap << "\n";
    os << "Number of Routes: " << ind.num_routes << "\n";
    os << "Upper Cost: " << ind.upper_cost << "\n";
    os << "Lower Cost: " << ind.lower_cost << "\n";

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

    return os;
}