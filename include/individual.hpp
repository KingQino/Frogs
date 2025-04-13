//
// Created by Yinghao Qin on 15/02/2025.
//

#ifndef FROGS_INDIVIDUAL_HPP
#define FROGS_INDIVIDUAL_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <set>
#include "case.hpp"
#include "preprocessor.hpp"

using namespace std;

class Individual {
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

    int** lower_routes;               // the solution of lower-level sub-problem
    int*  lower_num_nodes_per_route{};  // the node number of each route in the lower-level solution
    double lower_cost{};              // the lower cost of the solution


    Individual();                                                                   // Constructor: empty individual
    Individual(const Individual& ind);                                              // Copy constructor
    Individual(Case* instance, Preprocessor* preprocessor);                         // Constructor: random individual
    Individual(Case* instance, Preprocessor* preprocessor, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route);  // Constructor: some delicate methods for initialisation
    ~Individual();


    friend ostream& operator<<(ostream& os, const Individual& individual);
};


#endif //FROGS_INDIVIDUAL_HPP
