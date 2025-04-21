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


// This structure is specifically designed to coordinate the leader and follower, to avoid pass the whole individual (too expensive)
struct PartialSolution {
    int move_type;      // the type of move, 0 for intra-route, 1 for inter-route, 2 for inter-route with empty route

    int num_routes;     // the number of routes

    int idx1;           // the index of the route 1
    int* route1;        // the route 1
    int length1;        // the number of nodes in the route 1
    bool is_empty1;     // if the route 1 is empty or not

    int idx2;           // the index of the route 2
    int* route2;        // the route 2
    int length2;        // the number of nodes in the route 2
    bool is_empty2;     // if the route 2 is empty or not

    PartialSolution();

    void clean();
    void set_intra_route(int idx, int* route, int length);
    void set_inter_route(int r1, int* rout1, int len1, bool emp1, int r2, int* rout2, int len2, bool emp2);

    friend ostream& operator<<(ostream& os, const PartialSolution& partial_sol);
};

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

//    int** lower_routes;               // the solution of lower-level sub-problem
//    int*  lower_num_nodes_per_route{};  // the node number of each route in the lower-level solution
    double lower_cost{};              // the lower cost of the solution


    Individual();                                                                   // Constructor: empty individual
    Individual(const Individual& ind);                                              // Copy constructor
    Individual(Case* instance, Preprocessor* preprocessor);                         // Constructor: random individual
    Individual(Case* instance, Preprocessor* preprocessor, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route);  // Constructor: some delicate methods for initialisation
    ~Individual();

    [[nodiscard]] vector<int> get_chromosome() const; // Get the chromosome of the individual

    friend ostream& operator<<(ostream& os, const Individual& individual);
};


#endif //FROGS_INDIVIDUAL_HPP
