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

typedef struct tUpperCost  {
    double penalised_cost{};        // Penalized cost of the solution
    int nb_routes{};				// Number of routes
    double distance{};			    // Total Distance
    double capacity_excess{};		// Sum of excess load in all routes
    double duration_excess{};		// Sum of excess duration in all routes
} UpperCost;

class Individual {
public:
    Case* instance{};
    Preprocessor* preprocessor{};

    // chromosome information, for evolution
    vector<int> chromT;             // Giant tour representing the individual
    vector<vector<int>> chromR;     // For each vehicle, the associated sequence of deliveries (complete solution)
    UpperCost upper_cost;           // The cost of upper-level solution
    bool is_upper_feasible{};       // Feasibility status of the individual
    double lower_cost{};            // The cost of lower-level solution, i.e., the complete solution

    // Ma: population diversity control
    double biased_fitness{};        // The biased fitness. The smaller, the better.
    vector<int> successors;         // For each node, the successor in the solution (can be the depot 0)
    vector<int> predecessors;       // For each node, the predecessor in the solution (can be the depot 0)
    multiset<pair<double, Individual*>> proximate_individuals; // The other individuals in the population, ordered by increasing proximity (the set container follows a natural ordering based on the first value of the pair)

    Individual(const Individual& ind);                                              // Copy constructor
    Individual(Case* instance, Preprocessor* preprocessor);                         // Constructor: random individual
    Individual(Case* instance, Preprocessor* preprocessor, const vector<int>& chromT);     // Constructor: random individual, the next step is to use `Split` to generate the ChromR
    Individual(Case* instance, Preprocessor* preprocessor, const vector<int>& chromT, const vector<vector<int>>& chromR, double upper_cost);  // Constructor: some delicate methods for initialisation

    void evaluate_upper_cost();                                                     // Measuring cost of a solution from the information of chromR
    double broken_pairs_distance(Individual* ind);                                  // Distance measure with another individual
    double average_broken_pairs_distance_closest(int nb_closest);                   // Returns the average distance of this individual with the nbClosest individuals
};


#endif //FROGS_INDIVIDUAL_HPP
