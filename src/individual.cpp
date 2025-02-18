//
// Created by Yinghao Qin on 15/02/2025.
//

#include "individual.hpp"
#include <random>
#include <algorithm>


Individual::Individual(const Individual &ind) {
    this->instance = ind.instance;
    this->predecessors = ind.predecessors;
    this->chromT = ind.chromT;
    this->chromR = ind.chromR;
    this->upper_cost = ind.upper_cost;
    this->lower_cost = ind.lower_cost;
    this->is_upper_feasible = ind.is_upper_feasible;
    this->biased_fitness = ind.biased_fitness;
    this->successors = ind.successors;
    this->predecessors = ind.predecessors;
    this->proximate_individuals = ind.proximate_individuals;
}

Individual::Individual(Case* instance, Preprocessor *preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    this->successors = vector<int>(instance->num_customer_ + 1);
    this->predecessors = vector<int>(instance->num_customer_ + 1);
    this->chromR = vector<vector<int>>(preprocessor->route_cap_, vector<int>());
    this->chromT = vector<int>(instance->num_customer_);
}

Individual::Individual(Case* instance, Preprocessor* preprocessor, const vector<int>& chromT)
: Individual(instance, preprocessor) {
    this->chromT = chromT;
}

Individual::Individual(Case* instance, Preprocessor* preprocessor, const vector<int>& chromT, const vector<vector<int>>& chromR, double upper_cost)
: Individual(instance, preprocessor) {
    this->chromT = chromT;
    for (int i = 0; i < chromR.size(); ++i) {
        this->chromR[i] = chromR[i];
    }
    this->upper_cost.penalised_cost = upper_cost;
    this->upper_cost.distance = upper_cost;
    this->upper_cost.nb_routes = static_cast<int>(chromR.size());
    this->upper_cost.capacity_excess = 0;
    this->upper_cost.duration_excess = 0;
    this->is_upper_feasible = true; // assume that all initialised solutions are feasible
}

double Individual::broken_pairs_distance(Individual *ind) {
    int differences = 0;
    for (int j = 1; j <= instance->num_customer_; j++) {
        if (successors[j] != ind->successors[j] && successors[j] != ind->predecessors[j]) differences++;
        if (predecessors[j] == 0 && ind->predecessors[j] != 0 && ind->successors[j] != 0) differences++;
    }

    return (double)differences/(double)instance->num_customer_;
}

double Individual::average_broken_pairs_distance_closest(int nb_closest) {
    double result = 0 ;
    int max_size = std::min<int>(nb_closest, static_cast<int>(proximate_individuals.size()));
    auto it = proximate_individuals.begin();
    for (int i=0 ; i < max_size; i++) {
        result += it->first ;
        ++it ;
    }

    return result/(double)max_size ;
}
