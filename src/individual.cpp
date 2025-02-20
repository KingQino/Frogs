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

double Individual::broken_pairs_distance(const Individual* ind) const
{
    int differences = 0;
    for (int j = 1; j <= instance->num_customer_; j++) {
        if (successors[j] != ind->successors[j] && successors[j] != ind->predecessors[j]) differences++;
        if (predecessors[j] == 0 && ind->predecessors[j] != 0 && ind->successors[j] != 0) differences++;
    }

    return static_cast<double>(differences)/static_cast<double>(instance->num_customer_);
}

double Individual::average_broken_pairs_distance_closest(const int nb_closest) const {
    double result = 0 ;
    const int max_size = std::min<int>(nb_closest, static_cast<int>(proximate_individuals.size()));
    auto it = proximate_individuals.begin();
    for (int i=0 ; i < max_size; i++) {
        result += it->first ;
        ++it ;
    }

    return result/static_cast<double>(max_size) ;
}

void Individual::evaluate_upper_cost() {
    upper_cost.reset();
    for (int r = 0; r < preprocessor->route_cap_; r++) {
        if (!chromR[r].empty())
        {
            double distance = instance->get_distance(instance->depot_, chromR[r][0]);
            double load = preprocessor->customers_[chromR[r][0]].demand;
            double service = preprocessor->customers_[chromR[r][0]].service_duration;
            predecessors[chromR[r][0]] = instance->depot_;
            for (int i = 1; i < (int)chromR[r].size(); i++)
            {
                distance += instance->get_distance(chromR[r][i-1], chromR[r][i]);
                load += preprocessor->customers_[chromR[r][i]].demand;
                service += preprocessor->customers_[chromR[r][i]].service_duration;
                predecessors[chromR[r][i]] = chromR[r][i-1];
                successors[chromR[r][i-1]] = chromR[r][i];
            }
            successors[chromR[r][chromR[r].size()-1]] = instance->depot_;
            distance += instance->get_distance(chromR[r][chromR[r].size() - 1], instance->depot_);
            upper_cost.distance += distance;
            upper_cost.nb_routes++;
            if (load > instance->max_vehicle_capa_) upper_cost.capacity_excess += load - instance->max_vehicle_capa_;
            // The time spent on the route is directly proportional to the distance traveled, plus the service time at each customer
            // Here, assume that the time spent is equal to 1 by the distance traveled.
            if (distance + service > instance->max_service_time_) upper_cost.duration_excess += distance + service - instance->max_service_time_;
        }
    }

    upper_cost.penalised_cost = upper_cost.distance + upper_cost.capacity_excess * preprocessor->penalty_capacity_ + upper_cost.duration_excess * preprocessor->penalty_duration_;
    is_upper_feasible = (upper_cost.capacity_excess < MY_EPSILON && upper_cost.duration_excess < MY_EPSILON);
}