//
// Created by Yinghao Qin on 12/02/2025.
//

#include "preprocessor.hpp"

const int Preprocessor::MAX_EVALUATION_FACTOR = 25'000;

Preprocessor::Preprocessor(const Case &c, const Parameters &params) : c(c), params(params) {

    // Stop criteria setting up
    this->max_evals_ = c.problem_size_ * MAX_EVALUATION_FACTOR;
    if (c.num_customer_ <= 100) {
        max_exec_time_ = static_cast<int>(1 * (c.problem_size_ / 100.0) * 60 * 60);
    } else if (c.num_customer_ <= 915) {
        max_exec_time_ = static_cast<int>(2 * (c.problem_size_ / 100.0) * 60 * 60);
    } else {
        max_exec_time_ = static_cast<int>(3 * (c.problem_size_ / 100.0) * 60 * 60);
    }
    this->max_no_improvement_count_ = 800; // adjust further  default: 20,000

    this->nb_granular_ = params.nb_granular;
    this->max_demand_ = *std::max_element(c.demand_.begin(), c.demand_.end());
    this->total_demand_ = std::accumulate(c.demand_.begin(), c.demand_.end(), 0);
    this->route_cap_ = 3 * c.num_vehicle_;
    this->max_cruise_distance_ = c.max_battery_capa_ / c.energy_consumption_rate_;
    for (int i = 0; i < c.problem_size_; i++) {
        for (int j = 0; j < c.problem_size_; j++) {
            if (c.distances_[i][j] > max_distance_) max_distance_ = c.distances_[i][j];
        }
    }

    if (params.is_hard_constraint) {
        // A great penalty for the hard constraint
        this->penalty_capacity_ = 1e10;
        this->penalty_duration_ = 1e10;
    } else {
        // A reasonable scale for the initial values of the penalties
        this->penalty_capacity_ = std::max<double>(0.1, std::min<double>(1000., max_distance_ / max_demand_));
        this->penalty_duration_ = 1;
    }
    this->is_duration_constraint_ = params.is_duration_constraint;

    this->customers_ = vector<Customer>(c.num_customer_ + 1);
    customers_[0].coord_x = c.positions_[0].first;
    customers_[0].coord_y = c.positions_[0].second;
    for (int i = 1; i < c.num_depot_ + c.num_customer_; ++i) {
        customer_ids_.push_back(i);
        customers_[i].id = i;
        customers_[i].coord_x = c.positions_[i].first;
        customers_[i].coord_y = c.positions_[i].second;
        customers_[i].demand = c.demand_[i];
        customers_[i].polar_angle = CircleSector::positive_mod(static_cast<int>(32768.*atan2(customers_[i].coord_y - customers_[0].coord_y, customers_[i].coord_x - customers_[0].coord_x) / PI) );
    }
    for (int i = c.num_depot_ + c.num_customer_; i < c.problem_size_; ++i) {
        station_ids_.push_back(i);
    }

    this->sorted_nearby_customers_ = vector<vector<int>>(c.num_customer_ + 1);
    for (int i = 1; i <= c.num_customer_; i++) {
        for (auto node : customer_ids_) {
            if (node == i) continue;
            sorted_nearby_customers_[i].push_back(node);
        }

        sort(sorted_nearby_customers_[i].begin(), sorted_nearby_customers_[i].end(), [&](const int a, const int b) {
            return c.distances_[i][a] < c.distances_[i][b];
        });
    }

    // Local search (acceleration), Calculation of the correlated vertices for each customer (for the granular restriction)
    correlated_vertices_ = vector<vector<int>>(c.num_customer_ + 1);
    vector<set<int>> set_correlated_vertices = vector<set<int>>(c.num_customer_ + 1);
    vector<pair<double, int>> order_proximity;
    for (int i = 1; i <= c.num_customer_; i++) {
        order_proximity.clear();
        for (int j = 1; j <= c.num_customer_; j++) {
            if (i == j) continue;
            order_proximity.emplace_back(c.distances_[i][j], j);
        }
        std::sort(order_proximity.begin(), order_proximity.end());
        for (int j = 0; j < std::min<int>(nb_granular_, c.num_customer_ - 1); ++j) {
            //  if i is correlated with j, then j should be correlated with i
            set_correlated_vertices[i].insert(order_proximity[j].second);
            set_correlated_vertices[order_proximity[j].second].insert(i);
        }
    }
    for (int i = 1; i <= c.num_customer_; i++) {
        for (int x : set_correlated_vertices[i]) {
            correlated_vertices_[i].push_back(x);
        }
    }

    // Make charging decision, filling the vector with correlated vertices
    this->best_station_ = std::vector<std::vector<int>>(c.num_depot_ + c.num_customer_,std::vector<int>(c.num_depot_ + c.num_customer_));
    for (int i = 0; i < c.num_depot_ + c.num_customer_ - 1; i++) {
        for (int j = i + 1; j < c.num_depot_ + c.num_customer_; j++) {
            this->best_station_[i][j] = this->best_station_[j][i] = get_best_station(i, j);
        }
    }

    // initialise the random engine
    this->random_engine = std::default_random_engine(this->params.seed);
}

int Preprocessor::get_best_station(const int from, const int to) const {
    int target_station = -1;
    double min_dis = std::numeric_limits<double>::max();

    for (int i = c.num_customer_ + 1 ; i < c.problem_size_; ++i) {
        if (const double dis = c.distances_[from][i] + c.distances_[to][i]; min_dis > dis && from != i && to != i) {
            target_station = i;
            min_dis = dis;
        }
    }

    return target_station;
}

int Preprocessor::get_best_and_feasible_station(const int from, const int to, const double max_dis) const {
    int target_station = -1;
    double min_dis = std::numeric_limits<double>::max();

    for (int i = c.num_customer_ + 1; i < c.problem_size_; ++i) {
        if (c.distances_[from][i] < max_dis &&
            min_dis > c.distances_[from][i]  + c.distances_[to][i]  &&
            from != i && to != i &&
            c.distances_[i][to] < max_cruise_distance_) {

            target_station = i;
            min_dis = c.distances_[from][i] + c.distances_[to][i];
        }
    }

    return target_station;
}