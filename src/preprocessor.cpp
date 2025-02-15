//
// Created by Yinghao Qin on 12/02/2025.
//

#include "preprocessor.hpp"

Preprocessor::Preprocessor(const Case &c, const Parameters &params) : c(c), params(params) {

    this->max_demand_ = *std::max_element(c.demand_.begin(), c.demand_.end());
    this->total_demand_ = std::accumulate(c.demand_.begin(), c.demand_.end(), 0);
    this->route_cap_ = 3 * c.num_vehicle_;
    this->max_cruise_distance_ = c.max_battery_capa_ / c.energy_consumption_rate_;
    for (int i = 0; i < c.problem_size_; i++) {
        for (int j = 0; j < c.problem_size_; j++) {
            if (c.distances_[i][j] > max_distance_) max_distance_ = c.distances_[i][j];
        }
    }

    // A reasonable scale for the initial values of the penalties
    this->penalty_capacity_ = std::max<double>(0.1, std::min<double>(1000., max_distance_ / max_demand_));
    this->penalty_duration_ = 1;

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

    this->sorted_nearby_customers = vector<vector<int>>(c.num_customer_ + 1);
    for (int i = 1; i <= c.num_customer_; i++) {
        for (auto node : customer_ids_) {
            if (node == i) continue;
            sorted_nearby_customers[i].push_back(node);
        }

        sort(sorted_nearby_customers[i].begin(), sorted_nearby_customers[i].end(), [&](const int a, const int b) {
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
        for (int j = 0; j < std::min<int>(params.nb_granular, c.num_customer_ - 1); ++j) {
            //  if i is correlated with j, then j should be correlated with i
            set_correlated_vertices[i].insert(order_proximity[j].second);
            set_correlated_vertices[order_proximity[j].second].insert(i);
        }
    }

    // Make charging decision, filling the vector with correlated vertices
    this->best_stations_ = std::vector<std::vector<int>>(c.num_depot_ + c.num_customer_,std::vector<int>(c.num_depot_ + c.num_customer_));
    for (int i = 0; i < c.num_depot_ + c.num_customer_ - 1; i++) {
        for (int j = i + 1; j < c.num_depot_ + c.num_customer_; j++) {
            this->best_stations_[i][j] = this->best_stations_[j][i] = get_best_station(i, j);
        }
    }

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