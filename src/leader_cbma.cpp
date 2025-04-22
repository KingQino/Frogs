//
// Created by Yinghao Qin on 22/04/2025.
//

#include "leader_cbma.hpp"

LeaderCbma::LeaderCbma(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
        : instance(instance),
          preprocessor(preprocessor),
          random_engine(engine),
          uniform_int_dis(0, 7) {

    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;
    this->num_routes = 0;
    this->upper_cost = 0.;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);
}

LeaderCbma::~LeaderCbma() {
    for (int i = 0; i < route_cap; ++i) {
        delete[] routes[i];
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;
}

void LeaderCbma::run(Individual *ind) {
    load_individual(ind);

    two_opt_for_sol();
    two_opt_star_for_sol();
    node_relocation_for_sol();

    export_individual(ind);
}

void LeaderCbma::load_individual(Individual *ind) {
    clean();

    this->upper_cost = ind->upper_cost;;
    this->num_routes = ind->num_routes;
    memcpy(this->num_nodes_per_route, ind->num_nodes_per_route, sizeof(int) * ind->route_cap);
    memcpy(this->demand_sum_per_route, ind->demand_sum_per_route, sizeof(int) * ind->route_cap);
    for (int i = 0; i < ind->num_routes; ++i) {
        memcpy(this->routes[i], ind->routes[i], sizeof(int) * ind->node_cap);
    }
}

void LeaderCbma::export_individual(Individual *ind) const {
    ind->upper_cost = this->upper_cost;
    ind->num_routes = this->num_routes;
    memcpy(ind->num_nodes_per_route, this->num_nodes_per_route, sizeof(int) * this->route_cap);
    memcpy(ind->demand_sum_per_route, this->demand_sum_per_route, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->num_routes; ++i) {
        memcpy(ind->routes[i], this->routes[i], sizeof(int) * this->node_cap);
    }
}

void LeaderCbma::clean() {
    this->num_routes = 0;
    this->upper_cost = 0.;

    memset(this->num_nodes_per_route, 0, sizeof(int) * this->route_cap);
    memset(this->demand_sum_per_route, 0, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->route_cap; ++i) {
        memset(this->routes[i], 0, sizeof(int) * this->node_cap);
    }
}

void LeaderCbma::clean_empty_routes(int r1, int r2) {
    auto remove_if_empty = [&](int route) {
        if (demand_sum_per_route[route] == 0 && num_routes > 0) {
            int last = num_routes - 1;
            std::swap(routes[route], routes[last]);
            std::swap(demand_sum_per_route[route], demand_sum_per_route[last]);
            std::swap(num_nodes_per_route[route], num_nodes_per_route[last]);
            memset(routes[last], 0, sizeof(int) * node_cap);
            num_nodes_per_route[last] = 0;
            num_routes--;
        }
    };

    // clean the route beyond the length
    std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
    std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);

    if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
        int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
        remove_if_empty(empty_r);
    }

    // Clear unused routes beyond num_routes
    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
}

void LeaderCbma::moveItoJ(int* route, int a, int b) {
    int x = route[a];
    if (a < b) {
        for (int i = a; i < b; i++) {
            route[i] = route[i + 1];
        }
        route[b] = x;
    }
    else if (a > b) {
        for (int i = a; i > b; i--) {
            route[i] = route[i - 1];
        }
        route[b] = x;
    }
}

void LeaderCbma::two_opt_for_route(int *route, int length) {
    if (length < 5) return;

    double change = 0.0;
    bool improved = true;

    while (improved) {
        improved = false;

        for (size_t i = 1; i < length - 2; ++i) {
            for (size_t j = i + 1; j < length - 1; ++j) {
                // Calculate the cost difference between the old route and the new route obtained by swapping edges
                double original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
                double modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

                change = modified_cost - original_cost;
                if (fabs(change) < 1e-8) change = 0;
                if (change < 0) {
                    reverse(route + i, route + j + 1);
                    upper_cost += change;
                    improved = true;
                    goto end_loop;
                }
            }
        }

        end_loop: ;
    }
}


void LeaderCbma::two_opt_for_sol() {
    for (int i = 0; i < num_routes; ++i) {
        two_opt_for_route(routes[i], num_nodes_per_route[i]);
    }
}

bool LeaderCbma::two_opt_star_for_routes(int *route1, int *route2, int &length1, int &length2, int &loading1,
                                          int &loading2, int* temp_r1, int* temp_r2) {
    if (length1 < 3 || length2 < 3) return false;
    bool improved = true;

    bool has_moved = false;

    while (improved) {
        improved = false;

        // case1: route1 >=3, route2 >=3
        // case2: route1 >=3, route2 = 2 (0, 0)

        int partial_dem_r1 = 0;
        for (int n1 = 0; n1 < length1 - 1; ++n1) {
            partial_dem_r1 += instance->get_customer_demand_(route1[n1]);

            int partial_dem_r2 = 0;
            for (int n2 = 0; n2 < length2 - 1; ++n2) {
                partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

                if (partial_dem_r1 + loading2 - partial_dem_r2 <= instance->max_vehicle_capa_ && partial_dem_r2 + loading1 - partial_dem_r1 <= instance->max_vehicle_capa_) {
                    double old_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
                    double new_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

                    double change = new_cost - old_cost;
                    if (fabs(change) < 1e-8) change = 0;
                    if (change < 0) {
                        // update
                        upper_cost += change;
                        memcpy(temp_r1, route1, sizeof(int) * node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2 + 1; i < length2; i++) {
                            route1[counter1++] = route2[i];
                        }
                        int counter2 = n2 + 1;
                        for (int i = n1 + 1; i < length1; i++) {
                            route2[counter2++] = temp_r1[i];
                        }
                        length1 = counter1;
                        length2 = counter2;
                        int new_dem_sum_1 = partial_dem_r1 + loading2 - partial_dem_r2;
                        int new_dem_sum_2 = partial_dem_r2 + loading1 - partial_dem_r1;
                        loading1 = new_dem_sum_1;
                        loading2 = new_dem_sum_2;

                        improved = true;
                        has_moved = true;
                        goto end_loop;
                    }
                } else if (partial_dem_r1 + partial_dem_r2 <= instance->max_vehicle_capa_ && loading1 - partial_dem_r1 + loading2 - partial_dem_r2 <= instance->max_vehicle_capa_) {
                    double old_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
                    double new_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

                    double change = new_cost - old_cost;
                    if (fabs(change) < 1e-8) change = 0;
                    if (change < 0) {
                        // update
                        upper_cost += change;
                        memcpy(temp_r1, route1, sizeof(int) * node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2; i >= 0; i--) {
                            route1[counter1++] = route2[i];
                        }
                        int counter2 = 0;
                        for (int i = length1 - 1; i >= n1 + 1; i--) {
                            temp_r2[counter2++] = temp_r1[i];
                        }
                        for (int i = n2 + 1; i < length2; i++) {
                            temp_r2[counter2++] = route2[i];
                        }
                        memcpy(route2, temp_r2, sizeof(int) * node_cap);
                        length1 = counter1;
                        length2 = counter2;
                        int new_dem_sum_1 = partial_dem_r1 + partial_dem_r2;
                        int new_dem_sum_2 = loading1 - partial_dem_r1 + loading2 - partial_dem_r2;
                        loading1 = new_dem_sum_1;
                        loading2 = new_dem_sum_2;

                        improved = true;
                        has_moved = true;
                        goto end_loop;
                    }
                }

            }
        }

        end_loop: ;
    }

    return has_moved;
}

void LeaderCbma::two_opt_star_for_sol() {
    if (num_routes == 1) return;

    int* temp_r1 = new int[node_cap];
    int* temp_r2 = new int[node_cap];
    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    auto remove_if_empty = [&](int route) {
        if (demand_sum_per_route[route] == 0 && num_routes > 0) {
            int last = num_routes - 1;
            std::swap(routes[route], routes[last]);
            std::swap(demand_sum_per_route[route], demand_sum_per_route[last]);
            std::swap(num_nodes_per_route[route], num_nodes_per_route[last]);
            memset(routes[last], 0, sizeof(int) * node_cap);
            num_nodes_per_route[last] = 0;
            num_routes--;
        }
    };

    unordered_set<pair<int, int>, PairHash> route_pairs;
    for (int i = 0; i < num_routes - 1; i++) {
        for (int j = i + 1; j < num_routes; j++) {
            route_pairs.insert(make_pair(i, j));
        }
    }

    bool has_moved = false;
    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        has_moved = two_opt_star_for_routes(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2], temp_r1, temp_r2);

        if (has_moved) {
            for (int i = 0; i < r1; i++) {
                route_pairs.emplace(i, r1);
            }
            for (int i = 0; i < r2; i++) {
                route_pairs.emplace(i, r2);
            }

            if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
                int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
                remove_if_empty(empty_r);
                for (int i = 0; i < num_routes; i++) {
                    route_pairs.erase({i, num_routes});
                }
            }

            // Clear unused routes beyond num_routes
            std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
            std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
        }
    }

    delete[] temp_r1;
    delete[] temp_r2;
}

bool LeaderCbma::node_relocation_for_route(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    bool improved = true;

    double original_cost, modified_cost, change = 0.0;
    while (improved) {
        improved = false;

        for (int i = 1; i < length - 1; i++) {
            for (int j = 1; j < length - 1; j++) {
                if (i == j) continue;

                if (i < j) {
                    original_cost = instance->get_distance(route[i - 1], route[i]) +
                                    instance->get_distance(route[i], route[i + 1]) +
                                    instance->get_distance(route[j], route[j + 1]);
                    modified_cost = instance->get_distance(route[i - 1], route[i + 1]) +
                                    instance->get_distance(route[j], route[i]) +
                                    instance->get_distance(route[i], route[j + 1]);
                } else {
                    original_cost = instance->get_distance(route[i - 1], route[i]) +
                                    instance->get_distance(route[i], route[i + 1]) +
                                    instance->get_distance(route[j - 1], route[j]);
                    modified_cost = instance->get_distance(route[j - 1], route[i]) +
                                    instance->get_distance(route[i], route[j]) +
                                    instance->get_distance(route[i - 1], route[i + 1]);
                }

                change = modified_cost - original_cost;
                if (fabs(change) < 1e-8) change = 0;
                if (change < 0) {
                    // update
                    upper_cost += change;
                    moveItoJ(route, i, j);
                    improved = true;
                    has_moved = true;
                    goto end_loop;
                }
            }
        }

        end_loop: ;
    }

    return has_moved;
}

void LeaderCbma::node_relocation_for_sol() {
    for (int i = 0; i < num_routes; i++) {
        node_relocation_for_route(routes[i], num_nodes_per_route[i]);
    }
}

