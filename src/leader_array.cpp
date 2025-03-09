//
// Created by Yinghao Qin on 24/02/2025.
//
#include "leader_array.hpp"

LeaderArray::LeaderArray(int seed_val, Case *instance, Preprocessor *preprocessor) : instance(instance), preprocessor(preprocessor) {
    this->random_engine = std::default_random_engine(seed_val);
    this->uniform_int_dis = std::uniform_int_distribution<int>(0, 5); // 6 moves
    this->intra_route_move_dis = std::uniform_int_distribution<int>(0, 2); // 3 intra-route moves
    this->inter_route_move_dis = std::uniform_int_distribution<int>(0, 3); // 4 inter-route moves
    this->inter_route_move_r2_empty_dis = std::uniform_int_distribution<int>(0, 2); // 3 inter-route moves when route 2 is empty

    this->max_search_depth = 10;
    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;
    this->num_routes = 0;
    this->upper_cost = 0;
    this->history_cost = 0;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_routes = 0;
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);
}

LeaderArray::~LeaderArray() {
    for (int i = 0; i < route_cap; ++i) {
        delete[] routes[i];
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;
}

void LeaderArray::run(Individual* ind) {

}

void LeaderArray::neighbour_explore(const double& history_val) {
    history_cost = history_val;

    uniform_int_distribution<int> route1_dis = std::uniform_int_distribution<int>(0, num_routes - 1); // all possible non-empty routes
    uniform_int_distribution<int> route2_dis = std::uniform_int_distribution<int>(0, num_routes);     // possibly involve empty route

    int r1 = route1_dis(random_engine);
    int r2 = route2_dis(random_engine);
    if (r1 == r2) {
        // intra-route move
        // M1, M4, M7
        switch (intra_route_move_dis(random_engine)) {
            case 0:
                perform_intra_move([this](int* route, int length) { return move1_intra(route, length); },
                                   routes[r1], num_nodes_per_route[r1]);
                return;
            case 1:
                perform_intra_move([this](int* route, int length) { return move4_intra(route, length); },
                                   routes[r1], num_nodes_per_route[r1]);
                return;
            case 2:
                perform_intra_move([this](int* route, int length) { return move7_intra(route, length); },
                                   routes[r1], num_nodes_per_route[r1]);
                return;
        }
    } else {
        // inter-route move

        if (num_nodes_per_route[r2] != 0) {
            // if the route 2 is not empty, we can do the normal inter-route move
            switch (inter_route_move_dis(random_engine)) {
                case 0:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move1_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
                case 1:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move4_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
                case 2:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move8_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
                case 3:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move9_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
            }
        } else {
            // if the route 2 is empty, then the inter-route move involves the empty route
            // M1, M8, M9
            switch (inter_route_move_r2_empty_dis(random_engine)) {
                case 0:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move1_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
                case 1:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move8_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
                case 2:
                    perform_inter_move([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move9_inter(route1, route2, length1, length2, loading1, loading2);
                    }, routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
                    return;
            }
        }
    }
}

void LeaderArray::load_individual(Individual* ind) {
    memset(this->num_nodes_per_route, 0, sizeof(int) * this->route_cap);
    memset(this->demand_sum_per_route, 0, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->route_cap; ++i) {
        memset(this->routes[i], 0, sizeof(int) * this->node_cap);
    }

    // The individual loaded should have several consecutive routes from index 0 to num_routes - 1
    this->upper_cost = ind->upper_cost.penalised_cost;
    this->num_routes = ind->upper_cost.nb_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->num_nodes_per_route[i] = static_cast<int>(ind->chromR[i].size()) + 2;
        this->demand_sum_per_route[i] = instance->calculate_demand_sum(ind->chromR[i]);

        memcpy(&this->routes[i][1], ind->chromR[i].data(),ind->chromR[i].size() * sizeof(int));
    }
}

void LeaderArray::export_individual(Individual* ind) const {
    ind->upper_cost.penalised_cost = this->upper_cost;
    ind->upper_cost.distance = this->upper_cost;
    ind->upper_cost.nb_routes = this->num_routes;

    int index = 0;
    for (int i = 0; i < this->route_cap; ++i) {
        ind->chromR[i].clear();
        ind->chromR[i].shrink_to_fit();
        for (int j = 1; j < this->num_nodes_per_route[i] - 1; ++j) {
            ind->chromR[i].push_back(this->routes[i][j]);
            ind->chromT[index++] = this->routes[i][j];
        }
    }


//    ind->evaluate_upper_cost();
}

void LeaderArray::clean_empty_routes(int r1, int r2) {
    auto remove_if_empty = [&](int route) {
        if (demand_sum_per_route[route] == 0 && num_routes > 0) {
            int last = num_routes - 1;
            std::swap(routes[route], routes[last]);
            std::swap(demand_sum_per_route[route], demand_sum_per_route[last]);
            std::swap(num_nodes_per_route[route], num_nodes_per_route[last]);
            num_routes--;
        }
    };

    remove_if_empty(r1);
    remove_if_empty(r2);

    // Clear unused routes beyond num_routes
    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);
}

bool LeaderArray::perform_intra_move(const std::function<bool(int *, int)>& move_func, int* route, int length) const {
    bool is_moved = false;
    int search_depth = 0;

    while (!is_moved && search_depth < max_search_depth) {
        is_moved = move_func(route, length);
        search_depth++;
    }

    return is_moved;
}

bool LeaderArray::perform_inter_move(const std::function<bool(int *, int *, int &, int &, int &, int &)>& move_func,
                                     int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (num_routes == 1) return false;

    bool is_moved = false;
    int search_depth = 0;

    while (!is_moved && search_depth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int r1 = dist(random_engine);
        bool is_diff_route = false;
        int r2;
        while (!is_diff_route) {
            r2 = dist(random_engine);
            if (r1 != r2) {
                is_diff_route = true;
            }
        }

        is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],demand_sum_per_route[r1], demand_sum_per_route[r2]);

        clean_empty_routes(r1, r2);

        search_depth++;
    }

    return is_moved;
}

bool LeaderArray::is_accepted(const double &change) const {
    return upper_cost + change < history_cost || change <= -MY_EPSILON;
}

bool LeaderArray::move1_intra(int* route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 2);
    int i = dist(random_engine);

    double original_cost, modified_cost;
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

        double change = modified_cost - original_cost;
        if (is_accepted(change)) {
            moveItoJ(route, i, j);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderArray::move1_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (length1 < 3) return false;

    bool has_moved = false;
    double original_cost, modified_cost, change;

    // Case when length2 == 0 (route2 is empty)
    if (length2 == 0) {
        std::uniform_int_distribution<int> dist_cus(1, length1 - 2);
        int i = dist_cus(random_engine);
        int x = route1[i];

        original_cost = instance->get_distance(route1[i - 1], x) + instance->get_distance(x, route1[i + 1]);
        modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) + instance->get_distance(route2[0], x) + instance->get_distance(x, route2[1]);

        change = modified_cost - original_cost;

        if (is_accepted(change)) {
            for (int p = i; p < length1 - 1; p++) {
                route1[p] = route1[p + 1];
            }
            length1--;
            loading1 -= instance->get_customer_demand_(x);

            route2[1] = x;
            length2 = 3;
            loading2 += instance->get_customer_demand_(x);
            upper_cost += change;

            return true;
        }
    }


    // Case when length2 > 0, i.e., length2 >= 3, i.e., route2 is not empty
    vector<int> r1_possible_idx;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            r1_possible_idx.push_back(i);
        }
    }
    if (r1_possible_idx.empty()) return false;
    std::uniform_int_distribution<int> dist(0, static_cast<int>(r1_possible_idx.size()) - 1);
    int i = r1_possible_idx[dist(random_engine)];


    for (int j = 0; j < length2 - 1; ++j) {
        original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) + instance->get_distance(route2[j], route2[j + 1]);
        modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) + instance->get_distance(route2[j], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

        change = modified_cost - original_cost;
        if (is_accepted(change)) {
            int x = route1[i];
            for (int p = i; p < length1 - 1; p++) {
                route1[p] = route1[p + 1];
            }
            length1--;
            loading1 -= instance->get_customer_demand_(x);
            for (int q = length2; q > j + 1; q--) {
                route2[q] = route2[q - 1];
            }
            route2[j + 1] = x;
            length2++;
            loading2 += instance->get_customer_demand_(x);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderArray::move4_intra(int* route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    int i = distI(random_engine);
    double original_cost, modified_cost, change;
    for (int j = i + 1; j < length - 1; ++j) {
        if (j == i + 1) {
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);
            change = modified_cost - original_cost;
        } else {
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[i], route[i + 1])
                            + instance->get_distance(route[j - 1], route[j]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[j], route[i + 1])
                            + instance->get_distance(route[j - 1], route[i]) + instance->get_distance(route[i], route[j + 1]);

            change = modified_cost - original_cost;
        }

        if (is_accepted(change)) {
            swap(route[i], route[j]);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderArray::move4_inter(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 2);
    int i = distI(random_engine);
    double original_cost, modified_cost, change;
    for (int j = 1; j < length2 - 1; ++j) {
        int demand_I = instance->get_customer_demand_(route1[i]);
        int demand_J = instance->get_customer_demand_(route2[j]);
        if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                            instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                            instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted(change)) {
                swap(route1[i], route2[j]);
                loading1 = loading1 - demand_I + demand_J;
                loading2 = loading2 - demand_J + demand_I;
                upper_cost += change;

                has_moved = true;
                break;
            }

        }
    }

    return has_moved;
}

bool LeaderArray::move7_intra(int* route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    int i = distI(random_engine);

    double original_cost, modified_cost, change;
    for (int j = i + 1; j < length - 1; ++j) {
        // Calculate the cost difference between the old route and the new route obtained by swapping arcs
        original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
        modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

        change = modified_cost - original_cost; // negative represents the cost reduction
        if (is_accepted(change)) {
            // update current solution
            reverse(route + i, route + j + 1);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderArray::move8_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (length1 < 3) return false;

    int* temp_r1 = new int[node_cap];
    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;
    double original_cost, modified_cost, change;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    // Case when length2 == 0 (route2 is empty)
    if (length2 == 0) {
        if (n1 == 0 || n1 == length1 - 2) return false;

        int n2 = 0;
        int partial_dem_r2 = 0;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

        change = modified_cost - original_cost;

        if (is_accepted(change)) {
            length2 = 2; // two depots

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

            delete[] temp_r1;

            return true;
        }

    }


    // Case when length2 > 0, i.e., length2 >= 3, i.e., route2 is not empty
    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if (n1 == 0 && n2 == 0) continue; // n1 == 0 && n2 == 0, two routes no need to swap

        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ || partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

        change = modified_cost - original_cost;
        if (is_accepted(change)) {
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

            has_moved = true;
            break;
        }
    }

    delete[] temp_r1;

    return has_moved;
}

bool LeaderArray::move9_inter(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (length1 < 3) return false;

    int* temp_r1 = new int[node_cap];
    int* temp_r2 = new int[node_cap];
    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;
    double original_cost, modified_cost, change;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    // Case when length2 == 0 (route2 is empty)
    if (length2 == 0) {
        if (n1 == 0 || n1 == length1 - 2) return false;

        int n2 = 0;
        int partial_dem_r2 = 0;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        change = modified_cost - original_cost;

        if (is_accepted(change)) {
            length2 = 2; // two depots

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


            delete[] temp_r1;
            delete[] temp_r2;
            return true;
        }
    }


    // Case when length2 > 0, i.e., length2 >= 3, i.e., route2 is not empty
    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ || loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        change = modified_cost - original_cost;
        if (is_accepted(change)) {
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

            has_moved = true;
            break;
        }
    }

    delete[] temp_r1;
    delete[] temp_r2;

    return has_moved;
}

bool LeaderArray::two_opt_for_single_route(int* route, int length) {
    if (length < 5) return false;

    bool isAccept = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    int i = distI(random_engine);
    std::uniform_int_distribution<int> distJ(i + 1, length - 2);
    int j = distJ(random_engine);

    // Calculate the cost difference between the old route and the new route obtained by swapping arcs
    double original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
    double modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

    double change = modified_cost - original_cost; // negative represents the cost reduction
    if (is_accepted(change)) {
        // update current solution
        reverse(route + i, route + j + 1);
        upper_cost += change;

        isAccept = true;
    }

    return isAccept;
}

bool LeaderArray::two_opt_intra_for_individual() {
    bool isMoved = false;
    int searchDepth = 0;

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int random_route_idx = dist(random_engine);

        isMoved = two_opt_for_single_route(routes[random_route_idx], num_nodes_per_route[random_route_idx]);

        searchDepth++;
    }

    return isMoved;
}

// TODO: 它可以被拆成两个算子
bool LeaderArray::two_opt_star_between_two_routes(int *route1, int *route2, int &length1, int &length2, int &loading1,
                                                  int &loading2, int *temp_r1, int *temp_r2 ) {

    if (length1 < 3 || length2 < 3) return false;

    bool isAccept = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 <= instance->max_vehicle_capa_ && partial_dem_r2 + loading1 - partial_dem_r1 <= instance->max_vehicle_capa_) {
            double old_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
            double new_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

            double change = new_cost - old_cost;
            if (is_accepted(change)) {
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

                isAccept = true;
                break;
            }

        } else if (partial_dem_r1 + partial_dem_r2 <= instance->max_vehicle_capa_ && loading1 - partial_dem_r1 + loading2 - partial_dem_r2 <= instance->max_vehicle_capa_) {
            double old_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
            double new_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

            double change = new_cost - old_cost;
            if (is_accepted(change)) {
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

                isAccept = true;
                break;
            }
        }

    }

    return isAccept;
}

bool LeaderArray::two_opt_inter_for_individual() {
    if (num_routes == 1) return false;

    bool isMoved = false;
    int searchDepth = 0;

    int* temp_r1 = new int[node_cap];
    int* temp_r2 = new int[node_cap];
    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int r1 = dist(random_engine);
        bool isDiffRoute = false;
        int r2;
        while (!isDiffRoute) {
            r2 = dist(random_engine);
            if (r1 != r2) {
                isDiffRoute = true;
            }
        }

        isMoved = two_opt_star_between_two_routes(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2], temp_r1, temp_r2);

        // remove empty routes
        if (demand_sum_per_route[r1] == 0) {
            int* tmp = routes[r1];
            routes[r1] = routes[num_routes - 1];
            routes[num_routes - 1] = tmp;
            demand_sum_per_route[r1] = demand_sum_per_route[num_routes - 1];
            num_nodes_per_route[r1] = num_nodes_per_route[num_routes - 1];
            num_routes--;
        }
        if (demand_sum_per_route[r2] == 0) {
            int* tmp = routes[r2];
            routes[r2] = routes[num_routes - 1];
            routes[num_routes - 1] = tmp;
            demand_sum_per_route[r2] = demand_sum_per_route[num_routes - 1];
            num_nodes_per_route[r2] = num_nodes_per_route[num_routes - 1];
            num_routes--;
        }

        // update the variable "num_routes" and "route_cap" to remove the empty route
        for (size_t i = num_routes; i < route_cap; ++i) {
            num_nodes_per_route[i] = 0;
            demand_sum_per_route[i] = 0;
        }

        searchDepth++;
    }

    delete[] temp_r1;
    delete[] temp_r2;
    return isMoved;
}

bool LeaderArray::node_relocation_for_single_route(int *route, int length) {
    if (length <= 4) return false;

    bool isAccept = false;

    std::uniform_int_distribution<int> dist(1, length - 2);
    int i = dist(random_engine);
    bool isDiffNode = false;
    int j;
    while (!isDiffNode) {
        j = dist(random_engine);
        if (i != j) {
            isDiffNode = true;
        }
    }

    double original_cost, modified_cost;
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

    double change = modified_cost - original_cost;
    if (is_accepted(change)) {
        moveItoJ(route, i, j);
        upper_cost += change;

        isAccept = true;
    }

    return isAccept;
}

bool LeaderArray::node_relocation_intra_for_individual() {
    bool isMoved = false;
    int searchDepth = 0;

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int random_route_idx = dist(random_engine);

        isMoved = node_relocation_for_single_route(routes[random_route_idx], num_nodes_per_route[random_route_idx]);

        searchDepth++;
    }

    return isMoved;
}

void LeaderArray::moveItoJ(int* route, int a, int b) {
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

bool LeaderArray::node_relocation_between_two_routes(int *route1, int *route2, int &length1, int &length2,
                                                     int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool isAccept = false;

    vector<int> possible_r1_idx;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            possible_r1_idx.push_back(i);
        }
    }
    if (possible_r1_idx.empty()) return false;
    std::uniform_int_distribution<int> dist(0, static_cast<int>(possible_r1_idx.size()) - 1);
    int i = possible_r1_idx[dist(random_engine)];

    // 我们还是想希望有更多的move被接受，所以此处还是使用for loop去遍历更多可接受的move
    // TODO: 之后可以把这个for loop去掉看看会发生什么
    for (int j = 0; j < length2 - 1; ++j) {
        double old_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) + instance->get_distance(route2[j], route2[j + 1]);
        double new_cost = instance->get_distance(route1[i - 1], route1[i + 1]) + instance->get_distance(route2[j], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

        double change = new_cost - old_cost;
        if (is_accepted(change)) {
            int x = route1[i];
            for (int p = i; p < length1 - 1; p++) {
                route1[p] = route1[p + 1];
            }
            length1--;
            loading1 -= instance->get_customer_demand_(x);
            for (int q = length2; q > j + 1; q--) {
                route2[q] = route2[q - 1];
            }
            route2[j + 1] = x;
            length2++;
            loading2 += instance->get_customer_demand_(x);
            upper_cost += change;

            isAccept = true;
            break;
        }
    }

    return isAccept;
}

bool LeaderArray::node_relocation_inter_for_individual() {
    if (num_routes == 1) return false;

    bool isMoved = false;
    int searchDepth = 0;

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int r1 = dist(random_engine);
        bool isDiffRoute = false;
        int r2;
        while (!isDiffRoute) {
            r2 = dist(random_engine);
            if (r1 != r2) {
                isDiffRoute = true;
            }
        }

        isMoved = node_relocation_between_two_routes(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                                     demand_sum_per_route[r1], demand_sum_per_route[r2]);

        searchDepth++;

        // remove empty routes
        if (demand_sum_per_route[r1] == 0) {
            int* tmp = routes[r1];
            routes[r1] = routes[num_routes - 1];
            routes[num_routes - 1] = tmp;
            demand_sum_per_route[r1] = demand_sum_per_route[num_routes - 1];
            num_nodes_per_route[r1] = num_nodes_per_route[num_routes - 1];
            num_routes--;
        }
        if (demand_sum_per_route[r2] == 0) {
            int* tmp = routes[r2];
            routes[r2] = routes[num_routes - 1];
            routes[num_routes - 1] = tmp;
            demand_sum_per_route[r2] = demand_sum_per_route[num_routes - 1];
            num_nodes_per_route[r2] = num_nodes_per_route[num_routes - 1];
            num_routes--;
        }

        // update the variable "num_routes" and "route_cap" to remove the empty route
        for (size_t i = num_routes; i < route_cap; ++i) {
            num_nodes_per_route[i] = 0;
            demand_sum_per_route[i] = 0;
        }
    }

    return isMoved;
}

bool LeaderArray::node_exchange_for_single_route(int *route, int length) {
    if (length < 6) return false;

    bool isAccept = false;

    std::uniform_int_distribution<int> distI(1, length - 4);
    int i = distI(random_engine);
    double original_cost, modified_cost;
    // TODO: 考虑去掉这个for loop
    for (int j = i + 2; j < length - 1; ++j) {
        original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[i], route[i + 1])
                        + instance->get_distance(route[j - 1], route[j]) + instance->get_distance(route[j], route[j + 1]);
        modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[j], route[i + 1])
                        + instance->get_distance(route[j - 1], route[i]) + instance->get_distance(route[i], route[j + 1]);

        double change = modified_cost - original_cost;
        if (is_accepted(change)) {
            swap(route[i], route[j]);
            upper_cost += change;

            isAccept = true;
            break;
        }

    }

    return isAccept;
}

bool LeaderArray::node_exchange_intra_for_individual() {
    bool isMoved = false;
    int searchDepth = 0;

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int random_route_idx = dist(random_engine);

        isMoved = node_exchange_for_single_route(routes[random_route_idx], num_nodes_per_route[random_route_idx]);

        searchDepth++;
    }

    return isMoved;
}


bool LeaderArray::node_exchange_between_two_routes(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool isAccept = false;

    std::uniform_int_distribution<int> distI(1, length1 - 2);
    int i = distI(random_engine);
    // TODO: 考虑去掉这个for loop
    for (int j = 1; j < length2 - 1; ++j) {
        int demand_I = instance->get_customer_demand_(route1[i]);
        int demand_J = instance->get_customer_demand_(route2[j]);
        if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
            double original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                                   instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
            double modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                                   instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

            double change = modified_cost - original_cost;
            if (is_accepted(change)) {
                swap(route1[i], route2[j]);
                loading1 = loading1 - demand_I + demand_J;
                loading2 = loading2 - demand_J + demand_I;
                upper_cost += change;

                isAccept = true;
                break;
            }

        }
    }

    return isAccept;
}

bool LeaderArray::node_exchange_inter_for_individual() {
    if (num_routes == 1) return false;

    bool isMoved = false;
    int searchDepth = 0;

    while (!isMoved && searchDepth < max_search_depth) {
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        int r1 = dist(random_engine);
        bool isDiffRoute = false;
        int r2;
        while (!isDiffRoute) {
            r2 = dist(random_engine);
            if (r1 != r2) {
                isDiffRoute = true;
            }
        }

        isMoved = node_exchange_between_two_routes(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                                                   demand_sum_per_route[r1], demand_sum_per_route[r2]);

        searchDepth++;
    }

    return isMoved;
}

std::ostream& operator<<(std::ostream& os, const LeaderArray& leader) {
    os << "Route Capacity: " << leader.route_cap << "\n";
    os << "Node Capacity: " << leader.node_cap << "\n";
    os << "Number of Routes: " << leader.num_routes << "\n";
    os << "Upper Cost: " << leader.upper_cost << "\n";

    os << "Number of Nodes per route (upper): ";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << leader.num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << leader.demand_sum_per_route[i] << " ";
    }
    os << "\n";

    os << "Upper Routes: \n";
    for (int i = 0; i < leader.route_cap; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < leader.node_cap; ++j) {
            os << leader.routes[i][j] << " ";
        }
        os << "\n";
    }

    return os;
}