//
// Created by Yinghao Qin on 13/04/2025.
//

#include "leader_sga.hpp"

LeaderSga::LeaderSga(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
        : instance(instance),
          preprocessor(preprocessor),
          random_engine(engine),
          uniform_int_dis(0, 6) {

    this->partial_sol = nullptr;
    this->moves_count = 15;
    move_indices.resize(moves_count);
    std::iota(move_indices.begin(), move_indices.end(), 0);
    this->max_search_depth = 25;
    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;
    this->num_routes = 0;
    this->upper_cost = 0.;
    this->border_cost = 0.; // border_cost for example, is the best upper cost so far * 1.1
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_nodes_per_route = new int[route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);

    prepare_temp_buffers(node_cap);
}

LeaderSga::~LeaderSga() {
    for (int i = 0; i < route_cap; ++i) {
        delete[] routes[i];
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;

    delete[] temp_r1;
    delete[] temp_r2;
}

void LeaderSga::local_improve(Individual *ind) {
    load_individual(ind);

    int loop_count = 0;
    bool improvement_found = true;
    while (improvement_found) {
        shuffle(move_indices.begin(), move_indices.end(), random_engine);

        bool any_move_successful = false;
        for (int i = 0; i < moves_count; ++i) {

            bool has_moved = false;
            switch (move_indices[i]) {
                case 0:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move1_intra_impro(route, length);
                    });
                    break;
                case 1:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move1_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 2:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move2_intra_impro(route, length);
                    });
                    break;
                case 3:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move2_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 4:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move3_intra_impro(route, length);
                    });
                    break;
                case 5:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move3_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 6:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move4_intra_impro(route, length);
                    });
                    break;
                case 7:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move4_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 8:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move5_intra_impro(route, length);
                    });
                    break;
                case 9:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move5_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 10:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move6_intra_impro(route, length);
                    });
                    break;
                case 11:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move6_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 12:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move7_intra_impro(route, length);
                    });
                    break;
                case 13:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move8_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
                case 14:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move9_inter_impro(route1, route2, length1, length2, loading1, loading2);
                    });
                    break;
//                case 15:
//                    has_moved = perform_inter_move_with_empty_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
//                        return move1_inter_with_empty_route_impro(route1, route2, length1, length2, loading1, loading2);
//                    });
//                    break;
                default:
                    break;
            }

            if (has_moved) {
                any_move_successful = true;
            }
        }

        improvement_found = any_move_successful;
        loop_count++;
    }


    export_individual(ind);
}

bool LeaderSga::neighbour_explore(const double& border_val, PartialSolution* partial_ind) {
    border_cost = border_val;
    partial_sol = partial_ind;

    bool has_moved = false;
    switch (uniform_int_dis(random_engine)) {
        case 0:
            has_moved = perform_intra_move_neigh([this](int* route, int length) {
                return move1_intra_neigh(route, length);
            });
            break;
        case 1:
            has_moved = perform_intra_move_neigh([this](int* route, int length) {
                return move4_intra_neigh(route, length); });
            break;
        case 2:
            has_moved = perform_intra_move_neigh([this](int* route, int length) {
                return move7_intra_neigh(route, length); });
            break;
        case 3:
            has_moved = perform_inter_move_neigh([this](int* route1, int* route2, int& length1, int& length2,
                    int& loading1, int& loading2){
                return move1_inter_neigh(route1, route2, length1, length2, loading1, loading2);});
            break;
        case 4:
            has_moved = perform_inter_move_neigh([this](int* route1, int* route2, int length1, int length2,
                    int& loading1, int& loading2){
                return move4_inter_neigh(route1, route2, length1, length2, loading1, loading2);});
            break;
        case 5:
            has_moved = perform_inter_move_neigh([this](int* route1, int* route2, int& length1, int& length2,
                    int& loading1, int& loading2) {
                return move8_inter_neigh(route1, route2, length1, length2, loading1, loading2);});
            break;
        case 6:
            has_moved = perform_inter_move_neigh([this](int* route1, int* route2, int& length1, int& length2,
                    int& loading1, int& loading2) {
                return move9_inter_neigh(route1, route2, length1, length2, loading1, loading2);});
            break;
//        case 7:
//            has_moved = perform_inter_move_with_empty_neigh([this](int* route1, int* route2, int& length1,
//                    int& length2, int& loading1, int& loading2) {
//                return move1_inter_with_empty_route_neigh(route1, route2, length1, length2, loading1, loading2);});
//            break;
        default:
            break;
    }

    return has_moved;
}

void LeaderSga::run(Individual *ind) {

}


void LeaderSga::load_individual(Individual* ind) {
    clean();

    this->upper_cost = ind->upper_cost;
    this->num_routes = ind->num_routes;
    memcpy(this->num_nodes_per_route, ind->num_nodes_per_route, sizeof(int) * ind->route_cap);
    memcpy(this->demand_sum_per_route, ind->demand_sum_per_route, sizeof(int) * ind->route_cap);
    for (int i = 0; i < ind->num_routes; ++i) {
        memcpy(this->routes[i], ind->routes[i], sizeof(int) * ind->node_cap);
    }
}

void LeaderSga::export_individual(Individual* ind) const {
    ind->upper_cost = this->upper_cost;
    ind->num_routes = this->num_routes;
    memcpy(ind->num_nodes_per_route, this->num_nodes_per_route, sizeof(int) * this->route_cap);
    memcpy(ind->demand_sum_per_route, this->demand_sum_per_route, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->num_routes; ++i) {
        memcpy(ind->routes[i], this->routes[i], sizeof(int) * this->node_cap);
    }
}

void LeaderSga::clean() {
    this->num_routes = 0;
    this->upper_cost = 0.;

    memset(this->num_nodes_per_route, 0, sizeof(int) * this->route_cap);
    memset(this->demand_sum_per_route, 0, sizeof(int) * this->route_cap);
    for (int i = 0; i < this->route_cap; ++i) {
        memset(this->routes[i], 0, sizeof(int) * this->node_cap);
    }
}

void LeaderSga::clean_empty_routes(int r1, int r2) {
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

void LeaderSga::moveItoJ(int* route, int a, int b) {
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

bool LeaderSga::is_accepted_impro(const double &change) {
    return change < - 1.0e-8;
}

bool LeaderSga::perform_intra_move_impro(const std::function<bool(int*, int)>& move_func) const {
    if (num_routes < 1) return false;

    bool is_successful = false;

    for (int i = 0; i < num_routes; ++i) {
        bool is_improved;
        do {
            is_improved = move_func(routes[i], num_nodes_per_route[i]);
            is_successful = is_improved || is_successful;
        } while (is_improved);
    }

    return is_successful;
}

bool LeaderSga::perform_inter_move_impro(const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (num_routes <= 1) return false;

    bool is_successful = false;

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

    route_pairs.clear();
    for (int i = 0; i < num_routes - 1; i++) {
        for (int j = i + 1; j < num_routes; j++) {
            route_pairs.insert(make_pair(i, j));
        }
    }

    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        bool is_improved;
        do {
            is_improved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2], demand_sum_per_route[r1], demand_sum_per_route[r2]);
            is_successful = is_improved || is_successful;

            if (is_improved) {
                // clean the route beyond the length
                std::fill(routes[r1] + num_nodes_per_route[r1], routes[r1] + node_cap, 0);
                std::fill(routes[r2] + num_nodes_per_route[r2], routes[r2] + node_cap, 0);

                if (demand_sum_per_route[r1] == 0 || demand_sum_per_route[r2] == 0) {
                    int empty_r = demand_sum_per_route[r1] == 0 ? r1 : r2;
                    remove_if_empty(empty_r);
                    for (int i = 0; i < num_routes; i++) {
                        route_pairs.erase({i, num_routes});
                    }

                    if (r1 <= num_routes - 1) {
                        for (int i = 0; i < r1; i++) {
                            route_pairs.emplace(i, r1);
                        }
                    }
                    if (r2 <= num_routes - 1) {
                        for (int i = 0; i < r2; i++) {
                            route_pairs.emplace(i, r2);
                        }
                    }

                    // Clear unused routes beyond num_routes
                    std::fill(num_nodes_per_route + num_routes, num_nodes_per_route + route_cap, 0);
                    std::fill(demand_sum_per_route + num_routes, demand_sum_per_route + route_cap, 0);

                    break;
                }
            }
        } while (is_improved);

    }

    return is_successful;
}

bool LeaderSga::perform_inter_move_with_empty_impro(
        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (num_routes < 1) return false;

    bool is_successful = false;
    for (int r1 = 0; r1 < num_routes; ++r1) {
        int r2 = num_routes; // empty route

        bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],demand_sum_per_route[r1], demand_sum_per_route[r2]);

        if (is_moved) {
            num_routes++;
        }
        is_successful = is_moved || is_successful;
    }

    return is_successful;
}

bool LeaderSga::move1_intra_impro(int* route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 1; ++i) {
        for (int j = 1; j < length - 1; ++j) {
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
            if (is_accepted_impro(change)) {
                moveItoJ(route, i, j);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool LeaderSga::move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) > instance->max_vehicle_capa_) {
            continue;
        }
        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) + instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) + instance->get_distance(route2[j], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
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
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move2_intra_impro(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = 0; j < length - 1; j++) {
            if (j == i - 1 || j == i || j == i + 1) continue;

            if (i < j) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[i + 2]) +
                                instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]) +
                                instance->get_distance(route[i - 1], route[i + 2]);
            }

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                int u = route[i];
                int x = route[i + 1];
                if (i < j) {
                    for (int k = i; k < j - 1; k++) {
                        route[k] = route[k + 2];
                    }
                    route[j - 1] = u;
                    route[j] = x;
                }
                else if (i > j) {
                    for (int k = i + 1; k > j + 2; k--) {
                        route[k] = route[k - 2];
                    }
                    route[j + 1] = u;
                    route[j + 2] = x;
                }
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move4_intra_impro(int* route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            if (j == i + 1) {
                original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[i], route[i + 1]) +
                                instance->get_distance(route[j - 1], route[j]) + instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[j - 1], route[i]) + instance->get_distance(route[i], route[j + 1]);
            }

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                swap(route[i], route[j]);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool LeaderSga::move2_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1]) > instance->max_vehicle_capa_) {
            continue;
        }

        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route1[i]) +
                            instance->get_distance(route1[i + 1], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                int u = route1[i];
                int x = route1[i + 1];
                for (int p = i; p < length1 - 2; ++p) {
                    route1[p] = route1[p + 2];
                }
                length1 -= 2;
                loading1 -= instance->get_customer_demand_(u);
                loading1 -= instance->get_customer_demand_(x);

                for (int q = length2 + 1; q > j + 2; q--) {
                    route2[q] = route2[q - 2];
                }
                route2[j + 1] = u;
                route2[j + 2] = x;
                length2 += 2;
                loading2 += instance->get_customer_demand_(u);
                loading2 += instance->get_customer_demand_(x);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool LeaderSga::move3_intra_impro(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = 0; j < length - 1; j++) {
            if (j == i - 1 || j == i || j == i + 1) continue;

            if (i < j) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[i + 2]) +
                                instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[i], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[j], route[i + 1]) +
                                instance->get_distance(route[i], route[j + 1]) +
                                instance->get_distance(route[i - 1], route[i + 2]);
            }

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                int u = route[i];
                int x = route[i + 1];
                if (i < j) {
                    for (int k = i; k < j - 1; k++) {
                        route[k] = route[k + 2];
                    }
                    route[j - 1] = x;
                    route[j] = u;
                }
                else if (i > j) {
                    for (int k = i + 1; k > j + 2; k--) {
                        route[k] = route[k - 2];
                    }
                    route[j + 1] = x;
                    route[j + 2] = u;
                }
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move3_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1]) > instance->max_vehicle_capa_) {
            continue;
        }

        for (int j = 0; j < length2 - 1; ++j) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                            instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                            instance->get_distance(route2[j], route1[i + 1]) +
                            instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                int u = route1[i];
                int x = route1[i + 1];
                for (int p = i; p < length1 - 2; ++p) {
                    route1[p] = route1[p + 2];
                }
                length1 -= 2;
                loading1 -= instance->get_customer_demand_(u);
                loading1 -= instance->get_customer_demand_(x);

                for (int q = length2 + 1; q > j + 2; q--) {
                    route2[q] = route2[q - 2];
                }
                route2[j + 1] = x;
                route2[j + 2] = u;
                length2 += 2;
                loading2 += instance->get_customer_demand_(u);
                loading2 += instance->get_customer_demand_(x);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move4_inter_impro(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 1; ++i) {
        for (int j = 1; j < length2 - 1; ++j) {
            int demand_I = instance->get_customer_demand_(route1[i]);
            int demand_J = instance->get_customer_demand_(route2[j]);
            if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
                original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                                instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
                modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                                instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

                change = modified_cost - original_cost;
                if (is_accepted_impro(change)) {
                    swap(route1[i], route2[j]);
                    loading1 = loading1 - demand_I + demand_J;
                    loading2 = loading2 - demand_J + demand_I;
                    upper_cost += change;

                    has_moved = true;
                    goto end_loops;
                }
            }
        }
    }
    end_loops:;

    return has_moved;
}

bool LeaderSga::move5_intra_impro(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 3; ++i) {
        for (int j = i + 2; j < length - 1; ++j) {
            if (j == i + 2) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[j]) +
                                instance->get_distance(route[j], route[j + 1]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 1]);
            }
            change = modified_cost - original_cost;

            if (is_accepted_impro(change)) {
                // node pair (route[i], route[i + 1]) will be changed with node route[j]
                if (j == i + 2) {
                    std::swap(route[i], route[j]);
                    std::swap(route[i + 1], route[j]);
                } else {
                    std::swap(route[i], route[j]);
                    int x = route[i + 1];
                    for (int k = i + 1; k < j; ++k) {
                        route[k] = route[k + 1];
                    }
                    route[j] = x;
                }

                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move5_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        for (int j = 1; j < length2 - 1; ++j) {
            int demand_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
            int demand_J = instance->get_customer_demand_(route2[j]);
            if (loading1 - demand_pair + demand_J > instance->max_vehicle_capa_ || loading2 - demand_J + demand_pair > instance->max_vehicle_capa_) {
                continue;
            }

            original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i + 1], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
                // node pair (route1[i], route1[i + 1]) will be changed with node route2[j]
                int u = route1[i];
                int x = route1[i + 1];
                route1[i] = route2[j];
                for (int p = i + 1; p < length1 - 1; ++p) {
                    route1[p] = route1[p + 1];
                }
                length1--;
                loading1 = loading1 - demand_pair + demand_J;

                for (int q = length2; q > j + 1; q--) {
                    route2[q] = route2[q - 1];
                }
                route2[j] = u;
                route2[j + 1] = x;
                length2++;
                loading2 = loading2 - demand_J + demand_pair;

                upper_cost += change;
                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move6_intra_impro(int *route, int length) {
    if (length < 6) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 4; ++i) {
        for (int j = i + 2; j < length - 2; ++j) {
            if (j == i + 2) {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j]) +
                                instance->get_distance(route[j + 1], route[j + 2]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 2]);
            } else {
                original_cost = instance->get_distance(route[i - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[j + 2]);
                modified_cost = instance->get_distance(route[i - 1], route[j]) +
                                instance->get_distance(route[j + 1], route[i + 2]) +
                                instance->get_distance(route[j - 1], route[i]) +
                                instance->get_distance(route[i + 1], route[j + 2]);
            }
            change = modified_cost - original_cost;

            if (is_accepted_impro(change)) {
                swap(route[i], route[j]);
                swap(route[i + 1], route[j + 1]);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move6_inter_impro(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4 || (length1 == 4 && length2 == 4)) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 2; ++i) {
        for (int j = 1; j < length2 - 2; ++j) {
            int demand_I_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
            int demand_J_pair = instance->get_customer_demand_(route2[j]) + instance->get_customer_demand_(route2[j + 1]);
            if (loading1 - demand_I_pair + demand_J_pair > instance->max_vehicle_capa_ || loading2 - demand_J_pair + demand_I_pair > instance->max_vehicle_capa_) {
                continue;
            }

            original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j + 1], route2[j + 2]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j + 1], route1[i + 2]) +
                            instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i + 1], route2[j + 2]);
            change = modified_cost - original_cost;

            if (is_accepted_impro(change)) {
                swap(route1[i], route2[j]);
                swap(route1[i + 1], route2[j + 1]);
                loading1 = loading1 - demand_I_pair + demand_J_pair;
                loading2 = loading2 - demand_J_pair + demand_I_pair;
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move7_intra_impro(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            // Calculate the cost difference between the old route and the new route obtained by swapping arcs
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

            change = modified_cost - original_cost; // negative represents the cost reduction
            if (is_accepted_impro(change)) {
                // update current solution
                reverse(route + i, route + j + 1);
                upper_cost += change;

                has_moved = true;
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    double original_cost, modified_cost, change;


    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int n1 = 0; n1 < length1 - 1; ++n1) {
        partial_dem_r1 += instance->get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0;
        for (int n2 = 0; n2 < length2 - 1; ++n2) {
            if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
            partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ || loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

            original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
            modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
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
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move9_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    double original_cost, modified_cost, change;

    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int n1 = 0; n1 < length1 - 1; ++n1) {
        partial_dem_r1 += instance->get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0;
        for (int n2 = 0; n2 < length2 - 1; ++n2) {
            if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
            partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ || partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

            original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
            modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_impro(change)) {
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
                goto end_loops;
            }
        }
    }

    end_loops:;

    return has_moved;
}

bool LeaderSga::move1_inter_with_empty_route_impro(int *route1, int *route2, int &length1, int &length2, int &loading1,
                                                   int &loading2) {
    if (length1 < 4 || length2 != 0) return false; // make sure it won't generate empty route

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length1 - 1; ++i) {
        int x = route1[i];

        original_cost = instance->get_distance(route1[i - 1], x) + instance->get_distance(x, route1[i + 1]);
        modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) + instance->get_distance(route2[0], x) + instance->get_distance(x, route2[1]);

        change = modified_cost - original_cost;
        if (is_accepted_impro(change)) {
            for (int p = i; p < length1 - 1; p++) {
                route1[p] = route1[p + 1];
            }
            length1--;
            loading1 -= instance->get_customer_demand_(x);

            route2[1] = x;
            length2 = 3;
            loading2 += instance->get_customer_demand_(x);
            upper_cost += change;

            has_moved = true;
        }
    }

    return has_moved;
}

void LeaderSga::prepare_temp_buffers(int required_size) const {
    if (temp_buffer_size >= required_size) return;

    delete[] temp_r1;
    delete[] temp_r2;

    temp_r1 = new int[required_size];
    temp_r2 = new int[required_size];
    temp_buffer_size = required_size;

    temp_candidates.reserve(required_size);
}

bool LeaderSga::is_accepted_neigh(const double &change) const {
    return upper_cost + change < border_cost; // border_cost for example, is the best upper cost so far * 1.1
}

bool LeaderSga::perform_intra_move_neigh(const std::function<bool(int *, int)> &move_func) {
    if (num_routes < 1) return false;

    bool is_moved = false;
    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    int search_depth = 0;

    while (!is_moved && search_depth < max_search_depth) {
        int random_route_idx = dist(random_engine);

        is_moved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx]);
        if (is_moved) {
            partial_sol->set_intra_route(random_route_idx, routes[random_route_idx],num_nodes_per_route[random_route_idx]);
            partial_sol->num_routes = num_routes;
        }

        search_depth++;
    }

    return is_moved;
}

bool LeaderSga::perform_inter_move_neigh(
        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (num_routes <= 1) return false;

    bool is_moved = false;
    int search_depth = 0;

    int r1, r2;
    while (!is_moved && search_depth < max_search_depth) {
        // randomly generate two different route indices
        std::uniform_int_distribution<int> dist(0, num_routes - 1);
        r1 = dist(random_engine);
        do {
            r2 = dist(random_engine);
        } while (r1 == r2);

        is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],demand_sum_per_route[r1], demand_sum_per_route[r2]);

        if (is_moved) {
            // clean the route beyond the length
            partial_sol->set_inter_route(r1, routes[r1], num_nodes_per_route[r1], demand_sum_per_route[r1] == 0,
                                         r2, routes[r2], num_nodes_per_route[r2], demand_sum_per_route[r2] == 0);
            clean_empty_routes(r1, r2);
            partial_sol->num_routes = num_routes;
        }

        search_depth++;
    }

    return is_moved;
}

bool LeaderSga::move1_intra_neigh(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 2);
    int i = dist(random_engine);

    double original_cost, modified_cost, change;
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
        if (is_accepted_neigh(change)) {
            moveItoJ(route, i, j);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderSga::move1_inter_neigh(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    temp_candidates.clear();
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            temp_candidates.push_back(i);
        }
    }
    if (temp_candidates.empty()) return false;
    std::shuffle(temp_candidates.begin(), temp_candidates.end(), random_engine);  // Shuffle to pick one randomly
    int i = temp_candidates.front();  // Pick the first after shuffling

    double original_cost, modified_cost, change;
    for (int j = 0; j < length2 - 1; ++j) {
        original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                        instance->get_distance(route1[i], route1[i + 1]) +
                        instance->get_distance(route2[j], route2[j + 1]);
        modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                        instance->get_distance(route2[j], route1[i]) +
                        instance->get_distance(route1[i], route2[j + 1]);

        change = modified_cost - original_cost;
        if (is_accepted_neigh(change)) {
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

bool LeaderSga::move4_intra_neigh(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    const int i = distI(random_engine);
    double original_cost, modified_cost, change;
    for (int j = i + 1; j < length - 1; ++j) {
        if (j == i + 1) {
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);
        } else {
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[i], route[i + 1])
                            + instance->get_distance(route[j - 1], route[j]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[j], route[i + 1])
                            + instance->get_distance(route[j - 1], route[i]) + instance->get_distance(route[i], route[j + 1]);
        }

        change = modified_cost - original_cost;
        if (is_accepted_neigh(change)) {
            swap(route[i], route[j]);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderSga::move4_inter_neigh(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 2);
    const int i = distI(random_engine);
    double original_cost, modified_cost, change;
    for (int j = 1; j < length2 - 1; ++j) {
        const int demand_I = instance->get_customer_demand_(route1[i]);
        const int demand_J = instance->get_customer_demand_(route2[j]);
        if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
            original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                            instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
            modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                            instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

            change = modified_cost - original_cost;
            if (is_accepted_neigh(change)) {
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

bool LeaderSga::move7_intra_neigh(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    const int i = distI(random_engine);

    double original_cost, modified_cost, change;
    for (int j = i + 1; j < length - 1; ++j) {
        // Calculate the cost difference between the old route and the new route obtained by swapping arcs
        original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
        modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

        change = modified_cost - original_cost; // negative represents the cost reduction
        if (is_accepted_neigh(change)) {
            // update current solution
            reverse(route + i, route + j + 1);
            upper_cost += change;

            has_moved = true;
            break;
        }
    }

    return has_moved;
}

bool LeaderSga::move8_inter_neigh(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    double original_cost, modified_cost, change;

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ || loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        change = modified_cost - original_cost;
        if (is_accepted_neigh(change)) {
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

    return has_moved;
}

bool LeaderSga::move9_inter_neigh(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(random_engine);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    double original_cost, modified_cost, change;

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ || partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) + instance->get_distance(route2[n2], route1[n1 + 1]);

        change = modified_cost - original_cost;
        if (is_accepted_neigh(change)) {
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

    return has_moved;
}

std::ostream& operator<<(std::ostream& os, const LeaderSga& leader) {
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