//
// Created by Yinghao Qin on 22/04/2025.
//

#include "leader_cbma.hpp"

namespace {
    thread_local std::mt19937 thread_rng(std::random_device{}());
}

LeaderCbma::LeaderCbma(Case *instance, Preprocessor *preprocessor)
        : instance(instance),
          preprocessor(preprocessor),
          uniform_int_dis(0, 7) {

    this->partial_sol = nullptr;
    this->route_cap = preprocessor->route_cap_;
    this->node_cap  = preprocessor->node_cap_;
    this->moves_count = 15;
    this->move_indices.resize(moves_count);
    std::iota(move_indices.begin(), move_indices.end(), 0);
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

    prepare_temp_buffers(node_cap);
    k_active_moves = 0;
    k_active_moves_dist = uniform_int_distribution<int>(1, moves_count);
    active_moves.reserve(moves_count);
    max_chain_length = 512;
    local_search_dist = uniform_int_distribution<int>(0, 14);
    perturbation_dist = uniform_int_distribution<int>(0, 15);
}

LeaderCbma::~LeaderCbma() {
    for (int i = 0; i < route_cap; ++i) {
        delete[] routes[i];
    }
    delete[] routes;
    delete[] num_nodes_per_route;
    delete[] demand_sum_per_route;

    delete[] temp_r1;
    delete[] temp_r2;
}

int LeaderCbma::get_luby(int j) const {
    int k = 1;
    while ((1 << k) - 1 < j) ++k;
    int val = (1 << (k - 1));
    if (val >= max_chain_length) return max_chain_length;
    if ((1 << k) - 1 == j) return val;
    return get_luby(j - val + 1);
}

void LeaderCbma::prepare_temp_buffers(int required_size) const {
    if (temp_buffer_size >= required_size) return;

    delete[] temp_r1;
    delete[] temp_r2;

    temp_r1 = new int[required_size];
    temp_r2 = new int[required_size];
    temp_buffer_size = required_size;
}

void LeaderCbma::run(Individual *ind) {
    load_individual(ind);

    two_opt_for_sol();
    two_opt_star_for_sol();
    node_relocation_for_sol();

    export_individual(ind);
}

void LeaderCbma::run_plus(Individual *ind) {
    load_individual(ind);

    int loop_count = 0;
    bool improvement_found = true;
    while (improvement_found) {
        shuffle(move_indices.begin(), move_indices.end(), thread_rng);

        bool any_move_successful = false;
        for (int i = 0; i < moves_count; ++i) {
            int move = move_indices[i];

            bool has_moved = false;
            switch (move) {
                case 0:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move1_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 1:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move1_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 2:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move2_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 3:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move2_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 4:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move3_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 5:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move3_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 6:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move4_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 7:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move4_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 8:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move5_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 9:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move5_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 10:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move6_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 11:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move6_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 12:
                    has_moved = perform_intra_move_impro([this](int* route, int length) {
                        return move7_intra_impro(route, length, is_accepted_impro);
                    });
                    break;
                case 13:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move8_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
                case 14:
                    has_moved = perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                        return move9_inter_impro(route1, route2, length1, length2, loading1, loading2, is_accepted_impro);
                    });
                    break;
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

bool LeaderCbma::local_search_move(PartialSolution *partial_ind, const double& temperature) {
    partial_sol = partial_ind;

    AcceptanceFunc sa_accept = [temperature](double change) {
        return LeaderCbma::is_accepted_neigh(change, temperature);
    };

    using IntraFunc = std::function<bool(int*, int)>;
    using InterFunc = std::function<bool(int*, int*, int&, int&, int&, int&)>;

    std::vector<IntraFunc> intra_moves = {
            [this, &sa_accept](int* r, int l) { return move1_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move2_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move3_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move4_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move5_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move6_intra_impro(r, l, sa_accept); },
            [this, &sa_accept](int* r, int l) { return move7_intra_impro(r, l, sa_accept); },
    };

    std::vector<InterFunc> inter_moves = {
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move1_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move2_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move3_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move4_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move5_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move6_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move8_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
            [this, &sa_accept](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                return move9_inter_impro(r1, r2, l1, l2, ld1, ld2, sa_accept);
            },
    };

    int choice = local_search_dist(thread_rng);
    int num_intra = static_cast<int>(intra_moves.size());

    if (choice < num_intra) {
        return perform_intra_move_neigh(
                [&](int* r, int l) { return intra_moves[choice](r, l); });
    } else {
        int inter_idx = choice - num_intra;
        return perform_inter_move_neigh(
                [&](int* r1, int* r2, int& l1, int& l2, int& ld1, int& ld2) {
                    return inter_moves[inter_idx](r1, r2, l1, l2, ld1, ld2);
                });
    }
}

void LeaderCbma::strong_perturbation(int strength) {
    for (int i = 0; i < strength; ++i) {

        bool has_moved = false;
        switch (perturbation_dist(thread_rng)) {
            case 0:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move1_intra_pert(route, length);
                });
                break;
            case 1:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move1_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 2:
                has_moved = perform_inter_move_with_empty_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move1_inter_with_empty_route_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 3:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move2_intra_pert(route, length);
                });
                break;
            case 4:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move2_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 5:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move3_intra_pert(route, length);
                });
                break;
            case 6:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move3_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 7:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move4_intra_pert(route, length);
                });
                break;
            case 8:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move4_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 9:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move5_intra_pert(route, length);
                });
                break;
            case 10:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move5_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 11:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move6_intra_pert(route, length);
                });
                break;
            case 12:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move6_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 13:
                has_moved = perform_intra_move_pert([this](int* route, int length) {
                    return move7_intra_pert(route, length);
                });
                break;
            case 14:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move8_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            case 15:
                has_moved = perform_inter_move_pert([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2) {
                    return move9_inter_pert(route1, route2, length1, length2, loading1, loading2);
                });
                break;
            default:
                break;
        }
    }
}

void LeaderCbma::load_individual(Individual *ind) {
    clean();

    this->upper_cost = ind->upper_cost;
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
                                          int &loading2) {
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

    route_pairs.clear();
    for (int i = 0; i < num_routes - 1; i++) {
        for (int j = i + 1; j < num_routes; j++) {
            route_pairs.insert(make_pair(i, j));
        }
    }

    bool has_moved = false;
    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        has_moved = two_opt_star_for_routes(routes[r1], routes[r2], num_nodes_per_route[r1],
                                            num_nodes_per_route[r2], demand_sum_per_route[r1],
                                            demand_sum_per_route[r2]);

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


bool LeaderCbma::is_accepted_impro(const double &change) {
    return change < - 1.0e-8;
}

//bool LeaderCbma::is_accepted_neigh(const double &change) {
//    return change < - 1.0e-8;
//}

double LeaderCbma::get_temperature(int current_iter, double T0, double alpha) {
    return T0 * std::pow(alpha, current_iter);
}

bool LeaderCbma::is_accepted_neigh(const double& change, const double& temperature) {
    if (change < -1.0e-8) return true;

    std::uniform_real_distribution<double> dis(0.0, 1.0);
    double probability = std::exp(-change / temperature);
    return dis(thread_rng) < probability;
}


bool LeaderCbma::perform_intra_move_neigh(const std::function<bool(int *, int)> &move_func) const {
    if (num_routes < 1) return false;

    bool is_moved = false;

    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    int random_route_idx = dist(thread_rng);

    is_moved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx]);
    if (is_moved) {
        partial_sol->set_intra_route(random_route_idx, routes[random_route_idx],num_nodes_per_route[random_route_idx]);
        partial_sol->num_routes = num_routes;
    }

    return is_moved;
}

bool LeaderCbma::perform_inter_move_neigh(
        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (routes == nullptr || num_routes <= 1) return false;

    bool is_moved = false;

    int r1, r2;
    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    r1 = dist(thread_rng);
    do {
        r2 = dist(thread_rng);
    } while (r1 == r2);

    is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],demand_sum_per_route[r1], demand_sum_per_route[r2]);

    if (is_moved) {
        // clean the route beyond the length
        partial_sol->set_inter_route(r1, routes[r1], num_nodes_per_route[r1], demand_sum_per_route[r1] == 0,
                                     r2, routes[r2], num_nodes_per_route[r2], demand_sum_per_route[r2] == 0);
        clean_empty_routes(r1, r2);
        partial_sol->num_routes = num_routes;
    }

    return is_moved;
}

bool LeaderCbma::perform_intra_move_impro(const std::function<bool(int*, int)>& move_func) const {
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

bool LeaderCbma::perform_inter_move_impro(const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
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

//bool LeaderCbma::perform_inter_move_with_empty_impro(
//        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
//    if (num_routes < 1) return false;
//
//    bool is_successful = false;
//    for (int r1 = 0; r1 < num_routes; ++r1) {
//        int r2 = num_routes; // empty route
//
//        bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],demand_sum_per_route[r1], demand_sum_per_route[r2]);
//
//        if (is_moved) {
//            num_routes++;
//        }
//        is_successful = is_moved || is_successful;
//    }
//
//    return is_successful;
//}

bool LeaderCbma::move1_intra_impro(int* route, int length, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move1_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move2_intra_impro(int *route, int length, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move4_intra_impro(int* route, int length, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move2_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move3_intra_impro(int *route, int length, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move3_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move4_inter_impro(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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
                if (accept_func(change)) {
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

bool LeaderCbma::move5_intra_impro(int *route, int length, const AcceptanceFunc& accept_func) {
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

            if (accept_func(change)) {
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

bool LeaderCbma::move5_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move6_intra_impro(int *route, int length, const AcceptanceFunc& accept_func) {
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

            if (accept_func(change)) {
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

bool LeaderCbma::move6_inter_impro(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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

            if (accept_func(change)) {
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

bool LeaderCbma::move7_intra_impro(int *route, int length, const AcceptanceFunc& accept_func) {
    if (length < 5) return false;

    bool has_moved = false;

    double original_cost, modified_cost, change;
    for (int i = 1; i < length - 2; ++i) {
        for (int j = i + 1; j < length - 1; ++j) {
            // Calculate the cost difference between the old route and the new route obtained by swapping arcs
            original_cost = instance->get_distance(route[i - 1], route[i]) + instance->get_distance(route[j], route[j + 1]);
            modified_cost = instance->get_distance(route[i - 1], route[j]) + instance->get_distance(route[i], route[j + 1]);

            change = modified_cost - original_cost; // negative represents the cost reduction
            if (accept_func(change)) {
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

bool LeaderCbma::move8_inter_impro(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::move9_inter_impro(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2, const AcceptanceFunc& accept_func) {
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
            if (accept_func(change)) {
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

bool LeaderCbma::perform_intra_move_pert(const std::function<bool(int *, int)> &move_func) const {
    if (num_routes < 1) return false;

    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    int random_route_idx = dist(thread_rng);

    bool is_moved = move_func(routes[random_route_idx], num_nodes_per_route[random_route_idx]);

    return is_moved;
}

bool LeaderCbma::perform_inter_move_pert(
        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (num_routes <= 1) return false;

    int r1, r2;
    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    r1 = dist(thread_rng);
    do {
        r2 = dist(thread_rng);
    } while (r1 == r2);

    bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                              demand_sum_per_route[r1], demand_sum_per_route[r2]);

    if (is_moved) {
        clean_empty_routes(r1, r2);
    }

    return is_moved;
}

bool LeaderCbma::perform_inter_move_with_empty_pert(
        const std::function<bool(int *, int *, int &, int &, int &, int &)> &move_func) {
    if (num_routes < 1) return false;

    int r1, r2;
    std::uniform_int_distribution<int> dist(0, num_routes - 1);
    r1 = dist(thread_rng);
    r2 = num_routes; // empty route

    bool is_moved = move_func(routes[r1], routes[r2], num_nodes_per_route[r1], num_nodes_per_route[r2],
                         demand_sum_per_route[r1], demand_sum_per_route[r2]);

    if (is_moved) {
        num_routes++;
    }

    return is_moved;
}

bool LeaderCbma::move1_intra_pert(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 2);
    int i = dist(thread_rng);
    int j;
    do {
        j = dist(thread_rng);
    } while (i == j);

    double original_cost, modified_cost, change;
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

    moveItoJ(route, i, j);
    upper_cost += change;
    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move1_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 1; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    int j = dist(thread_rng);

    double original_cost, modified_cost, change;
    original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                    instance->get_distance(route1[i], route1[i + 1]) +
                    instance->get_distance(route2[j], route2[j + 1]);
    modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                    instance->get_distance(route2[j], route1[i]) +
                    instance->get_distance(route1[i], route2[j + 1]);
    change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move1_inter_with_empty_route_pert(int *route1, int *route2, int &length1, int &length2, int &loading1,
                                                   int &loading2) {
    if (length1 < 4 || length2 != 0) return false; // make sure it won't generate empty route

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length1 - 2);
    int i = dist(thread_rng);
    int x = route1[i];

    double original_cost = instance->get_distance(route1[i - 1], x) +
                           instance->get_distance(x, route1[i + 1]);
    double modified_cost = instance->get_distance(route1[i - 1], route1[i + 1]) +
                           instance->get_distance(route2[0], x) +
                           instance->get_distance(x, route2[1]);
    double change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move2_intra_pert(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist1(1, length - 3);
    std::uniform_int_distribution<int> dist2(0, length - 2);
    int i = dist1(thread_rng);
    int j;
    do {
        j = dist2(thread_rng);
    } while (j == i - 1 || j == i || j == i + 1);


    double original_cost, modified_cost, change;
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

    int u = route[i];
    int x = route[i + 1];
    if (i < j) {
        for (int k = i; k < j - 1; k++) {
            route[k] = route[k + 2];
        }
        route[j - 1] = u;
        route[j] = x;
    } else if (i > j) {
        for (int k = i + 1; k > j + 2; k--) {
            route[k] = route[k - 2];
        }
        route[j + 1] = u;
        route[j + 2] = x;
    }
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move2_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[ i + 1]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    int j = dist(thread_rng);

    double original_cost, modified_cost, change;
    original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                    instance->get_distance(route1[i + 1], route1[i + 2]) +
                    instance->get_distance(route2[j], route2[j + 1]);
    modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                    instance->get_distance(route2[j], route1[i]) +
                    instance->get_distance(route1[i + 1], route2[j + 1]);
    change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move3_intra_pert(int *route, int length) {
    if (length <= 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> dist(1, length - 3);
    int i = dist(thread_rng);
    std::uniform_int_distribution<int> dist2(0, length - 2);
    int j;
    do {
        j = dist2(thread_rng);
    } while (j == i - 1 || j == i || j == i + 1);

    double original_cost, modified_cost, change;
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

    return has_moved;
}

bool LeaderCbma::move3_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 3) return false;

    bool has_moved = false;

    std::vector<int> candidates;
    for (int i = 1; i < length1 - 2; ++i) {
        if (loading2 + instance->get_customer_demand_(route1[i]) +
            instance->get_customer_demand_(route1[ i + 1]) <= instance->max_vehicle_capa_) {
            candidates.push_back(i);
        }
    }
    if (candidates.empty()) return false;
    std::shuffle(candidates.begin(), candidates.end(), thread_rng);  // Shuffle to pick one randomly
    int i = candidates.front();  // Pick the first after shuffling
    std::uniform_int_distribution<int> dist(0, length2 - 2);
    int j = dist(thread_rng);

    double original_cost, modified_cost, change;
    original_cost = instance->get_distance(route1[i - 1], route1[i]) +
                    instance->get_distance(route1[i + 1], route1[i + 2]) +
                    instance->get_distance(route2[j], route2[j + 1]);
    modified_cost = instance->get_distance(route1[i - 1], route1[i + 2]) +
                    instance->get_distance(route2[j], route1[i + 1]) +
                    instance->get_distance(route1[i], route2[j + 1]);
    change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move4_intra_pert(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 1, length - 2);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
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

    swap(route[i], route[j]);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move4_inter_pert(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 2);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 2);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
    int demand_I = instance->get_customer_demand_(route1[i]);
    int demand_J = instance->get_customer_demand_(route2[j]);
    if (loading1 - demand_I + demand_J <= instance->max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance->max_vehicle_capa_) {
        original_cost =
                instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i], route1[i + 1]) +
                instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
        modified_cost =
                instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 1]) +
                instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i], route2[j + 1]);

        change = modified_cost - original_cost;

        swap(route1[i], route2[j]);
        loading1 = loading1 - demand_I + demand_J;
        loading2 = loading2 - demand_J + demand_I;
        upper_cost += change;

        has_moved = true;
    }

    return has_moved;
}

bool LeaderCbma::move5_intra_pert(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 4);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 2, length - 2);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
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

    return has_moved;
}

bool LeaderCbma::move5_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 3);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 2);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
    int demand_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
    int demand_J = instance->get_customer_demand_(route2[j]);
    if (loading1 - demand_pair + demand_J > instance->max_vehicle_capa_ || loading2 - demand_J + demand_pair > instance->max_vehicle_capa_) {
        return false;
    }

    original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i + 1], route1[i + 2]) +
                    instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j], route2[j + 1]);
    modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j], route1[i + 2]) +
                    instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i + 1], route2[j + 1]);

    change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move6_intra_pert(int *route, int length) {
    if (length < 6) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 5);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 2, length - 3);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
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

    swap(route[i], route[j]);
    swap(route[i + 1], route[j + 1]);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move6_inter_pert(int *route1, int *route2, int length1, int length2, int &loading1, int &loading2) {
    if (length1 < 4 || length2 < 4 || (length1 == 4 && length2 == 4)) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length1 - 3);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(1, length2 - 3);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
    int demand_I_pair = instance->get_customer_demand_(route1[i]) + instance->get_customer_demand_(route1[i + 1]);
    int demand_J_pair = instance->get_customer_demand_(route2[j]) + instance->get_customer_demand_(route2[j + 1]);
    if (loading1 - demand_I_pair + demand_J_pair > instance->max_vehicle_capa_ || loading2 - demand_J_pair + demand_I_pair > instance->max_vehicle_capa_) {
        return false;
    }

    original_cost = instance->get_distance(route1[i - 1], route1[i]) + instance->get_distance(route1[i + 1], route1[i + 2]) +
                    instance->get_distance(route2[j - 1], route2[j]) + instance->get_distance(route2[j + 1], route2[j + 2]);
    modified_cost = instance->get_distance(route1[i - 1], route2[j]) + instance->get_distance(route2[j + 1], route1[i + 2]) +
                    instance->get_distance(route2[j - 1], route1[i]) + instance->get_distance(route1[i + 1], route2[j + 2]);
    change = modified_cost - original_cost;

    swap(route1[i], route2[j]);
    swap(route1[i + 1], route2[j + 1]);
    loading1 = loading1 - demand_I_pair + demand_J_pair;
    loading2 = loading2 - demand_J_pair + demand_I_pair;
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move7_intra_pert(int *route, int length) {
    if (length < 5) return false;

    bool has_moved = false;

    std::uniform_int_distribution<int> distI(1, length - 3);
    int i = distI(thread_rng);
    std::uniform_int_distribution<int> distJ(i + 1, length - 2);
    int j = distJ(thread_rng);

    double original_cost, modified_cost, change;
    original_cost = instance->get_distance(route[i - 1], route[i]) +
                    instance->get_distance(route[j], route[j + 1]);
    modified_cost = instance->get_distance(route[i - 1], route[j]) +
                    instance->get_distance(route[i], route[j + 1]);

    change = modified_cost - original_cost; // negative represents the cost reduction`

    reverse(route + i, route + j + 1);
    upper_cost += change;

    has_moved = true;

    return has_moved;
}

bool LeaderCbma::move8_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);
    memset(temp_r2, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    double original_cost, modified_cost, change;

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == length1 - 2 && n2 == 0) || (n1 == 0 && n2 == length2 - 2)) continue; // the same as the current situation, just skip it.
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + partial_dem_r2 > instance->max_vehicle_capa_ ||
            loading1 - partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) + instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2]) + instance->get_distance(route1[n1 + 1], route2[n2 + 1]);

        change = modified_cost - original_cost;

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

    return has_moved;
}

bool LeaderCbma::move9_inter_pert(int *route1, int *route2, int &length1, int &length2, int &loading1, int &loading2) {
    if (length1 < 3 || length2 < 3) return false;

    memset(temp_r1, 0, sizeof(int) * node_cap);

    bool has_moved = false;

    std::uniform_int_distribution<int> distN1(0, length1 - 2);
    int n1 = distN1(thread_rng);
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    for (int i = 0; i <= n1; i++) {
        partial_dem_r1 += instance->get_customer_demand_(route1[i]);
    }

    double original_cost, modified_cost, change;

    int partial_dem_r2 = 0;
    for (int n2 = 0; n2 < length2 - 1; ++n2) {
        if ((n1 == 0 && n2 == 0) || (n1 == length1 - 2 && n2 == length2 - 2)) continue;
        partial_dem_r2 += instance->get_customer_demand_(route2[n2]);

        if (partial_dem_r1 + loading2 - partial_dem_r2 > instance->max_vehicle_capa_ ||
            partial_dem_r2 + loading1 - partial_dem_r1 > instance->max_vehicle_capa_) continue;

        original_cost = instance->get_distance(route1[n1], route1[n1 + 1]) +
                        instance->get_distance(route2[n2], route2[n2 + 1]);
        modified_cost = instance->get_distance(route1[n1], route2[n2 + 1]) +
                        instance->get_distance(route2[n2], route1[n1 + 1]);

        change = modified_cost - original_cost;
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

    return has_moved;
}

std::ostream& operator<<(std::ostream& os, const LeaderCbma& leader) {
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