//
// Created by Yinghao Qin on 19/02/2025.
//

#include "follower.hpp"

Follower::Follower(Case* instance, Preprocessor* preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;

    route_cap = preprocessor->route_cap_;
    node_cap = preprocessor->node_cap_;

    this->num_routes = 0;
    this->lower_routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->lower_routes[i] = new int [node_cap];
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
    }
    this->lower_num_nodes_per_route = new int [route_cap];
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->lower_cost = 0;
}

Follower::~Follower() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->lower_routes[i];
    }
    delete[] this->lower_routes;
    delete[] this->lower_num_nodes_per_route;
}

void Follower::clean() {
    this->num_routes = 0;
    this->lower_cost = 0.;
    for (int i = 0; i < route_cap; ++i) {
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
    }
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
}

void Follower::refine(Individual* ind) {
    load_individual(ind);

    lower_cost = 0.0;
    for (int i = 0; i < num_routes; ++i) {
        lower_cost += insert_station_by_all_enumeration(lower_routes[i], lower_num_nodes_per_route[i]);
    }

    export_individual(ind);
}

void Follower::run(Individual *ind) {
    load_individual(ind);

    for (int i = 0; i < num_routes; ++i) {
        double cost_SE = insert_station_by_simple_enum( lower_routes[i], lower_num_nodes_per_route[i]);

        if (cost_SE == -1) {
            double cost_RE = insert_station_by_remove_enum( lower_routes[i], lower_num_nodes_per_route[i]);
            if (cost_RE == -1) {
                lower_cost += INFEASIBLE;
            } else {
                lower_cost += cost_RE;
            }
        } else {
            lower_cost += cost_SE;
        }
    }

    export_individual(ind);
}

void Follower::run(Solution* sol) {
    load_solution(sol);

    for (int i = 0; i < num_routes; ++i) {
        double cost_SE = insert_station_by_simple_enum( lower_routes[i], lower_num_nodes_per_route[i]);

        if (cost_SE == -1) {
            double cost_RE = insert_station_by_remove_enum( lower_routes[i], lower_num_nodes_per_route[i]);
            if (cost_RE == -1) {
                lower_cost += INFEASIBLE;
            } else {
                lower_cost += cost_RE;
            }
        } else {
            lower_cost += cost_SE;
        }
    }

    export_solution(sol);
}

void Follower::load_solution(const Solution* sol) {
    clean();

    this->num_routes = sol->num_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->lower_num_nodes_per_route[i] = sol->num_nodes_per_route[i];
        memcpy(this->lower_routes[i], sol->routes[i], sizeof(int) * node_cap);
    }
}

void Follower::export_solution(Solution* sol) const {
    sol->lower_cost = this->lower_cost;
}

void Follower::load_individual(const Individual* ind) {
    clean();

    this->num_routes = ind->upper_cost.nb_routes;
    for (int i = 0; i < num_routes; ++i) {
        this->lower_num_nodes_per_route[i] = static_cast<int>(ind->chromR[i].size()) + 2;

        memcpy(&this->lower_routes[i][1], ind->chromR[i].data(),ind->chromR[i].size() * sizeof(int));
    }
}

void Follower::export_individual(Individual* ind) const {
    ind->lower_cost = this->lower_cost;
}

double Follower::insert_station_by_simple_enum(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;
    int* route = new int[length];
    memcpy(route, repaired_route, sizeof(int) * length);

    vector<double> accumulated_distance(length, 0);
    for (int i = 1; i < length; i++) {
        accumulated_distance[i] = accumulated_distance[i - 1] + instance->get_distance(route[i], route[i - 1]);
    }
    if (accumulated_distance.back() <= preprocessor->max_cruise_distance_) {
        delete[] route;
        return accumulated_distance.back();
    }

    int upper_bound = static_cast<int>((accumulated_distance.back() / preprocessor->max_cruise_distance_ + 1));
    int lower_bound = static_cast<int>((accumulated_distance.back() / preprocessor->max_cruise_distance_));
    int* chosen_pos = new int[length];
    int* best_chosen_pos = new int[length]; // customized variable
    double final_cost = numeric_limits<double>::max();
    double best_cost = final_cost; // customized variable
    for (int i = lower_bound; i <= upper_bound; i++) {
        recursive_charging_placement(0, i, chosen_pos, best_chosen_pos, final_cost, i, route, length, accumulated_distance);

        if (final_cost < best_cost) {
            memset(repaired_route, 0, sizeof(int) * repaired_length);
            int currentIndex = 0;
            int idx = 0;
            for (int j = 0; j < i; ++j) {
                int from = route[best_chosen_pos[j]];
                int to = route[best_chosen_pos[j] + 1];
                int station = preprocessor->best_station_[from][to];

                int numElementsToCopy = best_chosen_pos[j] + 1 - idx;
                memcpy(&repaired_route[currentIndex], &route[idx], numElementsToCopy * sizeof(int));

                currentIndex += numElementsToCopy;

                repaired_route[currentIndex++] = station;
                idx = best_chosen_pos[j] + 1;
            }

            int remainingElementsToCopy = length - idx;
            memcpy(&repaired_route[currentIndex], &route[idx], remainingElementsToCopy * sizeof(int));
            repaired_length = currentIndex + remainingElementsToCopy;

            best_cost = final_cost;
        }
    }
    delete[] chosen_pos;
    delete[] best_chosen_pos;
    delete[] route;
    return (final_cost != std::numeric_limits<double>::max()) ? final_cost : -1;
}

double Follower::insert_station_by_remove_enum(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;
    int* route = new int [length];
    memcpy(route, repaired_route, sizeof(int) * length);

    list<pair<int, int>> stationInserted;
    for (int i = 0; i < length - 1; i++) {
        double allowedDis = preprocessor->max_cruise_distance_;
        if (i != 0) {
            allowedDis = preprocessor->max_cruise_distance_ - instance->get_distance(stationInserted.back().second, route[i]);
        }
        int onestation = preprocessor->get_best_and_feasible_station(route[i], route[i + 1], allowedDis);
        if (onestation == -1) {
            delete[] route;
            return -1;
        }
        stationInserted.emplace_back(i, onestation);
    }
    while (!stationInserted.empty())
    {
        bool change = false;
        auto delone = stationInserted.begin();
        double savedis = 0;
        auto itr = stationInserted.begin();
        auto next = itr;
        next++;
        if (next != stationInserted.end()) {
            int endInd = next->first;
            int endstation = next->second;
            double sumdis = 0;
            for (int i = 0; i < endInd; i++) {
                sumdis += instance->get_distance(route[i], route[i + 1]);
            }
            sumdis += instance->get_distance(route[endInd], endstation);
            if (sumdis <= preprocessor->max_cruise_distance_) {
                savedis = instance->get_distance(route[itr->first], itr->second)
                          + instance->get_distance(itr->second, route[itr->first + 1])
                          - instance->get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        else {
            double sumdis = 0;
            for (int i = 0; i < length - 1; i++) {
                sumdis += instance->get_distance(route[i], route[i + 1]);
            }
            if (sumdis <= preprocessor->max_cruise_distance_) {
                savedis = instance->get_distance(route[itr->first], itr->second)
                          + instance->get_distance(itr->second, route[itr->first + 1])
                          - instance->get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        itr++;
        while (itr != stationInserted.end())
        {
            int startInd, endInd;
            next = itr;
            next++;
            auto prev = itr;
            prev--;
            double sumdis = 0;
            if (next != stationInserted.end()) {
                startInd = prev->first + 1;
                endInd = next->first;
                sumdis += instance->get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < endInd; i++) {
                    sumdis += instance->get_distance(route[i], route[i + 1]);
                }
                sumdis += instance->get_distance(route[endInd], next->second);
                if (sumdis <= preprocessor->max_cruise_distance_) {
                    double savedistemp = instance->get_distance(route[itr->first], itr->second)
                                         + instance->get_distance(itr->second, route[itr->first + 1])
                                         - instance->get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            else {
                startInd = prev->first + 1;
                sumdis += instance->get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < length - 1; i++) {
                    sumdis += instance->get_distance(route[i], route[i + 1]);
                }
                if (sumdis <= preprocessor->max_cruise_distance_) {
                    double savedistemp = instance->get_distance(route[itr->first], itr->second)
                                         + instance->get_distance(itr->second, route[itr->first + 1])
                                         - instance->get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            itr++;
        }
        if (savedis != 0) {
            stationInserted.erase(delone);
            change = true;
        }
        if (!change) {
            break;
        }
    }
    double sum = 0;
    for (int i = 0; i < length - 1; i++) {
        sum += instance->get_distance(route[i], route[i + 1]);
    }
    int currentIndex = 0;
    int idx = 0;
    for (auto& e : stationInserted) {
        int pos = e.first;
        int stat = e.second;
        sum -= instance->get_distance(route[pos], route[pos + 1]);
        sum += instance->get_distance(route[pos], stat);
        sum += instance->get_distance(stat, route[pos + 1]);

        int numElementsToCopy = pos + 1 - idx;
        memcpy(&repaired_route[currentIndex], &route[idx], numElementsToCopy * sizeof(int));
        currentIndex += numElementsToCopy;

        repaired_route[currentIndex++] = stat;

        idx = pos + 1;
    }
    int remainingElementsToCopy = length - idx;
    memcpy(&repaired_route[currentIndex], &route[idx], remainingElementsToCopy * sizeof(int));
    repaired_length = currentIndex + remainingElementsToCopy;

    delete[] route;
    return sum;
}

void Follower::recursive_charging_placement(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& final_cost, int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance) const {
    stack<State> stk;
    stk.push({m_len, n_len, m_len});

    while (!stk.empty()) {
        auto& s = stk.top();

        if (s.n_len == 0) {
            stk.pop();
            double dis_sum = accumulated_distance.back();
            for (int j = 0; j < cur_upper_bound; j++) {
                int first_node = route[chosen_pos[j]];
                int second_node = route[chosen_pos[j] + 1];
                int the_station = preprocessor->best_station_[first_node][second_node];
                dis_sum -= instance->get_distance(first_node, second_node);
                dis_sum += instance->get_distance(first_node, the_station);
                dis_sum += instance->get_distance(the_station, second_node);
            }
            if (dis_sum < final_cost) {
                final_cost = dis_sum;
                for (int j = 0; j < cur_upper_bound; ++j) {
                    best_chosen_pos[j] = chosen_pos[j];
                }
            }
            continue;
        }

        if (s.i <= length - 1 - s.n_len) {
            if (cur_upper_bound == s.n_len) {
                double one_dis = instance->get_distance(route[s.i], preprocessor->best_station_[route[s.i]][route[s.i + 1]]);
                if (accumulated_distance[s.i] + one_dis > preprocessor->max_cruise_distance_) {
                    break;
                }
            } else {
                int last_pos = chosen_pos[cur_upper_bound - s.n_len - 1];
                double one_dis = instance->get_distance(route[last_pos + 1], preprocessor->best_station_[route[last_pos]][route[last_pos + 1]]);
                double two_dis = instance->get_distance(route[s.i], preprocessor->best_station_[route[s.i]][route[s.i + 1]]);
                if (accumulated_distance[s.i] - accumulated_distance[last_pos + 1] + one_dis + two_dis > preprocessor->max_cruise_distance_) {
                    break;
                }
            }
            if (s.n_len == 1) {
                double one_dis = accumulated_distance.back() - accumulated_distance[s.i + 1] + instance->get_distance(preprocessor->best_station_[route[s.i]][route[s.i + 1]], route[s.i + 1]);
                if (one_dis > preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            chosen_pos[cur_upper_bound - s.n_len] = s.i;
            stk.push({s.i + 1, s.n_len - 1, s.i + 1});
            s.i++;
        } else {
            stk.pop();
        }
    }
}

double Follower::insert_station_by_all_enumeration(int* repaired_route, int& repaired_length) const {
    const int length = repaired_length;
    int* route = new int[length];
    memcpy(route, repaired_route, sizeof(int) * length);

    vector<double> accumulated_distance(length, 0);
    for (int i = 1; i < length; i++) {
        accumulated_distance[i] = accumulated_distance[i - 1] + instance->get_distance(route[i], route[i - 1]);
    }
    if (accumulated_distance.back() <= preprocessor->max_cruise_distance_) {
        delete[] route;
        return accumulated_distance.back();
    }

    const int upper_bound = ceil(accumulated_distance.back() / preprocessor->max_cruise_distance_);
    const int lower_bound = floor(accumulated_distance.back() / preprocessor->max_cruise_distance_);
    int* chosen_pos = new int[length];
    int* chosen_sta = new int[length];
    double cost = numeric_limits<double>::max();
    ChargingMeta meta;
    meta.cost = numeric_limits<double>::max();
    for (int i = lower_bound; i <= upper_bound; i++) {
        ChargingMeta iter_meta = try_enumerate_n_stations_to_route(0, i, chosen_sta, chosen_pos,cost, i, route, length, accumulated_distance);
        if (cost != numeric_limits<double>::max() && cost < meta.cost) {
            meta = iter_meta;
        }
    }
    delete[] chosen_pos;
    delete[] chosen_sta;
    delete[] route;

    if (cost != numeric_limits<double>::max()) {
        for (int k = meta.num_stations - 1; k >= 0; k--) {
            int insertPos = meta.chosen_pos[k] + 1;
            for (int i = repaired_length; i > insertPos; i--) {
                repaired_route[i] = repaired_route[i - 1];
            }
            repaired_route[insertPos] = meta.chosen_sta[k];
            repaired_length++;
        }

        return cost;
    }
    else {
        return -1;
    }
}

ChargingMeta Follower::try_enumerate_n_stations_to_route(int m_len, int n_len, int *chosen_sta, int *chosen_pos,
                                                         double &cost, int cur_upper_bound, int *route, int length,
                                                         vector<double> &accumulated_distance) const {
    ChargingMeta meta;

    stack<State> stk;

    // Push the initial state
    stk.push({m_len, n_len, m_len, 0});

    while (!stk.empty()) {
        auto& s = stk.top(); // Get the current state

        // If n_len == 0, evaluate the solution
        if (s.n_len == 0) {
            stk.pop(); // Backtrack
            bool feasible = true;
            double piece_distance = accumulated_distance[chosen_pos[0]] + instance->get_distance(route[chosen_pos[0]], chosen_sta[0]);
            if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;

            for (int k = 1; feasible && k < cur_upper_bound; k++) {
                piece_distance = accumulated_distance[chosen_pos[k]] - accumulated_distance[chosen_pos[k - 1] + 1];
                piece_distance += instance->get_distance(chosen_sta[k - 1], route[chosen_pos[k - 1] + 1]);
                piece_distance += instance->get_distance(chosen_sta[k], route[chosen_pos[k]]);
                if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;
            }

            piece_distance = accumulated_distance.back() - accumulated_distance[chosen_pos[cur_upper_bound - 1] + 1];
            piece_distance += instance->get_distance(route[chosen_pos[cur_upper_bound - 1] + 1], chosen_sta[cur_upper_bound - 1]);
            if (piece_distance > preprocessor->max_cruise_distance_) feasible = false;

            if (feasible) {
                double total_distance = accumulated_distance.back();
                for (int k = 0; k < cur_upper_bound; k++) {
                    int first_node = route[chosen_pos[k]];
                    int second_node = route[chosen_pos[k] + 1];
                    total_distance -= instance->get_distance(first_node, second_node);
                    total_distance += instance->get_distance(first_node, chosen_sta[k]);
                    total_distance += instance->get_distance(chosen_sta[k], second_node);
                }
                // produce the repaired route
                if (total_distance < cost) {
                    cost = total_distance;
                    meta.cost = cost;
                    meta.num_stations = cur_upper_bound;
                    meta.chosen_pos.assign(chosen_pos, chosen_pos + meta.num_stations);
                    meta.chosen_sta.assign(chosen_sta, chosen_sta + meta.num_stations);
                }
            }
            continue;
        }

        // Iterate through route positions
        if (s.i <= length - 1 - s.n_len) {
            if (cur_upper_bound == s.n_len) {
                if (accumulated_distance[s.i] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            } else {
                if (accumulated_distance[s.i] - accumulated_distance[chosen_pos[cur_upper_bound - s.n_len - 1] + 1] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            if (s.n_len == 1) {
                if (accumulated_distance.back() - accumulated_distance[s.i + 1] >= preprocessor->max_cruise_distance_) {
                    s.i++;
                    continue;
                }
            }

            // Iterate through stations
            while (s.stationIdx < instance->num_station_) {
                chosen_sta[cur_upper_bound - s.n_len] = preprocessor->station_ids_[s.stationIdx];
                chosen_pos[cur_upper_bound - s.n_len] = s.i;

                // Push the next state for further exploration
                stk.push({s.i + 1, s.n_len - 1, s.i + 1, 0});
                s.stationIdx++;
                break; // Go deeper
            }

            // If all stations are processed for the current position
            if (s.stationIdx == instance->num_station_) {
                s.stationIdx = 0; // Reset for the next iteration
                s.i++;            // Move to the next route position
            }
        } else {
            stk.pop(); // Backtrack if no more positions are left
        }
    }

    return std::move(meta);
}

std::ostream& operator<<(std::ostream& os, const Follower& follower) {
    os << "Number of Routes: " << follower.num_routes << "\n";
    os << "Lower Cost: " << follower.lower_cost << "\n";

    os << "Number of Nodes per route (lower): ";
    for (int i = 0; i < follower.route_cap; ++i) {
        os << follower.lower_num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Lower Routes: \n";
    for (int i = 0; i < follower.num_routes; ++i) {
        os << "Route " << i << ": ";
        for (int j = 0; j < follower.node_cap; ++j) {
            os << follower.lower_routes[i][j] << " ";
        }
        os << "\n";
    }
    os << "\n";

    return os;
}
