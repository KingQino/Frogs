//
// Created by Yinghao Qin on 05/03/2025.
//

#include "initializer.hpp"

Initializer::Initializer(std::mt19937& engine, Case *instance, Preprocessor *preprocessor)
    : random_engine(engine), instance(instance), preprocessor(preprocessor) {

    n = instance->num_customer_;
    temp_x.reserve(n + 1);
    temp_vv.reserve(n + 1);
    temp_pp.reserve(n + 1);


    depot_dist.resize(n + 1, 0.0);  // depot到每个点的距离
    // 预处理 depot 到每个点的距离
    for (int i = 1; i <= n; ++i) {
        depot_dist[i] = instance->get_distance(instance->depot_, preprocessor->customer_ids_[i - 1]);
    }
}

Initializer::~Initializer() = default;

// Prins, C., 2004. A simple and effective evolutionary algorithm for the vehicle routing problem. Computers & operations research, 31(12), pp.1985-2002.
vector<vector<int>> Initializer::prins_split(const vector<int>& chromosome) const {
    // giant tour starts from depot (node 0)
    temp_x[0] = 0;
    memcpy(temp_x.data() + 1, chromosome.data(), n * sizeof(int)); // copy the chromosome to temp_x

    fill(temp_vv.begin(), temp_vv.begin() + n + 1, numeric_limits<double>::max());
    temp_vv[0] = 0.0;
    fill(temp_pp.begin(), temp_pp.begin() + n + 1, 0);

    // dynamic programming to find the shortest path
    for (int i = 1; i <= n; ++i) {
        int load = 0;
        double cost = depot_dist[temp_x[i]] * 2;  // initial cost = 2 by dist(depot, x[1])
        for (int j = i; j <= n; ++j) {
            load += instance->get_customer_demand_(temp_x[j]);
            if (load > instance->max_vehicle_capa_) break;

            if (j > i) {
                cost -= depot_dist[temp_x[j - 1]];
                cost += instance->get_distance(temp_x[j - 1], temp_x[j]);
                cost += depot_dist[temp_x[j]];
            }

            if (temp_vv[i - 1] + cost < temp_vv[j]) {
                temp_vv[j] = temp_vv[i - 1] + cost;
                temp_pp[j] = i - 1;
            }
        }
    }

    vector<vector<int>> all_routes;
    int j = n;
    while (j > 0) {
        int i = temp_pp[j];
        all_routes.emplace_back(temp_x.begin() + i + 1, temp_x.begin() + j + 1);
        j = i;
    }

    return all_routes;
}

// Hien et al., "A greedy search based evolutionary algorithm for electric vehicle routing problem", 2023.
vector<vector<int>> Initializer::hien_clustering() {
    vector<int> chromosome = preprocessor->customer_ids_;

    std::shuffle(chromosome.begin(), chromosome.end(), random_engine);

    vector<vector<int>> routes;
    vector<int> route;
    route.reserve(instance->num_customer_); // 预留，防止反复扩容

    while (!chromosome.empty()) {
        route.clear();
        int anchor = chromosome.back();
        chromosome.pop_back();
        route.push_back(anchor);

        int cap = instance->get_customer_demand_(anchor);
        const auto& nearby_customers = preprocessor->sorted_nearby_customers_[anchor];

        for (int node : nearby_customers) {
            auto it = find(chromosome.begin(), chromosome.end(), node);
            if (it == chromosome.end()) continue;

            int demand = instance->get_customer_demand_(node);
            if (cap + demand <= instance->max_vehicle_capa_) {
                cap += demand;
                route.push_back(node);
                chromosome.erase(it);
            }

            if (cap >= instance->max_vehicle_capa_) {
                break; // 提前剪枝，满了就停
            }
        }

        routes.push_back(route);
    }

    return routes;
}

void Initializer::hien_balancing(vector<vector<int>>& routes) {
    vector<int>& lastRoute = routes.back();

    uniform_int_distribution<int> distribution(0, static_cast<int>(lastRoute.size() - 1));
    int customer = lastRoute[distribution(random_engine)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (int node : lastRoute) {
        cap1 += instance->get_customer_demand_(node);
    }
    int size = instance->num_customer_ - 1; // the size of nearby_customers

    for (int i = 0; i < size; ++i) {
        int x = preprocessor->sorted_nearby_customers_[customer][i];
        if (find(lastRoute.begin(), lastRoute.end(), x) != lastRoute.end()) {
            continue;
        }
        auto route2It = find_if(routes.begin(), routes.end(), [x](const vector<int>& route) {
            return find(route.begin(), route.end(), x) != route.end();
        });

        if (route2It != routes.end()) {
            vector<int>& route2 = *route2It;
            int cap2 = 0;
            for (int node : route2) {
                cap2 += instance->get_customer_demand_(node);
            }

            int demand_X = instance->get_customer_demand_(x);

            if (demand_X + cap1 <= instance->max_vehicle_capa_ && abs((cap1 + demand_X) - (cap2 - demand_X)) < abs(cap1 - cap2)) {
                route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
                lastRoute.push_back(x);
                cap1 += demand_X;
            } else {
                break;
            }
        }
    }
}

vector<vector<int>> Initializer::routes_constructor_with_split() {
    vector<int> a_giant_tour(preprocessor->customer_ids_);

    shuffle(a_giant_tour.begin(), a_giant_tour.end(), random_engine);

    vector<vector<int>> all_routes = prins_split(a_giant_tour); // 这里传的是不带depot的顾客

    for (auto& route : all_routes) {
        route.insert(route.begin(), instance->depot_); // 加在每条路线最开头
        route.push_back(instance->depot_); // 最后也补 depot
    }

    return all_routes;
}

vector<vector<int>> Initializer::routes_constructor_with_hien_method(){
    vector<vector<int>> routes = hien_clustering();
    hien_balancing(routes);

    for (auto& route : routes) {
        route.insert(route.begin(), instance->depot_);
        route.push_back(instance->depot_);
    }

    return std::move(routes);
}

// Jia Ya-Hui, et al., "Confidence-Based Ant Colony Optimization for Capacitated Electric Vehicle Routing Problem With Comparison of Different Encoding Schemes", 2022
vector<vector<int>> Initializer::routes_constructor_with_direct_encoding() {
    vector<int> customers_(preprocessor->customer_ids_);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance->depot_}; // the first route starts from depot_ 0

    vector<vector<int>> all_routes;
    while(!customers_.empty()) {
        vector<int> all_temp;
        for(int i : customers_) {
            if(instance->get_customer_demand_(i) <= instance->max_vehicle_capa_ - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        int remain_total_demand_ = accumulate(customers_.begin(), customers_.end(), 0, [&](int total, int i) {
            return total + instance->get_customer_demand_(i);
        });
        if(remain_total_demand_ <= instance->max_vehicle_capa_ * (instance->num_vehicle_ - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance->depot_); // add depot_ node into the all_temp
        }

        int cur = route.back();
        uniform_int_distribution<> distribution(0, static_cast<int>(all_temp.size()) - 1);
        int next = all_temp[distribution(random_engine)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance->depot_) {
            if (cur == instance->depot_) continue; // fix-bug
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance->get_customer_demand_(next);
            customers_.erase(remove(customers_.begin(), customers_.end(), next), customers_.end());
        }
    }

    route.push_back(instance->depot_);
    all_routes.push_back(route);

    return all_routes;
}