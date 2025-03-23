//
// Created by Yinghao Qin on 05/03/2025.
//

#include "initializer.hpp"

Initializer::Initializer(int seed_val, Case *instance, Preprocessor *preprocessor) {
    this->instance = instance;
    this->preprocessor = preprocessor;
    this->random_engine = default_random_engine(seed_val);
}

Initializer::~Initializer() = default;

// Prins, C., 2004. A simple and effective evolutionary algorithm for the vehicle routing problem. Computers & operations research, 31(12), pp.1985-2002.
vector<vector<int>> Initializer::prins_split(const vector<int>& chromosome) const {
    vector<int> x(chromosome.size() + 1, 0); // a giant tour starts from 0
    copy(chromosome.begin(), chromosome.end(), x.begin() + 1);

    vector<double> vv(x.size(), std::numeric_limits<double>::max()); // value, the accumulated cost of the shortest path from 0 to i
    vector<int> pp(x.size(), 0); // path, record the split routes of the corresponding shortest path
    vv[0] = 0.0;

    for (int i = 1; i < x.size(); ++i) {
        int load = 0;
        double cost = 0;
        int j = i;
        do
        {
            load += instance->get_customer_demand_(x[j]);
            if (i == j) {
                cost = instance->get_distance(instance->depot_, x[j]) * 2;
            } else {
                cost -= instance->get_distance(x[j -1], instance->depot_);
                cost += instance->get_distance(x[j -1], x[j]);
                cost += instance->get_distance(instance->depot_, x[j]);
            }

            if (load <= instance->max_vehicle_capa_) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load >instance->max_vehicle_capa_));
    }

    vector<vector<int>> all_routes;
    int j = static_cast<int>(x.size()) - 1;
    while (true) {
        int i = pp[j];
        vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
        all_routes.push_back(temp);
        j = i;
        if (i == 0) {
            break;
        }
    }

    return all_routes;
}

// Hien et al., "A greedy search based evolutionary algorithm for electric vehicle routing problem", 2023.
vector<vector<int>> Initializer::hien_clustering() {
    vector<int> chromosome = preprocessor->customer_ids_;

    // Clustering
    std::shuffle(chromosome.begin(),chromosome.end(), random_engine);
    vector<vector<int>> routes;
    vector<int> route;
    while (!chromosome.empty()) {
        route.clear();

        int anchor = chromosome.front();
        chromosome.erase(chromosome.begin());
        route.push_back(anchor);
        int cap = instance->get_customer_demand_(anchor);

        vector<int> nearby_customers = preprocessor->sorted_nearby_customers_[anchor];
        int length = static_cast<int>(nearby_customers.size()); // the size of nearby_customers

        for (int i = 0; i < length; ++i) {
            int node = nearby_customers[i];
            auto it = find(chromosome.begin(),chromosome.end(), node);
            if (it == chromosome.end()) {
                continue;
            }
            if (cap + instance->get_customer_demand_(node) <= instance->max_vehicle_capa_) {
                route.push_back(node);
                cap += instance->get_customer_demand_(node);
                chromosome.erase(it);
            } else {
                routes.push_back(route);
                break;
            }
        }
    }

    routes.push_back(route);

    return std::move(routes);
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

    vector<vector<int>> all_routes = prins_split(a_giant_tour);
    for (auto& route : all_routes) {
        route.insert(route.begin(), instance->depot_);
        route.push_back(instance->depot_);
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