//
// Created by Yinghao Qin on 09/04/2025.
//

#include "sga.hpp"

const std::string Sga::ALGORITHM = "Sga";

Sga::Sga(int seed_val, Case *instance, Preprocessor* preprocessor)
        : HeuristicInterface("SGA", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    this->pop_size = 100;
    this->gen = 0;
    global_best = make_unique<Individual>();
    global_best_upper_so_far = numeric_limits<double>::max();

    data_logging1 = vector<double>(pop_size);
    data_logging2 = vector<double>(pop_size);

    uniform_int_dis = uniform_int_distribution<int>(0, pop_size - 1);
    mut_ind_prob = 0.2;
    max_neigh_attempts = preprocessor->params.max_neigh_attempts;

    initializer = new Initializer(random_engine, instance, preprocessor);
    leaders.reserve(pop_size);
    followers.reserve(pop_size);
    partial_sols.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        leaders.emplace_back(new LeaderSga(random_engine, instance, preprocessor));
        followers.emplace_back(new Follower(instance, preprocessor));
        partial_sols.emplace_back(new PartialSolution());
    }

}

Sga::~Sga() {
    delete initializer;
    for (int i = 0; i < pop_size; ++i) {
        delete leaders[i];
        delete followers[i];
        delete partial_sols[i];
    }
    population.clear();
    population.shrink_to_fit();
}

void Sga::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();

    if (enable_logging) {
        open_log_for_evolution();  // Open log if logging is enabled
    }

    initialize_heuristic();

    switch (stop_criteria) {
        case 0:
            while (!stop_criteria_max_evals()) {
                run_heuristic();
            }
            break;
        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                run_heuristic();
                duration = std::chrono::high_resolution_clock::now() - start;
            }
            break;

        default:
            std::cerr << "Invalid stop criteria option!" << std::endl;
            break;
    }

    if (enable_logging) {
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

void Sga::initialize_heuristic() {
    population.reserve(pop_size);

    for (int i = 0; i < pop_size; ++i) {
        vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();

        unique_ptr<Individual> ind_ptr = make_unique<Individual>(instance, preprocessor, routes,
                                                                 instance->compute_total_distance(routes),
                                                                 instance->compute_demand_sum_per_route(routes));

        population.emplace_back(std::move(ind_ptr));
    }

    global_best = make_unique<Individual>();
}

void Sga::run_heuristic() {
    gen++;

    #pragma omp parallel for schedule(dynamic) default(none)
    for (int i = 0; i < pop_size; ++i) {
        auto& ind = population[i];
        data_logging1[i] = ind->upper_cost;

        leaders[i]->local_improve(ind.get());
        followers[i]->run(ind.get());
        data_logging2[i] = ind->upper_cost;

        // 更新 global_best_upper_so_far（用原子或临界区）
        #pragma omp critical
        {
            global_best_upper_so_far = std::min(global_best_upper_so_far, ind->upper_cost);
        }

        // for loop for neighbour exploration
        for (int j = 0; j < max_neigh_attempts; ++j) {
            bool has_moved = leaders[i]->neighbour_explore(global_best_upper_so_far * 1.1, partial_sols[i]);
            if (has_moved) {
                followers[i]->run(partial_sols[i]);

                leaders[i]->export_individual(ind.get());
                followers[i]->export_individual(ind.get());

                // update the global best solution
                // 更新 global_best（只在某个条件下）
                #pragma omp critical
                {
                    if (ind->lower_cost < global_best->lower_cost) {
                        global_best = std::make_unique<Individual>(*ind);
                    }
                }
            }

            partial_sols[i]->clean();
        }

    }

    pop_cost_metrics = StatsInterface::calculate_statistical_indicators(data_logging1);
    pop_cost_metrics_after_impro = StatsInterface::calculate_statistical_indicators(data_logging2);
    flush_row_into_evol_log();


    vector<vector<int>> elites;
    elites.reserve(pop_size);
    for(auto& ind : population) {
        elites.emplace_back(std::move(ind->get_chromosome())); // encoding
    }

    vector<vector<int>> immigrants;
    immigrants.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        vector<int> immigrant(preprocessor->customer_ids_);
        shuffle(immigrant.begin(), immigrant.end(), random_engine);
        immigrants.emplace_back(std::move(immigrant));
    }

    vector<int> indices(pop_size);
    std::iota(indices.begin(), indices.end(), 0);

    vector<vector<int>> offspring;
    offspring.reserve(pop_size);
    // crossover and mutation
    // 10 pairs of elites
    for (int i = 0; i < 10; ++i) {
        int parent1_idx = i , parent2_idx = pop_size - 1 - i;

        vector<int> parent1 = elites[parent1_idx];
        vector<int> parent2 = elites[parent2_idx];

        cx_partially_matched(parent1, parent2);

        mut_shuffle_indexes(parent1, mut_ind_prob);
        mut_shuffle_indexes(parent2, mut_ind_prob);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 25 pairs of elite and immigrants
    std::shuffle(indices.begin(), indices.end(), random_engine);
    for (int i = 0; i < 25; ++i) {
        vector<int> parent1 = elites[i];
        vector<int> parent2 = immigrants[i];
        cx_partially_matched(parent1, parent2);
        mut_shuffle_indexes(parent1, mut_ind_prob);
        mut_shuffle_indexes(parent2, mut_ind_prob);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 15 pairs of immigrants
    std::shuffle(indices.begin(), indices.end(), random_engine);
    for (int i = 0; i < 15; ++i) {
        offspring.emplace_back(std::move(immigrants[i]));
        offspring.emplace_back(std::move(immigrants[pop_size - 1 - i]));
    }


    for (int i = 0; i < pop_size; ++i) {
        population[i]->clean();

        vector<vector<int>> dumb_routes = initializer->prins_split(offspring[i]);
        for (auto& route : dumb_routes) {
            route.insert(route.begin(), instance->depot_);
            route.push_back(instance->depot_);
        }

        population[i]->load_routes(dumb_routes,
                                   instance->compute_total_distance(dumb_routes),
                                   instance->compute_demand_sum_per_route(dumb_routes));
    }
}

void Sga::open_log_for_evolution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "iters,global_best,min,max,mean,std,after_impro_min,max,mean,std\n";
}

void Sga::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Sga::flush_row_into_evol_log() {
    oss_row_evol << gen << ",";

    if (global_best->lower_cost > 1e6) {
        oss_row_evol << std::scientific << std::setprecision(3) << global_best->lower_cost;
    } else {
        oss_row_evol << std::fixed << std::setprecision(3) << global_best->lower_cost;
    }

    // Reset to default float formatting
    oss_row_evol << std::defaultfloat << ",";

    // Ensure fixed precision for history metrics
    oss_row_evol << std::fixed << std::setprecision(3)
                 << pop_cost_metrics.min << ","
                 << pop_cost_metrics.max << ","
                 << pop_cost_metrics.avg << ","
                 << pop_cost_metrics.std << ","
                 << pop_cost_metrics_after_impro.min << ","
                 << pop_cost_metrics_after_impro.max << ","
                 << pop_cost_metrics_after_impro.avg << ","
                 << pop_cost_metrics_after_impro.std
                 << "\n";
}

void Sga::save_log_for_solution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);

    const string file_name = "solution." + instance->instance_name_ + ".txt";

    log_solution.open(directory + "/" + file_name);
    log_solution << fixed << setprecision(5) << global_best->lower_cost << endl;
    followers[0]->run(global_best.get());
    for (int i = 0; i < followers[0]->num_routes; ++i) {
        for (int j = 0; j < followers[0]->lower_num_nodes_per_route[i]; ++j) {
            log_solution << followers[0]->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

void Sga::cx_partially_matched(vector<int>& parent1, vector<int>& parent2) {
    int size = static_cast<int>(parent1.size());

    uniform_int_distribution<int> distribution(0, size - 1);

    int point1 = distribution(random_engine);
    int point2 = distribution(random_engine);

    if (point1 > point2) {
        swap(point1, point2);
    }

    // Copy the middle segment from parents to children
    vector<int> child1(parent1.begin() + point1, parent1.begin() + point2);
    vector<int> child2(parent2.begin() + point1, parent2.begin() + point2);

    // Create a mapping of genes between parents
    unordered_map<int, int> mapping1;
    unordered_map<int, int> mapping2;

    // Initialize mapping with the middle segment
    for (int i = 0; i < point2 - point1; ++i) {
        mapping1[child2[i]] = child1[i];
        mapping2[child1[i]] = child2[i];
    }

    // Copy the rest of the genes, filling in the mapping
    for (int i = 0; i < size; ++i) {
        if (i < point1 || i >= point2) {
            int gene1 = parent1[i];
            int gene2 = parent2[i];

            while (mapping1.find(gene1) != mapping1.end()) {
                gene1 = mapping1[gene1];
            }

            while (mapping2.find(gene2) != mapping2.end()) {
                gene2 = mapping2[gene2];
            }

            child1.push_back(gene2);
            child2.push_back(gene1);
        }
    }

    // Modify the input arguments directly
    parent1 = child1;
    parent2 = child2;
}

void Sga::mut_shuffle_indexes(vector<int>& chromosome, double ind_pb) {
    int size = static_cast<int>(chromosome.size());

    uniform_real_distribution<double> dis(0.0, 1.0);
    uniform_int_distribution<int> swap_dist(0, size - 2);

    for (int i = 0; i < size; ++i) {
        if (dis(random_engine) < ind_pb) {
            int swapIndex = swap_dist(random_engine);
            if (swapIndex >= i) {
                swapIndex += 1;
            }
            swap(chromosome[i], chromosome[swapIndex]);
        }
    }
}

