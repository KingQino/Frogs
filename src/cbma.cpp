//
// Created by Yinghao Qin on 05/04/2025.
//

#include "cbma.hpp"

const std::string Cbma::ALGORITHM = "Cbma";

Cbma::Cbma(int seed_val, Case *instance, Preprocessor *preprocessor) : HeuristicInterface("CBMA", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    pop_size = 100;
    elite_ratio = 0.01;
    immigrants_ratio = 0.05;
    crossover_prob = 1.0;
    mutation_prob = 0.8;
    mut_ind_prob = 0.2;

    gen = 0;
    gammaL = 1.2;
    gammaR = 0.8;
    delta = 30;
    r = 0.0;

    initializer = new Initializer(random_engine, instance, preprocessor);
    leader = new LeaderCbma(random_engine, instance, preprocessor);
    follower = new Follower(instance, preprocessor);

    elites.reserve(pop_size);
    offspring.reserve(pop_size);
    indices = vector<int>(pop_size);
    std::iota(indices.begin(), indices.end(), 0);
}

Cbma::~Cbma() {
    delete initializer;
    delete leader;
    delete follower;
}

void Cbma::run() {
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
        flush_row_into_evol_log();
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

void Cbma::initialize_heuristic() {
    population.clear();
    population.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();

        auto cost = instance->compute_total_distance(routes);
        auto demand = instance->compute_demand_sum_per_route(routes);

        auto ind = std::make_shared<Individual>(instance, preprocessor, routes, cost,
                                                demand);
        population.push_back(std::move(ind));
    }

    global_best = make_unique<Individual>(*population[0]);
    iter_best = make_unique<Individual>(*population[0]);
}

void Cbma::run_heuristic() {
    before_up_opt = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    for(auto& ind : population) {
        leader->run_plus(ind.get());
    }

    after_up_opt = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));


    // Current S2 has been selected and ready for recharging, make recharging on S2
    for (auto& ind:population) {
        follower->run(ind.get());
    }

    after_low_opt = calculate_statistical_indicators(get_fitness_vector_from_lower_group(population));

    // statistics
    iter_best = make_unique<Individual>(*select_best_lower_individual(population));
    if (global_best->lower_cost > iter_best->lower_cost) {
        global_best = make_unique<Individual>(*iter_best);
    }
    if (gen % 100 == 0) {
        flush_row_into_evol_log();
    }


    offspring.clear();
    elites.clear();
    for(auto& ind_ptr : population) {
        elites.emplace_back(std::move(ind_ptr->get_chromosome()));
    }

    std::shuffle(indices.begin(), indices.end(), random_engine);
    for (int i = 0; i < 10; ++i) {
        auto parent1 = elites[indices[i]];
        auto parent2 = elites[indices[pop_size - 1 - i]];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    std::shuffle(indices.begin(), indices.end(), random_engine);
    for (int i = 0; i < 40; ++i) {
        auto elite = std::move(elites[indices[i]]);

        vector<int> immigrant(preprocessor->customer_ids_);
        shuffle(immigrant.begin(), immigrant.end(), random_engine);

        cx_partially_matched(elite, immigrant);

        offspring.emplace_back(std::move(elite));
        offspring.emplace_back(std::move(immigrant));
    }

    for (auto& chromosome: offspring) {
        if (uniform_real_dist(random_engine) < mutation_prob) {
            mut_shuffle_indexes(chromosome, mut_ind_prob);
        }
    }


    // update population
    population[0] = make_shared<Individual>(*iter_best);
    for (int i = 1; i < pop_size; ++i) {
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

    gen++;
}

void Cbma::open_log_for_evolution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "gen,g_best,"
                     "af_up_min,avg,max,std,"
                     "evals\n";
}

void Cbma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Cbma::flush_row_into_evol_log() {
    oss_row_evol << std::fixed << std::setprecision(3) << gen << "," << global_best->lower_cost <<","
                 << after_up_opt.min << "," << after_up_opt.avg << "," << after_up_opt.max << "," << after_up_opt.std << ","
                 << instance->get_evals() << "\n";
}

void Cbma::save_log_for_solution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);

    const string file_name = "solution." + instance->instance_name_ + ".txt";

    log_solution.open(directory + "/" + file_name);
    log_solution << fixed << setprecision(5) << global_best->lower_cost << endl;
    follower->run(global_best.get());
    for (int i = 0; i < follower->num_routes; ++i) {
        for (int j = 0; j < follower->lower_num_nodes_per_route[i]; ++j) {
            log_solution << follower->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

shared_ptr<Individual> Cbma::select_best_upper_individual(const vector<shared_ptr<Individual>>& pop) {
    if (pop.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
        return ind1->upper_cost < ind2->upper_cost;
    };

    auto best = std::min_element(pop.begin(), pop.end(), comparator);

    return *best;
}

shared_ptr<Individual> Cbma::select_best_lower_individual(const vector<shared_ptr<Individual>>& pop) {
    if (pop.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
        return ind1->lower_cost < ind2->lower_cost;
    };

    auto best = std::min_element(pop.begin(), pop.end(), comparator);

    return *best;
}

vector<vector<int>>  Cbma::select_random(const vector<vector<int>> &chromosomes, int k) {
    vector<vector<int>> selectedSeqs;

    std::uniform_int_distribution<std::size_t> distribution(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t randomIndex = distribution(random_engine);
        selectedSeqs.push_back(chromosomes[randomIndex]);
    }

    return selectedSeqs;
}

void Cbma::cx_partially_matched(vector<int>& parent1, vector<int>& parent2) {
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

void Cbma::mut_shuffle_indexes(vector<int>& chromosome, double ind_pb) {
    int size = static_cast<int>(chromosome.size());

    uniform_real_distribution<double> dis(0.0, 1.0);
    uniform_int_distribution<int> swapDist(0, size - 2);

    for (int i = 0; i < size; ++i) {
        if (dis(random_engine) < ind_pb) {
            int swapIndex = swapDist(random_engine);
            if (swapIndex >= i) {
                swapIndex += 1;
            }
            swap(chromosome[i], chromosome[swapIndex]);
        }
    }
}

vector<double> Cbma::get_fitness_vector_from_upper_group(const vector<shared_ptr<Individual>>& group){
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->upper_cost; });

    return ans;
}

vector<double> Cbma::get_fitness_vector_from_lower_group(const vector<shared_ptr<Individual>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->lower_cost; });

    return ans;
}