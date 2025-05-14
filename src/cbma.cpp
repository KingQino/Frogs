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
    mutation_prob = 0.5;
    mut_ind_prob = 0.2;

    gen = 0;
    gammaL = 1.2;
    gammaR = 0.8;
    delta = 30;
    r = 0.0;

    max_neigh_attempts = preprocessor->params.max_neigh_attempts;
    max_chain_length = 256;

    indices = vector<int>(pop_size);
    std::iota(indices.begin(), indices.end(), 0);

    initializer = new Initializer(random_engine, instance, preprocessor);
    leaders.reserve(pop_size);
    followers.reserve(pop_size);
    partial_sols.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        leaders.emplace_back(std::make_unique<LeaderCbma>(instance, preprocessor));
        followers.emplace_back(std::make_unique<Follower>(instance, preprocessor));
        partial_sols.emplace_back(std::make_unique<PartialSolution>());
    }

    population.reserve(pop_size);

    elites.reserve(pop_size);
    non_elites.reserve(pop_size);
    immigrants.reserve(pop_size);
    offspring.reserve(pop_size);

    chromosome_length = instance->num_customer_;
    temp_dumb_routes.reserve(preprocessor->route_cap_);
    temp_child1.reserve(chromosome_length);
    temp_child2.reserve(chromosome_length);
    temp_cx_map1.reserve(chromosome_length);
    temp_cx_map2.reserve(chromosome_length);
    temp_best_individuals.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        temp_best_individuals.emplace_back(instance, preprocessor);
    }
}

Cbma::~Cbma() {
    delete initializer;
}

int Cbma::get_luby(int j) const {
    int k = 1;
    while ((1 << k) - 1 < j) ++k;
    int val = (1 << (k - 1));
    if (val >= max_chain_length) return max_chain_length;
    if ((1 << k) - 1 == j) return val;
    return get_luby(j - val + 1);
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
    for (int i = 0; i < pop_size; ++i) {
        vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();

        auto cost = instance->compute_total_distance(routes);
        auto demand = instance->compute_demand_sum_per_route(routes);

        auto ind = std::make_shared<Individual>(instance, preprocessor, routes, cost,
                                                demand);
        population.push_back(std::move(ind));
    }

    global_best = make_unique<Individual>(*population[0]);
}

void Cbma::run_heuristic() {
    for (int i = 0; i < pop_size; ++i) {
        auto& ind = population[i];

        leaders[i]->run_plus(ind.get());
        followers[i]->run(ind.get());

        if (ind->lower_cost < global_best->lower_cost) {
            *global_best = *ind;  // copy the content of ind to global_best, not deep copy
        }
    }

    after_local_opt = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    for (int i = 0; i < pop_size; ++i) {
        auto& ind = population[i];

        int k = 1; // index for Luby sequence
        int no_improve_counter = 0;

        temp_best_individuals[i] = *population[i];

        for (int j = 0; j < max_neigh_attempts; ++j) {
            // local search move
            // only accepts better moves
            bool has_moved = leaders[i]->local_search_move(partial_sols[i].get());

            if (has_moved) {
                followers[i]->run(partial_sols[i].get());

                leaders[i]->export_individual(ind.get());
                followers[i]->export_individual(ind.get());

                if (ind->upper_cost < temp_best_individuals[i].upper_cost) {
                    temp_best_individuals[i] = *ind;
                }
                if (ind->lower_cost < global_best->lower_cost) {
                    *global_best = *ind;  // copy the content of ind to global_best, not deep copy
                }

                no_improve_counter = 0;
                k = 1;
            } else {
                no_improve_counter += 1;
            }

            partial_sols[i]->clean();

            // If the number of consecutive unimprovements reaches Luby(k), a perturbation is triggered
            int strength = get_luby(k);
            if (no_improve_counter >= strength) {
//                cout << " Escape triggered with strength " << strength << " at iter " << j << endl;

                leaders[i]->load_individual(&temp_best_individuals[i]);
                leaders[i]->strong_perturbation(strength);
                leaders[i]->export_individual(ind.get());
                followers[i]->run(ind.get());

                no_improve_counter = 0;
                k += 1;

//                if (strength >= max_chain_length) k = 1;
            }
        }
    }

    after_neighbour_explore = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    if (gen % 100 == 0) {
        flush_row_into_evol_log();
    }

    std::sort(population.begin(), population.end(),
              [](const std::shared_ptr<Individual>& a, const std::shared_ptr<Individual>& b) {
                  return a->upper_cost < b->upper_cost;
              });


    elites.clear();
    for (int i = 0; i < 10; ++i) {
        elites.emplace_back(std::move(population[i]->get_chromosome()));
    }
    non_elites.clear();
    for (int i = 10; i < pop_size; ++i) {
        non_elites.emplace_back(std::move(population[i]->get_chromosome()));
    }
    immigrants.clear();
    for (int i = 0; i < 20; ++i) {
        vector<int> immigrant(preprocessor->customer_ids_);
        shuffle(immigrant.begin(), immigrant.end(), random_engine);
        immigrants.emplace_back(std::move(immigrant));
    }

    offspring.clear();
    // crossover and mutation
    // 5 pairs of elite and elite
    for (int i = 0; i < 5; ++i) {
        auto parents = select_random(elites, 2);
        auto& parent1 = parents[0];
        auto& parent2 = parents[1];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 25 pairs of elite and non_elite
    std::shuffle(non_elites.begin(), non_elites.end(), random_engine);
    for (int i = 0; i < 25; ++i) {
        auto parent1 = select_random(elites, 1)[0];
        auto& parent2 = non_elites[i];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 20 pairs of elite and immigrant
    for (int i = 0; i < 20; ++i) {
        auto parent1 = select_random(elites, 1)[0];
        auto& parent2 = immigrants[i];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }

    for (auto& chromosome: offspring) {
        if (uniform_real_dist(random_engine) < mutation_prob) {
            mut_shuffle_indexes(chromosome, mut_ind_prob);
        }
    }


    // update population
    for (int i = 0; i < pop_size; ++i) {
        population[i]->clean();

        temp_dumb_routes.clear();
        initializer->prins_split(offspring[i], temp_dumb_routes);

        population[i]->load_routes(temp_dumb_routes,
                                   instance->compute_total_distance(temp_dumb_routes),
                                   instance->compute_demand_sum_per_route(temp_dumb_routes));
    }

    gen++;
}

void Cbma::open_log_for_evolution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "gen,g_best,"
                     "af_opt_min,avg,max,std,"
                     "af_ne_min,avg,max,std,"
                     "evals\n";
}

void Cbma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.str("");
    oss_row_evol.clear();
    log_evolution.close();
}

void Cbma::flush_row_into_evol_log() {
    oss_row_evol << std::fixed << std::setprecision(3) << gen << "," << global_best->lower_cost <<","
    << after_local_opt.min << "," << after_local_opt.avg << "," << after_local_opt.max << "," << after_local_opt.std << ","
    << after_neighbour_explore.min << "," << after_neighbour_explore.avg << "," << after_neighbour_explore.max << "," << after_neighbour_explore.std << ","
    << instance->get_evals() << "\n";
}

void Cbma::save_log_for_solution() {
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
    vector<vector<int>> selected_seqs;
    selected_seqs.reserve(k);

    std::uniform_int_distribution<std::size_t> distribution(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t randomIndex = distribution(random_engine);
        selected_seqs.push_back(chromosomes[randomIndex]);
    }

    return std::move(selected_seqs);
}

void Cbma::cx_partially_matched(vector<int>& parent1, vector<int>& parent2) {
    uniform_int_distribution<int> distribution(0, chromosome_length - 1);

    int point1 = distribution(random_engine);
    int point2 = distribution(random_engine);

    if (point1 > point2) {
        swap(point1, point2);
    }

    // Copy the middle segment from parents to children
    temp_child1.clear();
    temp_child2.clear();
    temp_child1.insert(temp_child1.end(), parent1.begin() + point1, parent1.begin() + point2);
    temp_child2.insert(temp_child2.end(), parent2.begin() + point1, parent2.begin() + point2);

    temp_cx_map1.clear();
    temp_cx_map2.clear();
    // Initialize mapping with the middle segment
    for (int i = 0; i < point2 - point1; ++i) {
        temp_cx_map1[temp_child2[i]] = temp_child1[i];
        temp_cx_map2[temp_child1[i]] = temp_child2[i];
    }

    // Copy the rest of the genes, filling in the mapping
    for (int i = 0; i < chromosome_length; ++i) {
        if (i < point1 || i >= point2) {
            int gene1 = parent1[i];
            int gene2 = parent2[i];

            while (temp_cx_map1.find(gene1) != temp_cx_map1.end()) {
                gene1 = temp_cx_map1[gene1];
            }

            while (temp_cx_map2.find(gene2) != temp_cx_map2.end()) {
                gene2 = temp_cx_map2[gene2];
            }

            temp_child1.push_back(gene2);
            temp_child2.push_back(gene1);
        }
    }

    // Modify the input arguments directly
    parent1 = temp_child1;
    parent2 = temp_child2;
}

void Cbma::mut_shuffle_indexes(vector<int>& chromosome, double ind_pb) {
    uniform_real_distribution<double> dis(0.0, 1.0);
    uniform_int_distribution<int> swap_dist(0, chromosome_length - 2);

    for (int i = 0; i < chromosome_length; ++i) {
        if (dis(random_engine) < ind_pb) {
            int swapIndex = swap_dist(random_engine);
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