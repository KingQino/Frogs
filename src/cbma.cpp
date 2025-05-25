//
// Created by Yinghao Qin on 05/04/2025.
//

#include "cbma.hpp"

const std::string Cbma::ALGORITHM = "Cbma";

Cbma::Cbma(int seed_val, Case *instance, Preprocessor *preprocessor) : HeuristicInterface("CBMA", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    max_neigh_attempts = preprocessor->params.max_neigh_attempts;
    max_sol_chain_length = 1024;

    gen = 0;
    pop_size = 100;
    mut_prob = 0.5;
    mut_ind_prob = 0.2;
    chromosome_length = instance->num_customer_;

    population.reserve(pop_size);
    elites.reserve(pop_size);
    non_elites.reserve(pop_size);
    immigrants.reserve(pop_size);
    offspring.reserve(pop_size);
    global_best_upper_so_far = std::numeric_limits<double>::max();

    initializer = std::make_unique<Initializer>(random_engine, instance, preprocessor);
    leaders.reserve(pop_size);
    followers.reserve(pop_size);
    partial_sols.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        leaders.emplace_back(std::make_unique<LeaderCbma>(instance, preprocessor));
        followers.emplace_back(std::make_unique<Follower>(instance, preprocessor));
        partial_sols.emplace_back(std::make_unique<PartialSolution>());
    }

    temp_dumb_routes.reserve(preprocessor->route_cap_);
    temp_child1.reserve(chromosome_length);
    temp_child2.reserve(chromosome_length);
    temp_cx_map1.reserve(chromosome_length);
    temp_cx_map2.reserve(chromosome_length);
    temp_individuals.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        temp_individuals.emplace_back(instance, preprocessor);
    }
}

Cbma::~Cbma() = default;

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

        population.emplace_back(instance, preprocessor, routes, cost, demand);
    }

    global_best = make_unique<Individual>(instance, preprocessor);
    global_best_upper_so_far = numeric_limits<double>::max();
}

void Cbma::run_heuristic() {
    // Greedy local search until getting stuck into local optimum
    // for (int i = 0; i < pop_size; ++i) {
    //     auto& ind = population[i];
    //
    //     leaders[i]->load_individual(&ind);
    //     followers[i]->run(&ind);
    //
    //     global_best_upper_so_far = std::min(global_best_upper_so_far, ind.upper_cost);
    //     if (ind.lower_cost < global_best->lower_cost) *global_best = ind;
    // }

    stats_greedy_local_opt = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    // Neighbourhood exploration by constructing solution chains
    for (int i = 0; i < pop_size; ++i) {
        neighbourhood_explore(i, max_neigh_attempts);
    }

    stats_neigh_explore = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));


    flush_row_into_evol_log();
    // if (gen % 100 == 0) {
    //     flush_row_into_evol_log();
    // }


    // Sort the population based on upper cost and select elites, non-elites, and introduce immigrants
    std::sort(population.begin(), population.end(),
              [](const Individual& a, const Individual& b) {
                  return a.upper_cost < b.upper_cost;
              });

    elites.clear();
    for (int i = 0; i < 10; ++i) {
        elites.emplace_back(std::move(population[i].get_chromosome()));
    }
    non_elites.clear();
    for (int i = 10; i < pop_size; ++i) {
        non_elites.emplace_back(std::move(population[i].get_chromosome()));
    }
    immigrants.clear();
    for (int i = 0; i < 30; ++i) {
        vector<int> immigrant(preprocessor->customer_ids_);
        shuffle(immigrant.begin(), immigrant.end(), random_engine);
        immigrants.emplace_back(std::move(immigrant));
    }

    offspring.clear();
    // crossover and mutation
    // 5 pairs of (elite x elite)
    for (int i = 0; i < 5; ++i) {
        auto parents = select_random(elites, 2);
        auto& parent1 = parents[0];
        auto& parent2 = parents[1];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 20 pairs of (elite x non_elite)
    std::shuffle(non_elites.begin(), non_elites.end(), random_engine);
    for (int i = 0; i < 20; ++i) {
        auto parent1 = select_random(elites, 1)[0];
        auto& parent2 = non_elites[i];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 20 pairs of (elite x immigrant)
    for (int i = 0; i < 20; ++i) {
        auto parent1 = select_random(elites, 1)[0];
        auto& parent2 = immigrants[i];

        cx_partially_matched(parent1, parent2);

        offspring.emplace_back(std::move(parent1));
        offspring.emplace_back(std::move(parent2));
    }
    // 10 immigrants
    for (int i = 20; i < 30; ++i) {
        offspring.emplace_back(std::move(immigrants[i]));
    }

    for (auto& chromosome: offspring) {
        if (uniform_real_dist(random_engine) < mut_prob) {
            mut_shuffle_indexes(chromosome, mut_ind_prob);
        }
    }


    // update population
    for (int i = 0; i < pop_size; ++i) {
        population[i].clean();

        temp_dumb_routes.clear();
        initializer->prins_split(offspring[i], temp_dumb_routes);

        population[i].load_routes(temp_dumb_routes,
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
                     "greedy_min,avg,max,std,"
                     "neigh_min,avg,max,std,"
                     "evals\n";
}

void Cbma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.str("");
    oss_row_evol.clear();
    log_evolution.close();
}

void Cbma::flush_row_into_evol_log() {
    oss_row_evol << std::fixed << std::setprecision(3)
                 << gen << "," << global_best->lower_cost << ","
                 << stats_greedy_local_opt.min << "," << stats_greedy_local_opt.avg << ","
                 << stats_greedy_local_opt.max << "," << stats_greedy_local_opt.std << ","
                 << stats_neigh_explore.min << "," << stats_neigh_explore.avg << ","
                 << stats_neigh_explore.max << "," << stats_neigh_explore.std << ","
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

int Cbma::get_luby(int j) const {
    int k = 1;
    while ((1 << k) - 1 < j) ++k;
    int val;
    while (true) {
        val = (1 << (k - 1));
        if ((1 << k) - 1 == j) break;
        j = j - val + 1;
        k = 1;
        while ((1 << k) - 1 < j) ++k;
    }
    return val >= max_sol_chain_length ? max_sol_chain_length : val;
}

bool Cbma::is_accepted(const double &candidate_cost, const double &current_cost, int steps) {
    if (candidate_cost < current_cost) {
        return true;
    }

    constexpr double T0=40.0;
    constexpr double alpha=0.99;
    const double temperature = std::max(1e-4, T0 * std::pow(alpha, steps));

    const double delta = candidate_cost - current_cost;
    const double prob = std::exp(-delta / temperature);

    return uniform_real_dist(random_engine) < prob;
}


void Cbma::neighbourhood_explore(const int index, const int max_attempts) {
    int k = 1;

    auto& ind = population[index];
    const auto& leader = leaders[index];
    const auto& follower = followers[index];
    const auto& partial_sol = partial_sols[index];

    // temp_history_list.clear();

    int steps = 0;  // records the number of steps taken in this neighbourhood exploration for terminating the loop
    while (steps < max_attempts) {
        const int length = get_luby(k++);

        temp_individuals[index] = ind;
        auto& dummy_ind = temp_individuals[index];

        leader->load_individual(&dummy_ind);
        follower->run(&dummy_ind);
        double current_cost = dummy_ind.upper_cost;

        // 记录 START，记录当前 upper_cost 和实际 length
        // temp_history_list.emplace_back(current_cost, START, length);
        // std::string exit_status = "normal";

        for (int i = 0; i < length; ++i) {
            steps++;
            if (leader->neighbour_move(partial_sol.get())) {
                follower->run(partial_sol.get());

                leader->export_individual(&dummy_ind);
                follower->export_individual(&dummy_ind);

                global_best_upper_so_far = std::min(global_best_upper_so_far, leader->upper_cost);
                if (dummy_ind.lower_cost < global_best->lower_cost) *global_best = dummy_ind;
            }

            partial_sol->clean();


            // 记录 SEARCH（中间搜索点）
            // temp_history_list.emplace_back(dummy_ind.upper_cost, SEARCH, 0);

            if (is_accepted(leader->upper_cost, current_cost, steps)) {
                // exit_status = "early_exit";
                break;
            }
        }

        if (dummy_ind.upper_cost < ind.upper_cost) {
            ind = dummy_ind;
        }

        // 记录 END（无论提前退出或正常结束）
        // if (exit_status == "early_exit") {
        // temp_history_list.emplace_back(dummy_ind.upper_cost, EARLY_END, 0);
        // } else {
        // temp_history_list.emplace_back(dummy_ind.upper_cost, FULL_END, 0);
        // }
    }

    // const string filename = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed) + "/"
    // + to_string(index) + ".csv";
    // save_vector_to_csv(temp_history_list, filename);
    // cout << "Individual " << index << " neighbourhood exploration: " << endl;
    // cout << endl;
    // temp_history_list.clear();
    // temp_history_list.shrink_to_fit();
}

vector<vector<int>> Cbma::select_random(const vector<vector<int>> &chromosomes, int k) {
    vector<vector<int>> selected_seqs;
    selected_seqs.reserve(k);

    std::uniform_int_distribution<std::size_t> distribution(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t idx = distribution(random_engine);
        selected_seqs.push_back(chromosomes[idx]);
    }

    return selected_seqs;
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
    // Initialise mapping with the middle segment
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

vector<double> Cbma::get_fitness_vector_from_upper_group(const vector<Individual>& group){
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind.upper_cost; });

    return ans;
}

vector<double> Cbma::get_fitness_vector_from_lower_group(const vector<Individual>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind.lower_cost; });

    return ans;
}

std::string Cbma::tag_to_str(HistoryTag tag) {
    switch (tag) {
        case HistoryTag::START: return "START";
        case HistoryTag::SEARCH:  return "SEARCH";
        case HistoryTag::EARLY_END: return "EARLY_END";
        case HistoryTag::FULL_END: return "FULL_END";
        default: return "unknown";
    }
}

void Cbma::save_vector_to_csv(const std::vector<std::tuple<double, HistoryTag, int>>& history_list,
                              const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Failed to open file for writing: " << filename << std::endl;
        return;
    }

    file << "upper_cost,tag,length\n";
    for (const auto& [cost, tag, length] : history_list) {
        file << cost << "," << tag_to_str(tag) << "," << length  << "\n";
    }

    file.close();
    std::cout << "[INFO] History written to: " << filename << " (" << history_list.size() << " entries)\n";
}