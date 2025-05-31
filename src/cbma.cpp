//
// Created by Yinghao Qin on 05/04/2025.
//

#include "cbma.hpp"

const std::string Cbma::ALGORITHM = "Cbma";

Cbma::Cbma(int seed_val, Case *instance, Preprocessor *preprocessor) : HeuristicInterface("CBMA", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    max_neigh_attempts = preprocessor->params.max_neigh_attempts;
    max_perturbation_strength = 512;
    T0 = preprocessor->params.T0;
    alpha = preprocessor->params.alpha;
    min_window_size = preprocessor->params.min_win;
    max_window_size = preprocessor->params.max_win;
    window_k = preprocessor->params.win_k;

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
    temp_best_individuals.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        temp_best_individuals.emplace_back(instance, preprocessor);
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
    for (int i = 0; i < pop_size; ++i) {
        auto& ind = population[i];

        leaders[i]->fully_greedy_local_optimum(&ind);
        followers[i]->run(&ind);

        global_best_upper_so_far = std::min(global_best_upper_so_far, ind.upper_cost);
        if (ind.lower_cost < global_best->lower_cost) {
            *global_best = ind;  // copy the content of ind to global_best, not deep copy
        }
        // neighbourhood exploration starting points, half from global best, half from the individual itself
        // TODO: 1. all from global best, 2. all from individual, 3. random mix
        temp_best_individuals[i] = *global_best;
    }

    stats_greedy_local_opt = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    // Neighbourhood exploration by repeatedly applying strong perturbation and local search moves
    for (int i = 0; i < pop_size; ++i) {
        int luby_idx = 1; // index for Luby sequence

//        const string file = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" +
//                            to_string(seed) + "/" + to_string(i) + ".csv";
        int total_steps = 0;
        temp_history_list.clear();
        while (total_steps < max_neigh_attempts) {
            total_steps += neighbourhood_explore(i, luby_idx, temp_best_individuals[i],
                                                         population[i], temp_history_list);
        }
//        save_vector_to_csv(temp_history_list, file);
    }

    stats_neigh_explore = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));


    if (gen % 100 == 0) {
        flush_row_into_evol_log();
    }


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

    // TODO: Adaptive Selection Scheme should be implemented here
    // check if the elites are diverse enough, if not, then reduce the number of elites and increase the number of non-elites
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
    const string directory = preprocessor->params.kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
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
    const string directory = preprocessor->params.kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);

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
    return val >= max_perturbation_strength ? max_perturbation_strength : val;
}

inline void Cbma::update_recent_deltas(std::deque<double>& deltas, const double delta, const size_t max_size) {
    while (deltas.size() >= max_size) {
        deltas.pop_front();
    }
    deltas.push_back(delta);
}

size_t Cbma::get_dynamic_window(const double gap_ratio, const int min_window=20, const int max_window=500, const double k=0.5) {
    // Normalise gap_ratio to [0, 1) range using bounded transformation
    const double norm = 1.0 - std::exp(-k*gap_ratio);  // asymptotically approaches 1
    const double window = min_window + norm * (max_window - min_window);
    return static_cast<size_t>(std::clamp(static_cast<int>(std::round(window)), min_window, max_window));
}

bool Cbma::stuck_in_local_optima(const deque<double>& changes, const size_t window_size, const double epsilon=1e-3,
                                 double strong_delta_thresh=0.01) {
    if (changes.size() < window_size) return false;

    // Step 1: Count obvious changes
    const auto strong_change = std::count_if(changes.begin(), changes.end(),
        [strong_delta_thresh](const double d){ return std::abs(d) >= strong_delta_thresh; });

    // Step 2: Average magnitude of recent changes (absolute value)
    const double avg = std::accumulate(changes.begin(), changes.end(), 0.0,
        [](const double sum, const double d)
        { return sum + std::abs(d); }) / static_cast<double>(changes.size());

    // Step 3: Determine if the search is stuck
    // - No strong changes
    // - Overall change is small
    return (strong_change < 1) && (avg < epsilon);
}

int Cbma::neighbourhood_explore(const int individual_index, int& luby_index, Individual& temp_best, Individual& ind,
                                vector<tuple<double, HistoryTag, int>>& history_list) {
    int steps = 0;  // records the number of steps taken in this neighbourhood exploration for terminating the loop
    const int strength = get_luby(luby_index);

    auto* leader = leaders[individual_index].get();
    auto* follower = followers[individual_index].get();
    auto* partial_sol = partial_sols[individual_index].get();

    leader->load_individual(&temp_best);
    leader->perturbation(strength);
    leader->export_individual(&ind);
    follower->run(&ind);
//    history_list.emplace_back(ind.upper_cost, HistoryTag::PERTURB, strength); // after perturbation
    steps++;  // count the perturbation step


    // Parameters controlling the search intensity in the local search
    // Parameters used to judge whether the local search is stuck in local optima
    temp_recent_deltas.clear();
    auto& recent_deltas = temp_recent_deltas;
    size_t window_size = 10; // initial window size for recent moves, TODO: initial window size should be tuned

    bool local_optima_flag = false;
    bool is_profitable = false;
    int local_step = 0;

    while ((recent_deltas.size() < window_size || !local_optima_flag) && local_step < MAX_LOCAL_STEPS) {
        double temperature = LeaderCbma::get_temperature(local_step, T0, alpha);
        const double current_cost = leader->upper_cost;
        const bool has_moved = leader->local_search_move(partial_sol, temperature);
        const double candidate_cost = leader->upper_cost;

        local_step++;

        if (has_moved) {
            follower->run(partial_sol);
            leader->export_individual(&ind);
            follower->export_individual(&ind);

            update_recent_deltas(recent_deltas, current_cost - candidate_cost, window_size);

            if (candidate_cost < temp_best.upper_cost) {
                temp_best = ind;
                global_best_upper_so_far = std::min(global_best_upper_so_far, candidate_cost);
                is_profitable = true;
            }

            if (ind.lower_cost < global_best->lower_cost) *global_best = ind;

//            history_list.emplace_back(ind.upper_cost, HistoryTag::SEARCH, 0);

        } else {
            // if there is no move, record it as a stable segment (delta = 0)
            update_recent_deltas(recent_deltas, 0.0, window_size);

            local_optima_flag = stuck_in_local_optima(recent_deltas, window_size);
//            if (local_optima_flag) {
//                history_list.emplace_back(ind.upper_cost, HistoryTag::STUCK, 0);
//            } else {
//                history_list.emplace_back(ind.upper_cost, HistoryTag::SEARCH, 0);
//            }
        }

        partial_sol->clean();

        const double gap_ratio = std::abs(current_cost - global_best_upper_so_far) / global_best_upper_so_far;
        window_size = get_dynamic_window(gap_ratio, min_window_size, max_window_size, window_k);
    }


    steps += local_step;  // add the local search steps

    // If the local search (while-loop) yields improvement, the region is promising — apply slight perturbation and
    // continue searching nearby.
    // If several consecutive local search stages show no improvement, gradually increase the perturbation to escape
    // local optima.
    luby_index = is_profitable ? std::max(1, luby_index - 1) : luby_index + 1;

    return steps;
}

std::string Cbma::tag_to_str(HistoryTag tag) {
    switch (tag) {
        case PERTURB: return "perturb";
        case SEARCH:  return "search";
        case STUCK:   return "stuck";
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

    file << "upper_cost,tag,strength\n";
    for (const auto& [cost, tag, strength] : history_list) {
        file << cost << "," << tag_to_str(tag) << "," << strength  << "\n";
    }

    file.close();
    std::cout << "[INFO] History written to: " << filename << " (" << history_list.size() << " entries)\n";
}

vector<vector<int>>  Cbma::select_random(const vector<vector<int>> &chromosomes, int k) {
    vector<vector<int>> selected_seqs;
    selected_seqs.reserve(k);

    std::uniform_int_distribution<std::size_t> distribution(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t idx = distribution(random_engine);
        selected_seqs.push_back(chromosomes[idx]);
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