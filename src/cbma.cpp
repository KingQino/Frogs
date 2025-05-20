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
    mutation_ind_prob = 0.2;

    gen = 0;
    gammaL = 1.2;
    gammaR = 0.8;
    delta = 30;
    r = 0.0;

    initializer = new Initializer(random_engine, instance, preprocessor);
    leader = new LeaderArray(random_engine, instance, preprocessor);
    follower = new Follower(instance, preprocessor);
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
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

void Cbma::initialize_heuristic() {
    population.clear();
    population.reserve(pop_size);
    for (int i = 0; i < pop_size; ++i) {
        vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
        auto sol = make_shared<Solution>(instance, preprocessor, routes,
                                         instance->compute_total_distance(routes),
                                         instance->compute_demand_sum_per_route(routes));
        population.push_back(sol);
        routes.clear();
        routes.shrink_to_fit();
    }

    global_best = make_unique<Solution>(*population[0]);
    iter_best = make_unique<Solution>(*population[0]);
}

void Cbma::run_heuristic() {
    gen++;

    vector<double> data = get_fitness_vector_from_upper_group(population);
    S_stats = calculate_statistical_indicators(get_fitness_vector_from_upper_group(population));

    vector<shared_ptr<Solution>> S1 = population;
    double v1 = 0;
    double v2;
    shared_ptr<Solution> talented_ind = select_best_upper_individual(population);
    if (gen > delta) { //  switch off - False
        // when the generations are greater than the threshold, part of the upper-level sub-solutions S1 will be selected for local search
        double old_cost = talented_ind->upper_cost;
        leader->run(talented_ind.get());
        double new_cost = talented_ind->upper_cost;
        v1 = old_cost - new_cost;
        v2 = *std::max_element(P.begin(), P.end());
        if (v2 < v1) {
            v2 = v1 * gammaL;
        }

        S1.clear();
        for (auto& ind:population) {
            if (ind->upper_cost - v2 <= new_cost) S1.push_back(ind);
        }

        auto it = std::find(S1.begin(), S1.end(), talented_ind);
        // If genius_upper is found, remove it from S2
        if (it != S1.end()) {
            S1.erase(it);
        }
    }

    // make local search on S1
    v2 = 0;
    for(auto& ind : S1) {
        double old_cost = ind->upper_cost;

        leader->run(ind.get());

        if (v2 < old_cost - ind->upper_cost)
            v2 = old_cost - ind->upper_cost;
    }
    v2 = (v1 > v2) ? v1 : v2;
    P.push_back(v2);
    if (P.size() > delta)  P.pop_front();
    if (gen > delta) S1.push_back(talented_ind); //  *** switch off ***

    S1_stats = calculate_statistical_indicators(get_fitness_vector_from_upper_group(S1));

    // Current S1 has been selected and local search.
    // Pick a portion of the upper sub-solutions to go for recharging process, by the difference between before and after charging of the best solution in S1
    vector<shared_ptr<Solution>> S2 = S1;
    double v3;
    shared_ptr<Solution> outstanding_upper = select_best_upper_individual(S1);
    if (gen > 0) { // Switch = off False
        double old_cost = outstanding_upper->upper_cost; // fitness without recharging f
        follower->run(outstanding_upper.get());
        double new_cost = outstanding_upper->lower_cost; // fitness with recharging f
        v3 = new_cost - old_cost;
        if (r > v3) r = v3 * gammaR;

        S2.clear();
        for (auto& ind:S1) {
            if (ind->upper_cost + r <= new_cost)
                S2.push_back(ind);
        }

        auto it = std::find(S2.begin(), S2.end(), outstanding_upper);
        // If genius_upper is found, remove it from S2
        if (it != S2.end()) {
            S2.erase(it);
        }
    }

    // Current S2 has been selected and ready for recharging, make recharging on S2
    vector<shared_ptr<Solution>> S3;
    S3.push_back(outstanding_upper); //  *** switch off ***
    for (auto& ind:S2) {
        double old_cost = ind->upper_cost;
        follower->run(ind.get());
        double new_cost = ind->lower_cost;
        S3.push_back(ind);
        if (v3 > new_cost - old_cost)
            v3 = new_cost - old_cost;
    }
    if (r == 0 || r > v3) {
        r = v3;
    }

    S3_stats = calculate_statistical_indicators(get_fitness_vector_from_lower_group(S3));

    // statistics
    iter_best = make_unique<Solution>(*select_best_lower_individual(S3));
    if (global_best->lower_cost > iter_best->lower_cost) {
        global_best = make_unique<Solution>(*iter_best);
    }
    flush_row_into_evol_log();


    // Selection
    vector<vector<int>> promising_seqs;
    promising_seqs.reserve(S3.size());
    for(auto& sol : S3) {
        promising_seqs.push_back(sol->get_chromosome()); // encoding
    }

    vector<vector<int>> average_seqs;
    for(auto& sol : population) {
        // judge whether sol in S3 or not
        auto it = std::find(S3.begin(), S3.end(), sol);
        if (it != S3.end()) continue;
        average_seqs.push_back(sol->get_chromosome()); // encoding
    }

    vector<vector<int>> chromosomes;
    if (promising_seqs.size() == 1) {
        const vector<int>& father = promising_seqs[0];
        // 90% - elite x non-elites
        for (int i = 0; i < int (0.45 * pop_size); ++i) {
            vector<int> _father(father);
            vector<int> mother = select_random(average_seqs, 1)[0];
            cx_partially_matched(_father, mother);
            chromosomes.push_back(std::move(_father));
            chromosomes.push_back(std::move(mother));
        }
        // 9%  - elite x immigrants
        for (int i = 0; i < int(0.05 * pop_size); ++i) {
            vector<int> _father(father);
            vector<int> mother(preprocessor->customer_ids_);
            shuffle(mother.begin(), mother.end(), random_engine);
            cx_partially_matched(_father, mother);
            chromosomes.push_back(std::move(_father));
            chromosomes.push_back(std::move(mother));
        }
//        chromosomes.pop_back();
        // free 1 space  - best ind
    } else {
        // part of elites x elites
        int num_promising_seqs = static_cast<int>(promising_seqs.size());
        int loop_num = int(num_promising_seqs / 2.0) <= (pop_size/2) ? int(num_promising_seqs / 2.0) : int(pop_size/4);
        for (int i = 0; i < loop_num; ++i) {
            vector<vector<int>> parents = select_random(promising_seqs, 2);
            cx_partially_matched(parents[0], parents[1]);
            chromosomes.push_back(std::move(parents[0]));
            chromosomes.push_back(std::move(parents[1]));
        }
        // portion of elites x non-elites
        int num_promising_x_average = pop_size - static_cast<int>(chromosomes.size());
        for (int i = 0; i < int(num_promising_x_average / 2.0); ++i) {
            vector<int> parent1 = select_random(promising_seqs, 1)[0];
            vector<int> parent2 = select_random(average_seqs, 1)[0];
            cx_partially_matched(parent1, parent2);
            chromosomes.push_back(std::move(parent1));
            chromosomes.push_back(std::move(parent2));
        }
    }

    for (auto& chromosome: chromosomes) {
        if (uniform_real_dist(random_engine) < mutation_prob) {
            mut_shuffle_indexes(chromosome, mutation_ind_prob);
        }
    }

    // destroy all the individual objects
    S3.clear();
    S2.clear();
    S1.clear();
    population.clear();
    population.shrink_to_fit();


    // update population
    population.reserve(pop_size);
    population.push_back(make_shared<Solution>(*iter_best));
    for (int i = 0; i < pop_size - 1; ++i) {
        vector<vector<int>> dumb_routes = initializer->prins_split(chromosomes[i]);

        for (auto& route : dumb_routes) {
            route.insert(route.begin(), instance->depot_);
            route.push_back(instance->depot_);
        }

        population.push_back(make_shared<Solution>(instance, preprocessor,dumb_routes,
                                                   instance->compute_total_distance(dumb_routes),
                                                   instance->compute_demand_sum_per_route(dumb_routes))
        );
    }

}

void Cbma::open_log_for_evolution() {
    const string directory = preprocessor->params.kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "gen,g_best,"
                     "S_min,S_avg,S_max,S_std,"
                     "up_size,S1_min,S1_avg,S1_max,S1_std,"
                     "low_size,"
                     "evals\n";
}

void Cbma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Cbma::flush_row_into_evol_log() {
    oss_row_evol << std::fixed << std::setprecision(3) << gen << "," << global_best->lower_cost <<","
                 << S_stats.min << "," << S_stats.avg << "," << S_stats.max << "," << S_stats.std << ","
                 << S1_stats.size << "," << S1_stats.min << "," << S1_stats.avg << "," << S1_stats.max << "," << S1_stats.std << ","
                 << S3_stats.size << ","
                 << instance->get_evals() << "\n";
}

void Cbma::save_log_for_solution() {
    const string directory = preprocessor->params.kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);

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

shared_ptr<Solution> Cbma::select_best_upper_individual(const vector<shared_ptr<Solution>>& pop) {
    if (pop.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    auto comparator = [](const shared_ptr<Solution>& ind1, const shared_ptr<Solution>& ind2) {
        return ind1->upper_cost < ind2->upper_cost;
    };

    auto best = std::min_element(pop.begin(), pop.end(), comparator);

    return *best;
}

shared_ptr<Solution> Cbma::select_best_lower_individual(const vector<shared_ptr<Solution>>& pop) {
    if (pop.empty()) {
        return nullptr;  // Handle the case where the population is empty
    }

    auto comparator = [](const shared_ptr<Solution>& ind1, const shared_ptr<Solution>& ind2) {
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

vector<double> Cbma::get_fitness_vector_from_upper_group(const vector<shared_ptr<Solution>>& group){
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->upper_cost; });

    return ans;
}

vector<double> Cbma::get_fitness_vector_from_lower_group(const vector<shared_ptr<Solution>>& group) {
    std::vector<double> ans;
    ans.reserve(group.size());  // Reserve space to avoid unnecessary reallocation

    // Use transform along with a lambda to extract fitness values
    std::transform(group.begin(), group.end(), std::back_inserter(ans),
                   [](const auto& ind) { return ind->lower_cost; });

    return ans;
}