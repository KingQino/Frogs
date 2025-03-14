//
// Created by Yinghao Qin on 19/02/2025.
//

#include "lahc.hpp"

const std::string ALGORITHM = "Lahc";

Lahc::Lahc(int seed_val, Case* instance, Preprocessor* preprocessor) : HeuristicInterface("LAHC", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    iter = 0L;
    idle_iter = 0L;
    history_length = static_cast<long>(preprocessor->params.history_length);
    history_list = vector<double>(history_length);
    current = nullptr;
    global_best = make_unique<Solution>();

    initializer = new Initializer(seed_val, instance, preprocessor);
//    leader = new LeaderLahc(seed_val, instance, preprocessor);
    leader = new LeaderArray(seed_val, instance, preprocessor);
    follower = new Follower(instance, preprocessor);
}

Lahc::~Lahc() {
    delete initializer;
    delete leader;
    delete follower;
    delete current;
}

void Lahc::initialize_heuristic() {
    delete current;
    vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
    current = new Solution(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));

    history_list.assign(history_length, current->upper_cost);
    this->iter = 0L;
    this->idle_iter = 0L;
}

void Lahc::run_heuristic() {
    leader->load_solution(current);

    do {

        double current_cost = leader->upper_cost;

        auto v = iter % history_length;
        double history_cost = history_list[v];

        leader->neighbour_explore(history_cost);
        // TODO: if neighbour_explore is successful, then we should update the current solution, otherwise, we should keep the current solution
        leader->export_solution(current);
        double candidate_cost = leader->upper_cost;

        // idle judgement and counting
        idle_iter = candidate_cost >= current_cost ? idle_iter + 1 : 0;
        // update the history list
        history_list[v] = candidate_cost < history_cost ? candidate_cost : history_cost;

        if (v == 0L) {
            history_list_metrics = StatsInterface::calculate_statistical_indicators(history_list);
            flush_row_into_evol_log();
        }

        iter++; // TODO: can be integrated into the v variable calculation

        // TODO: if neighbour_explore is successful, then we should make the follower decision, we should keep the current solution
        follower->run(current);
        if (current->lower_cost < global_best->lower_cost) {
            global_best = std::move(make_unique<Solution>(*current));
        }

    } while (iter < 100'000L || idle_iter < iter / 5);
}

void Lahc::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration<double>::zero();

    if (enable_logging) {
        open_log_for_evolution();  // Open log if logging is enabled
    }


    switch (stop_criteria) {
        case 0:
            while (!stop_criteria_max_evals()) {
                initialize_heuristic();
                run_heuristic();
            }
            break;
        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                initialize_heuristic();
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

void Lahc::open_log_for_evolution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    const string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "iters,global_best,min,max,mean,std\n";
}

void Lahc::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Lahc::flush_row_into_evol_log() {
    oss_row_evol << iter << "," << global_best->lower_cost << "," << history_list_metrics.min << "," <<
        history_list_metrics.max <<"," << history_list_metrics.avg << "," << history_list_metrics.std << "\n";
}

void Lahc::save_log_for_solution() {
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


