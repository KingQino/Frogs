//
// Created by Yinghao Qin on 19/02/2025.
//

#include "lahc.hpp"

const std::string ALGORITHM = "Lahc";

Lahc::Lahc(int seed_val, Case* instance, Preprocessor* preprocessor) : HeuristicInterface("LAHC", seed_val, instance, preprocessor) {
    enable_logging = preprocessor->params.enable_logging;
    stop_criteria = preprocessor->params.stop_criteria;

    iter = 0;
    idle_iter = 0;
    history_length = preprocessor->params.history_length;
    history_list = vector<double>(history_length);
    current = nullptr;
    global_best = make_unique<Individual>();

    split = new Split(seed_val, instance, preprocessor);
    leader = new LeaderLahc(seed_val, instance, preprocessor);
    follower = new Follower(instance, preprocessor);
}

Lahc::~Lahc() {
    delete split;
    delete leader;
    delete follower;
    delete current;
}

void Lahc::initialize_heuristic() {
    current = new Individual(instance, preprocessor);

    split->initIndividualWithHienClustering(current);
    for(int i = 0; i < history_length; ++i)
        history_list[i] = current->upper_cost.penalised_cost;
    this->iter = 0;
    this->idle_iter = 0;
}

void Lahc::run_heuristic() {
    do {

        double current_cost = current->upper_cost.penalised_cost;

        auto v = iter % history_length;
        double history_cost = history_list[v];

        leader->loadIndividual(current);
        leader->neighbourExplore(history_cost);
        leader->exportChromosome(current);

        double candidate_cost = leader->upperCost;

        // idle judgement and counting
        idle_iter = candidate_cost >= current_cost ? idle_iter + 1 : 0;
        // update the history list
        history_list[v] = candidate_cost < history_cost ? candidate_cost : history_cost;

        if (v == 0) {
            history_list_metrics = StatsInterface::calculate_statistical_indicators(history_list);
            flush_row_into_evol_log();
        }

        iter++;
//        duration = std::chrono::high_resolution_clock::now() - start;

        follower->run(current);
        if (current->lower_cost < global_best->lower_cost) {
            global_best = std::move(make_unique<Individual>(*current));
        }

//    } while ((iter < 100'000L || idle_iter < static_cast<long>(static_cast<double>(iter) * 0.2)) && duration.count() < preprocessor->max_exec_time_ );
    } while (iter < 100'000L || idle_iter < static_cast<long>(static_cast<double>(iter) * 0.2));
}

void Lahc::run() {
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

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
            }
            break;
        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                run_heuristic();
                duration = std::chrono::high_resolution_clock::now() - start;

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
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


