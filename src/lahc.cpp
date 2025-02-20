//
// Created by Yinghao Qin on 19/02/2025.
//

#include "lahc.hpp"

const std::string ALGORITHM = "Lahc";

Lahc::Lahc(Case* instance, Preprocessor* preprocessor): HeuristicInterface("LAHC", instance, preprocessor) {
    iter = 0;
    idle_iter = 0;
    history_list = new double[preprocessor->params.history_length];
}

Lahc::~Lahc() {
    delete[] this->history_list;
}

void Lahc::initialize_heuristic() {

}

void Lahc::run_heuristic() {

}

void Lahc::run() {

}

void Lahc::open_log_for_evolution() {
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(preprocessor->params.seed);
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
    const string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(preprocessor->params.seed);

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


