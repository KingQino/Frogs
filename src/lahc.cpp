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

}

void Lahc::close_log_for_evolution() {

}

void Lahc::flush_row_into_evol_log() {

}

void Lahc::save_log_for_solution() {

}


