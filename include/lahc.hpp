//
// Created by Yinghao Qin on 19/02/2025.
//

#ifndef FROGS_LAHC_HPP
#define FROGS_LAHC_HPP

#include "case.hpp"
#include "individual.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Lahc : public HeuristicInterface, public StatsInterface {
private:
    static const std::string ALGORITHM;
    Case* instance;
    Preprocessor* preprocessor;

public:
    Lahc(Case *instance, Preprocessor* preprocessor);
    ~Lahc() override;

};

#endif //FROGS_LAHC_HPP
