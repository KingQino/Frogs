//
// Created by Yinghao Qin on 21/02/2025.
//

#include "gtest/gtest.h"
#include "Split.h"
#include "leader_lahc.hpp"
#include <random>
#include <memory>  // Include for smart pointers

using namespace ::testing;

class LeaderLahcTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        random_engine = std::mt19937 (params->seed);
        split = new Split(random_engine, instance, preprocessor);
        leader = new LeaderLahc(random_engine, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete split;
        delete leader;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Split* split{};
    LeaderLahc* leader{};
    std::mt19937 random_engine;
};

TEST_F(LeaderLahcTest, Run) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    UpperCost upper_cost_prev = ind.upper_cost;

    leader->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);


    double ground_truth_dis = instance->calculate_total_dist(ind.chromR);

    EXPECT_GT(upper_cost_prev.penalised_cost, ind.upper_cost.penalised_cost);
    EXPECT_GT(upper_cost_prev.nb_routes, ind.upper_cost.nb_routes);
    EXPECT_DOUBLE_EQ(ind.upper_cost.distance, ground_truth_dis);
}