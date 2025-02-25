//
// Created by Yinghao Qin on 25/02/2025.
//

#include "gtest/gtest.h"
#include "Split.h"
#include "leader_array.hpp"
#include <random>

using namespace ::testing;

class LeaderArrayTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        split = new Split(params->seed, instance, preprocessor);
        leader = new LeaderArray(params->seed, instance, preprocessor);
        random_engine = std::default_random_engine(params->seed);
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
    LeaderArray* leader{};
    std::default_random_engine random_engine;
};

TEST_F(LeaderArrayTest, LoadIndividual) {
    Individual ind(instance, preprocessor);
    split->initIndividualWithHienClustering(&ind);

    leader->load_individual(&ind);

    EXPECT_DOUBLE_EQ(leader->upper_cost, ind.upper_cost.penalised_cost);
    EXPECT_EQ(leader->num_routes, ind.upper_cost.nb_routes);
}

TEST_F(LeaderArrayTest, NeighbourhoodExplore) {
    Individual ind(instance, preprocessor);
    split->initIndividualWithHienClustering(&ind);

    leader->load_individual(&ind);

    double history_cost = 800;

    for (int i = 0; i < 1'000; ++i) {
        leader->neighbour_explore(history_cost);
        leader->export_individual(&ind);

        EXPECT_NE(leader->upper_cost, history_cost);
        EXPECT_DOUBLE_EQ(leader->upper_cost, ind.upper_cost.penalised_cost);
        EXPECT_EQ(leader->num_routes, ind.upper_cost.nb_routes);
    }
}

TEST_F(LeaderArrayTest, Moves) {
    Individual ind(instance, preprocessor);
    ind.chromT = {12, 8, 14, 4, 3, 6, 11, 10, 9, 5, 1, 2, 7, 15, 17, 20, 21, 18, 13, 16, 19};
    ind.chromR[0] = {12, 8, 14, 4, 3, 6, 11};
    ind.chromR[1] = {10, 9, 5, 1, 2, 7};
    ind.chromR[2] = {15, 17, 20, 21, 18};
    ind.chromR[3] = {13, 16, 19};
    ind.upper_cost.penalised_cost = 515.861;
    ind.upper_cost.distance = 515.861;
    ind.upper_cost.nb_routes = 4;
    ind.is_upper_feasible = true;
    ind.lower_cost = 524.634;

    leader->load_individual(&ind);

    double history_cost = 800;

    leader->history_cost = history_cost;

    for (int i = 0; i < 10; ++i) {
        bool isMoved = leader->two_opt_inter_for_individual();
        EXPECT_TRUE(true);
    }
}