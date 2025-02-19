//
// Created by Yinghao Qin on 19/02/2025.
//

#include "gtest/gtest.h"
#include "follower.hpp"
#include "Split.h"
#include "LocalSearch.h"
#include <random>

using namespace ::testing;

class FollowerTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        split = new Split(instance, preprocessor);
        local_search = new LocalSearch(instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete split;
        delete local_search;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Split* split{};
    LocalSearch* local_search;
};

TEST_F(FollowerTest, LoadIndividual) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), preprocessor->random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);
    local_search->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);

    Follower follower(instance, preprocessor);
    follower.load_individual(&ind);

    EXPECT_EQ(follower.num_routes, ind.upper_cost.nb_routes);
    EXPECT_EQ(follower.lower_cost, 0);
    EXPECT_EQ(follower.lower_num_nodes_per_route[0], ind.chromR[0].size() + 2);
    EXPECT_EQ(follower.lower_routes[0][1], ind.chromR[0][0]);
}

TEST_F(FollowerTest, Run) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), preprocessor->random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);
    local_search->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);


    Follower follower(instance, preprocessor);
    follower.run(&ind);

    EXPECT_NE(ind.lower_cost, ind.upper_cost.penalised_cost);
}