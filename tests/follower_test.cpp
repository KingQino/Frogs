//
// Created by Yinghao Qin on 19/02/2025.
//

#include "gtest/gtest.h"
#include "follower.hpp"
#include "Split.h"
//#include "LocalSearch.h"
#include "leader_lahc.hpp"
#include <random>

using namespace ::testing;

class FollowerTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        split = new Split(params->seed, instance, preprocessor);
        leader = new LeaderLahc(params->seed, instance, preprocessor);
        follower = new Follower(instance, preprocessor);
        random_engine = std::default_random_engine(params->seed);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete split;
        delete leader;
        delete follower;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Split* split{};
    LeaderLahc* leader{};
    Follower* follower{};
    std::default_random_engine random_engine;
};

TEST_F(FollowerTest, LoadIndividual) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);
    leader->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);

    follower->load_individual(&ind);

    EXPECT_EQ(follower->num_routes, ind.upper_cost.nb_routes);
    EXPECT_EQ(follower->lower_cost, 0);
    EXPECT_EQ(follower->lower_num_nodes_per_route[0], ind.chromR[0].size() + 2);
    EXPECT_EQ(follower->lower_routes[0][1], ind.chromR[0][0]);
}

TEST_F(FollowerTest, Run) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);
    leader->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);

    follower->run(&ind);

    EXPECT_NE(ind.lower_cost, ind.upper_cost.penalised_cost);
    EXPECT_DOUBLE_EQ(ind.lower_cost, instance->calculate_total_dist_follower(follower->lower_routes, follower->num_routes, follower->lower_num_nodes_per_route));
}

TEST_F(FollowerTest, Refine) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);
    leader->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);

    follower->refine(&ind);

    EXPECT_NE(ind.lower_cost, ind.upper_cost.penalised_cost);
    EXPECT_DOUBLE_EQ(ind.lower_cost, instance->calculate_total_dist_follower(follower->lower_routes, follower->num_routes, follower->lower_num_nodes_per_route));
}

TEST_F(FollowerTest, ConsecutiveRun) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, instance->calculate_total_dist(ind.chromR));

    double historyVal = 800;
    leader->loadIndividual(&ind);
    int length = 10000;
    int num_infeasible = 0;
    for (int i = 0; i < length; i++) {
        leader->neighbourExplore(historyVal);
        leader->exportChromosome(&ind);

        historyVal = leader->getUpperCost() * 1.1;
        EXPECT_NEAR(leader->getUpperCost(), instance->calculate_total_dist(ind.chromR), 0.000'001);

        follower->run(&ind);

        if (follower->lower_cost >= INFEASIBLE) {
            num_infeasible++;
        } else {
            EXPECT_NEAR(ind.lower_cost, instance->calculate_total_dist_follower(follower->lower_routes, follower->num_routes, follower->lower_num_nodes_per_route), 0.000'001);
        }
//        cout << "Upper Cost: " << leader->getUpperCost() << " | Ground Truth: " << instance->calculate_total_dist(ind.chromR)
//        <<  " | Lower Cost: " <<  follower->lower_cost  << " | Ground Truth: "
//        << instance->calculate_total_dist_follower(follower->lower_routes, follower->num_routes, follower->lower_num_nodes_per_route) << endl;
    }
}