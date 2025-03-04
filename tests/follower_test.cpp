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

TEST_F(FollowerTest, SpecialCase_En23k3) {
    string file_name = "E-n23-k3.evrp";
    Case instance_E23(file_name);
    Preprocessor preprocessor_E23(instance_E23, *params);
    Split split_E23(params->seed, &instance_E23, &preprocessor_E23);
    LeaderLahc leader_E23(params->seed, &instance_E23, &preprocessor_E23);
    Follower follower_E23(&instance_E23, &preprocessor_E23);


    vector<int> chromT = vector<int>{12, 8, 5, 21, 15, 2, 19, 18, 7, 6, 1, 3, 16, 20, 22, 17, 14, 11, 13, 9, 4, 10};

    Individual ind(&instance_E23, &preprocessor_E23, chromT);
    ind.chromR[0] = {12, 8, 5, 21, 15, 2, 19, 18, 7, 6, 1, 3, 16, 20, 22, 17, 14, 11};
    ind.chromR[1] = {13, 9, 4};
    ind.chromR[2] = {10};

    double upper_cost = instance_E23.calculate_total_dist(ind.chromR);
//    cout << ind << endl;
//    cout << upper_cost << endl;
//    cout << "Max vehicle capacity: " << instance_E23.max_vehicle_capa_ << endl;
//    cout << instance_E23.calculate_demand_sum(ind.chromR[0]) << endl;
//    cout << instance_E23.calculate_demand_sum(ind.chromR[1]) << endl;
//    cout << instance_E23.calculate_demand_sum(ind.chromR[2]) << endl;

    EXPECT_NEAR(upper_cost, 935.405, 0.001);
    EXPECT_LT(instance_E23.calculate_demand_sum(ind.chromR[0]), instance_E23.max_vehicle_capa_);
    EXPECT_LT(instance_E23.calculate_demand_sum(ind.chromR[1]), instance_E23.max_vehicle_capa_);
    EXPECT_LT(instance_E23.calculate_demand_sum(ind.chromR[2]), instance_E23.max_vehicle_capa_);
}