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
        std::string file_name = "E-n22-k4.evrp";

        instance = std::make_unique<Case>(file_name);
        if (!instance) {
            FAIL() << "Error: Failed to initialize Case instance!";
        }

        params = std::make_unique<Parameters>();
        preprocessor = std::make_unique<Preprocessor>(*instance, *params);
        split = std::make_unique<Split>(params->seed, instance.get(), preprocessor.get());
        leader = std::make_unique<LeaderLahc>(params->seed, instance.get(), preprocessor.get());

        // Initialize random engine properly
        random_engine.seed(params->seed);

//        string file_name = "E-n22-k4.evrp";
//        instance = new Case(file_name);
//        params = new Parameters();
//        preprocessor = new Preprocessor(*instance, *params);
//        split = new Split(params->seed, instance, preprocessor);
//        leader = new LeaderLahc(params->seed, instance, preprocessor);
//        random_engine = std::default_random_engine(params->seed);
    }

    void TearDown() override {
//        delete instance;
//        delete params;
//        delete preprocessor;
//        delete split;
//        delete leader;
    }

    // Use std::unique_ptr instead of raw pointers
    std::unique_ptr<Case> instance;
    std::unique_ptr<Parameters> params;
    std::unique_ptr<Preprocessor> preprocessor;
    std::unique_ptr<Split> split;
    std::unique_ptr<LeaderLahc> leader;
    std::default_random_engine random_engine;

//    Case* instance{};
//    Parameters* params{};
//    Preprocessor* preprocessor{};
//    Split* split{};
//    LeaderLahc* leader{};
//    std::default_random_engine random_engine;
};

TEST_F(LeaderLahcTest, Run) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance.get(), preprocessor.get(), chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    UpperCost upper_cost_prev = ind.upper_cost;

    leader->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);


    double ground_truth_dis = instance->calculate_total_dist(ind.chromR);

    EXPECT_GT(upper_cost_prev.penalised_cost, ind.upper_cost.penalised_cost);
    EXPECT_GT(upper_cost_prev.nb_routes, ind.upper_cost.nb_routes);
    EXPECT_DOUBLE_EQ(ind.upper_cost.distance, ground_truth_dis);
}

TEST_F(LeaderLahcTest, IntraRouteMoves) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance.get(), preprocessor.get(), chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, instance->calculate_total_dist(ind.chromR));

    leader->loadIndividual(&ind);

    for (int i = 0; i < 1000; ++i) {
        leader->nodeU = &leader->clients[leader->getRandomCustomerNodeU()];
        leader->setLocalVariablesRouteU();
        if (leader->routeU->nbCustomers <= 2) continue;
        while (true) {
            leader->nodeV = &leader->clients[leader->getRandomCorrelatedNodeV(leader->nodeU->cour)];
            leader->setLocalVariablesRouteV();
            if (leader->routeU == leader->routeV) break;
        }

//        bool isMoved = leader->move1_intra();
//        bool isMoved = leader->move4_intra();
        bool isMoved = leader->move7_intra();

//        leader->exportIndividual(&ind);
        leader->exportChromosome(&ind);

//        cout << "Is Moved: " << isMoved << " | Upper Cost: " << leader->getUpperCost() << " | Ground Truth Cost: " <<  instance->calculate_total_dist(ind.chromR) << endl;
        leader->historyCost = leader->getUpperCost() * 1.1;

        EXPECT_NEAR(leader->getUpperCost(), instance->calculate_total_dist(ind.chromR), 0.000'001);
    }
}

TEST_F(LeaderLahcTest, InterRouteMoves) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance.get(), preprocessor.get(), chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, instance->calculate_total_dist(ind.chromR));

    leader->loadIndividual(&ind);

    for (int i = 0; i < 1000; ++i) {
        leader->nodeU = &leader->clients[leader->getRandomCustomerNodeU()];
        leader->setLocalVariablesRouteU();
        while (true) {
            leader->nodeV = &leader->clients[leader->getRandomCorrelatedNodeV(leader->nodeU->cour)];
            leader->setLocalVariablesRouteV();
            if (leader->routeU != leader->routeV) break;
        }


//        bool isMoved = leader->move1_inter();
//        bool isMoved = leader->move4_inter();
//        bool isMoved = leader->move8_inter();
        bool isMoved = leader->move9_inter();


        leader->exportChromosome(&ind);

//        cout << "Is Moved: " << isMoved << " | Upper Cost: " << leader->getUpperCost() << " | Ground Truth Cost: " <<  instance->calculate_total_dist(ind.chromR) << endl;
        leader->historyCost = leader->getUpperCost() * 1.1;

        EXPECT_NEAR(leader->getUpperCost(), instance->calculate_total_dist(ind.chromR), 0.000'001);
    }
}

TEST_F(LeaderLahcTest, NeighbourExplore) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), random_engine);
    Individual ind(instance.get(), preprocessor.get(), chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, instance->calculate_total_dist(ind.chromR));

    double historyVal = 800;
    leader->loadIndividual(&ind);
    int length = 1000;
    for (int i = 0; i < length; i++) {
        leader->neighbourExplore(historyVal);
        leader->exportChromosome(&ind);

//        cout << "Upper Cost: " << leader->getUpperCost() << " | Ground Truth Cost: " <<  instance->calculate_total_dist(ind.chromR) << endl;
        historyVal = leader->getUpperCost() * 1.1;
        EXPECT_NEAR(leader->getUpperCost(), instance->calculate_total_dist(ind.chromR), 0.000'001);
    }

//    cout << leader->nbMoves << endl;
//    cout << "Hit Move Ratio: " << leader->nbMoves / static_cast<double>(length) << endl;
}