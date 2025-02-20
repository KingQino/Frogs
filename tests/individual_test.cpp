//
// Created by Yinghao Qin on 18/02/2025.
//

#include "gtest/gtest.h"
#include "individual.hpp"
#include "Split.h"
#include "LocalSearch.h"
#include <random>

using namespace ::testing;

class IndividualTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(file_name_);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        split = new Split(instance, preprocessor);
        local_search = new LocalSearch(instance, preprocessor);
        rng = std::default_random_engine(preprocessor->params.seed);
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
    LocalSearch* local_search{};
    std::default_random_engine rng;
};


TEST_F(IndividualTest, Constructor) {
    SCOPED_TRACE("Creating an individual...");

    Individual ind(instance, preprocessor);


    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromR.size(), preprocessor->route_cap_);
    EXPECT_EQ(ind.successors.size(), instance->num_customer_ + 1);
    EXPECT_EQ(ind.predecessors.size(), instance->num_customer_ + 1);
}

TEST_F(IndividualTest, ConstructorWithChromT) {
    SCOPED_TRACE("Creating an individual with chromT...");

    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), rng);
    Individual ind(instance, preprocessor, chromT);

    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromR.size(), preprocessor->route_cap_);
    EXPECT_EQ(ind.successors.size(), instance->num_customer_ + 1);
    EXPECT_EQ(ind.predecessors.size(), instance->num_customer_ + 1);
}

TEST_F(IndividualTest, ConstructorWithChromTAndChromR) {
    SCOPED_TRACE("Creating an individual with chromT and chromR...");

    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), rng);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    // Prins split - O(nB) Ground truth
    vector<vector<int>> routes = split->prinsSplit(chromT);

    EXPECT_EQ(ind.upper_cost.nb_routes, routes.size());
    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromR.size(), preprocessor->route_cap_);
    EXPECT_EQ(ind.successors.size(), instance->num_customer_ + 1);
    EXPECT_EQ(ind.predecessors.size(), instance->num_customer_ + 1);
    EXPECT_NE(ind.chromR[preprocessor->route_cap_ - 1].size(), 0);
    EXPECT_EQ(ind.chromR[preprocessor->route_cap_ - routes.size()], routes[routes.size() - 1]);
    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, ind.upper_cost.distance);
}

TEST_F(IndividualTest, InitIndividualWithHienClustering) {
    Individual ind(instance, preprocessor);

    split->initIndividualWithHienClustering(&ind);

    EXPECT_TRUE(ind.is_upper_feasible);
    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromT[0], ind.chromR[0][0]);
    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, ind.upper_cost.distance);
}

TEST_F(IndividualTest, InitIndividualWithDirectEncoding) {
    Individual ind(instance, preprocessor);

    split->initIndividualWithDirectEncoding(&ind);

    EXPECT_TRUE(ind.is_upper_feasible);
    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromT[0], ind.chromR[0][0]);
    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, ind.upper_cost.distance);
}

TEST_F(IndividualTest, LocalSearch) {
    vector<int> chromT(preprocessor->customer_ids_);
    std::shuffle(chromT.begin(), chromT.end(), rng);
    Individual ind(instance, preprocessor, chromT);
    split->generalSplit(&ind, preprocessor->route_cap_);

    UpperCost upper_cost_prev = ind.upper_cost;

    local_search->run(&ind, preprocessor->penalty_capacity_, preprocessor->penalty_duration_);

    double ground_truth_dis = instance->calculate_total_dist(ind.chromR);

    EXPECT_GT(upper_cost_prev.penalised_cost, ind.upper_cost.penalised_cost);
    EXPECT_GT(upper_cost_prev.nb_routes, ind.upper_cost.nb_routes);
    EXPECT_DOUBLE_EQ(ind.upper_cost.distance, ground_truth_dis);
}