//
// Created by Yinghao Qin on 18/02/2025.
//

#include "gtest/gtest.h"
#include "individual.hpp"
#include "Split.h"
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
        rng = std::default_random_engine(0);;
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete split;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Split* split{};
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


    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromR.size(), preprocessor->route_cap_);
    EXPECT_EQ(ind.successors.size(), instance->num_customer_ + 1);
    EXPECT_EQ(ind.predecessors.size(), instance->num_customer_ + 1);
    EXPECT_NE(ind.chromR[preprocessor->route_cap_ - 1].size(), 0);
    EXPECT_EQ(ind.chromR[preprocessor->route_cap_ - routes.size()], routes[routes.size() - 1]);
}
