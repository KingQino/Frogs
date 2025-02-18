//
// Created by Yinghao Qin on 18/02/2025.
//

#include "gtest/gtest.h"
#include "individual.hpp"
#include<random>

using namespace ::testing;

class IndividualTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(file_name_);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        rng = std::default_random_engine(0);;
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
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

//TEST_F(IndividualTest, ConstructorWithChromTAndChromR) {
//    SCOPED_TRACE("Creating an individual with chromT and chromR...");
//
//    vector<int> chromT(preprocessor->customer_ids_);
//    std::shuffle(chromT.begin(), chromT.end(), rng);
//    vector<vector<int>> chromR(preprocessor->route_cap_, vector<int>());
//    for (int i = 0; i < chromR.size(); ++i) {
//        chromR[i] = chromT;
//    }
//    Individual ind(instance, preprocessor, chromT, chromR, 0);
//
//    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
//    EXPECT_EQ(ind.chromR.size(), preprocessor->route_cap_);
//    EXPECT_EQ(ind.successors.size(), instance->num_customer_ + 1);
//    EXPECT_EQ(ind.predecessors.size(), instance->num_customer_ + 1);
//}
