//
// Created by Yinghao Qin on 18/02/2025.
//

#include "gtest/gtest.h"
#include "individual.hpp"
#include <random>

using namespace ::testing;

class IndividualTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(file_name_);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        rng = std::mt19937(params->seed);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    std::mt19937 rng;
};


TEST_F(IndividualTest, Constructor) {
    SCOPED_TRACE("Creating an individual...");

    Individual ind(instance, preprocessor);


    EXPECT_EQ(ind.route_cap, preprocessor->route_cap_);
    EXPECT_EQ(ind.node_cap, preprocessor->node_cap_);
}
