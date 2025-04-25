//
// Created by Yinghao Qin on 19/02/2025.
//

#include "gtest/gtest.h"
#include "follower.hpp"
#include <random>

using namespace ::testing;

class FollowerTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        random_engine = std::mt19937(params->seed);
        follower = new Follower(instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete follower;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Follower* follower{};
    std::mt19937 random_engine;
};
