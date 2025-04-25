//
// Created by Yinghao Qin on 25/02/2025.
//

#include "gtest/gtest.h"
#include "leader_array.hpp"
#include "initializer.hpp"
#include <random>

using namespace ::testing;

class LeaderArrayTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        random_engine = std::mt19937(params->seed);
        initializer = new Initializer(random_engine, instance, preprocessor);
        leader = new LeaderArray(random_engine, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete initializer;
        delete leader;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Initializer* initializer{};
    LeaderArray* leader{};
    std::mt19937 random_engine;
};

TEST_F(LeaderArrayTest, LoadIndividual) {
    vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
    Individual ind(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));

    leader->load_individual(&ind);

    EXPECT_DOUBLE_EQ(leader->upper_cost, ind.upper_cost);
    EXPECT_EQ(leader->num_routes, ind.num_routes);
}

TEST_F(LeaderArrayTest, NeighbourhoodExplore) {
    vector<vector<int>> routes = initializer->routes_constructor_with_hien_method();
    Individual ind(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));

    leader->load_individual(&ind);

    double history_cost = 800;

    for (int i = 0; i < 1'000; ++i) {
        leader->neighbour_explore(history_cost);
        leader->export_individual(&ind);

        EXPECT_NE(leader->upper_cost, history_cost);
        EXPECT_DOUBLE_EQ(leader->upper_cost, ind.upper_cost);
        EXPECT_EQ(leader->num_routes, ind.num_routes);
    }
}
