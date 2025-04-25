//
// Created by Yinghao Qin on 14/04/2025.
//

#include "gtest/gtest.h"
#include "leader_sga.hpp"
#include "initializer.hpp"
#include <random>

using namespace ::testing;

class LeaderSgaTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        random_engine = std::mt19937(params->seed);
        initializer = new Initializer(random_engine, instance, preprocessor);
        leader_sga = new LeaderSga(random_engine, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete initializer;
        delete leader_sga;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Initializer* initializer{};
    LeaderSga* leader_sga{};
    std::mt19937 random_engine;
};


TEST_F(LeaderSgaTest, PerformInterMoveImpro) {
    vector<vector<int>> routes = initializer->routes_constructor_with_split();
    Individual ind(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));

    leader_sga->load_individual(&ind);

    bool has_moved = false;
    has_moved = leader_sga->perform_intra_move_impro([&](int* route, int length)
            {return leader_sga->move1_intra_impro(route, length);});
    has_moved = leader_sga->perform_inter_move_impro([this](int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2)
            {return leader_sga->move1_inter_impro(route1, route2, length1, length2, loading1, loading2);});
    has_moved = leader_sga->perform_intra_move_impro([&](int* route, int length)
            {return leader_sga->move4_intra_impro(route, length);});

    leader_sga->export_individual(&ind);

    EXPECT_DOUBLE_EQ(leader_sga->upper_cost, ind.upper_cost);
    EXPECT_EQ(leader_sga->num_routes, ind.num_routes);
}

TEST_F(LeaderSgaTest, LocalImprove) {
    for (int i = 0; i < 1'000; ++i) {

        vector<vector<int>> routes = initializer->routes_constructor_with_split();
        Individual ind(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));
        double cost_prev = ind.upper_cost;
        leader_sga->local_improve(&ind);
        double cost_after = ind.upper_cost;

        EXPECT_LT(cost_after, cost_prev);
    }
}