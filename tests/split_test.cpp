//
// Created by Yinghao Qin on 07/03/2025.
//

#include "gtest/gtest.h"
#include "individual.hpp"
#include "Split.h"
#include "leader_array.hpp"
#include "follower.hpp"
#include <random>
#include <memory>

using namespace ::testing;

class SplitTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n30-k3.evrp"; // this instance is special with some properties
        instance = new Case(file_name);
        params = new Parameters();
        preprocessor = new Preprocessor(*instance, *params);
        split = new Split(params->seed, instance, preprocessor);
        leader = new LeaderArray(params->seed, instance, preprocessor);
        follower = new Follower(instance, preprocessor);
        rng = std::default_random_engine(params->seed);
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
    LeaderArray* leader{};
    Follower* follower{};
    std::default_random_engine rng;
};


TEST_F(SplitTest, InitIndividualWithDirectEncoding) {
    Individual ind(instance, preprocessor);

//    cout << ind << endl;

    split->initIndividualWithDirectEncoding(&ind);
//    cout << ind << endl;
//    cout << ind.chromT.size() << endl;
    vector<int> all_elements;
    for (int i = 0; i < ind.upper_cost.nb_routes; ++i) {
        all_elements.insert(all_elements.end(), ind.chromR[i].begin(), ind.chromR[i].end());
    }
    sort(all_elements.begin(), all_elements.end());
//    cout << all_elements.size() << endl;
//    for (auto node:all_elements) {
//        cout << node << " ";
//    }
//    cout << endl;

    EXPECT_TRUE(ind.is_upper_feasible);
    EXPECT_EQ(ind.chromT.size(), instance->num_customer_);
    EXPECT_EQ(ind.chromT[0], ind.chromR[0][0]);
    EXPECT_DOUBLE_EQ(ind.upper_cost.penalised_cost, ind.upper_cost.distance);
}

TEST_F(SplitTest, Debug_En30_Init) {
    // 1'000'000
    for (int i = 0; i < 1'000; ++i) {
        Individual ind(instance, preprocessor);

        split->initIndividualWithHienClustering(&ind);
        for (int j = 0; j < ind.upper_cost.nb_routes; ++j) {
            EXPECT_GT(ind.chromR[j].size(), 0);
        }
        vector<int> all_elements;
        for (int j = 0; j < ind.upper_cost.nb_routes; ++j) {
            all_elements.insert(all_elements.end(), ind.chromR[j].begin(), ind.chromR[j].end());
        }

        EXPECT_GT(ind.upper_cost.nb_routes, 2);
        EXPECT_EQ(all_elements.size(), 29);
    }
}

TEST_F(SplitTest, Debug_En30_NeighborExplore) {
    Individual ind(instance, preprocessor);
    split->initIndividualWithHienClustering(&ind);
//    cout << ind << endl;

    leader->load_individual(&ind);

    for (int i = 0; i < 1'000; ++i) {
        leader->neighbour_explore(2'000.0);
        leader->export_individual(&ind);

        vector<int> all_elements;
        for (int j = 0; j < ind.upper_cost.nb_routes; ++j) {
            all_elements.insert(all_elements.end(), ind.chromR[j].begin(), ind.chromR[j].end());
        }
        sort(all_elements.begin(), all_elements.end());

        for (int j = 0; j < leader->num_routes; ++j) {
//            if (leader->num_nodes_per_route[j] <= 2) {
//                cout << "Route " << j << " has less than 2 nodes" << endl;
//                cout << *leader << endl;
//                cout << ind << endl;
//                cout << "AAA" << endl;
//            }
            EXPECT_GT(leader->num_nodes_per_route[j], 2);
        }
        EXPECT_GT(ind.upper_cost.nb_routes, 2);
        EXPECT_EQ(all_elements.size(), 29);
        EXPECT_EQ(all_elements, preprocessor->customer_ids_);
    }
}

TEST_F(SplitTest, Debug_En30_FollowerRun) {
    Individual ind(instance, preprocessor);
    split->initIndividualWithHienClustering(&ind);

    leader->load_individual(&ind);

    for (int i = 0; i < 1'000; ++i) {
        leader->neighbour_explore(1000.0);
        leader->export_individual(&ind);

        follower->run(&ind);

        EXPECT_GT(follower->num_routes, 2);
        EXPECT_EQ(follower->num_routes, ind.upper_cost.nb_routes);

        vector<int> all_elements;
        for (int j = 0; j < follower->num_routes; ++j) {
            all_elements.insert(all_elements.end(), follower->lower_routes[j] + 1, follower->lower_routes[j] + follower->lower_num_nodes_per_route[j] - 1);
        }
        all_elements.erase(
                std::remove_if(all_elements.begin(), all_elements.end(),
                               [&](int node) { return instance->is_charging_station(node); }),
                all_elements.end()
        );
        sort(all_elements.begin(), all_elements.end());


        EXPECT_EQ(all_elements.size(), 29);
        EXPECT_EQ(all_elements, preprocessor->customer_ids_);
    }
}

TEST_F(SplitTest, Debug_En30_Specific_Case) {
    leader->num_routes = 3;
    leader->upper_cost = 538.795;
    leader->num_nodes_per_route[0] = 14;
    leader->num_nodes_per_route[1] = 13;
    leader->num_nodes_per_route[2] = 8;
    leader->demand_sum_per_route[0] = 4425;
    leader->demand_sum_per_route[1] = 4400;
    leader->demand_sum_per_route[2] = 3925;
    vector<int> route0 = {0, 21, 14, 8, 9, 17, 7, 13, 16, 12, 11, 10, 23, 0};
    vector<int> route1 = {0, 18, 15, 26, 28, 27, 29, 25, 24, 1, 6, 19, 0};
    vector<int> route2 = {0, 22, 2, 5, 4, 3, 20, 0}; //0 22 2 5 4 3 20 0 24 1 6 19 0
    memcpy(leader->routes[0], route0.data(), route0.size() * sizeof(int));
    memcpy(leader->routes[1], route1.data(), route1.size() * sizeof(int));
    memcpy(leader->routes[2], route2.data(), route2.size() * sizeof(int));


//    int total_demands = 0;
//    for (int i = 0; i < 1'000; ++i) {
//        leader->neighbour_explore(1760);
//        total_demands = 0;
//        for (int j = 0; j < leader->num_routes; ++j) {
//            total_demands += leader->demand_sum_per_route[j];
//        }
//    }

    EXPECT_TRUE(true);
}