//
// Created by Yinghao Qin on 21/02/2025.
//

#include "gtest/gtest.h"
#include "lahc.hpp"

using namespace ::testing;

class LahcTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        params->enable_logging = true;
        params->stop_criteria = 0;
        preprocessor = new Preprocessor(*instance, *params);

        lahc = new Lahc(preprocessor->params.seed, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete lahc;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Lahc* lahc{};
};


TEST_F(LahcTest, InitializeHeuristic) {
    lahc->initialize_heuristic();
    int num_routes = 0;
    int num_nodes = 0;
    for (const auto & i : lahc->current->chromR) {
        if (!i.empty()) {
            num_routes++;
            num_nodes += i.size();
        }
    }

    EXPECT_EQ(lahc->iter, 0L);
    EXPECT_EQ(lahc->current->upper_cost.penalised_cost, lahc->history_list[0]);
    EXPECT_EQ(lahc->current->upper_cost.nb_routes, num_routes);
    EXPECT_EQ(lahc->current->chromT.size(), num_nodes);
}

TEST_F(LahcTest, RunHeuristic) {
    lahc->initialize_heuristic();

//    cout << "Before run_heuristic: " << endl;
//    cout << *lahc->current << endl;

    lahc->run_heuristic();
//    cout << "Iter: " <<  lahc->iter << endl << endl;

//    cout << "After run_heuristic: " << endl;
//    cout << *lahc->current << endl;
//    cout << *lahc->follower << endl;

//    cout << "Global best: " << endl;
//    cout << *lahc->global_best << endl;

    lahc->follower->run(lahc->global_best.get());
//    cout << *lahc->follower << endl;
    EXPECT_DOUBLE_EQ(lahc->global_best->lower_cost, instance->calculate_total_dist_follower(
            lahc->follower->lower_routes, lahc->follower->num_routes, lahc->follower->lower_num_nodes_per_route));
}

TEST_F(LahcTest, Run) {

//    lahc->run();
    EXPECT_TRUE(true);
}