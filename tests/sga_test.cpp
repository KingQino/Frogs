//
// Created by Yinghao Qin on 13/04/2025.
//

#include "gtest/gtest.h"
#include "sga.hpp"

using namespace ::testing;

class SgaTest : public Test {
protected:
    void SetUp() override {
//        string file_name = "X-n916-k207.evrp";
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        params->enable_logging = true;
        params->stop_criteria = 0;
        preprocessor = new Preprocessor(*instance, *params);
        sga = new Sga(preprocessor->params.seed, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete sga;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Sga* sga{};
};

TEST_F(SgaTest, Intensification) {
    shared_ptr<Individual> ind_ptr = make_shared<Individual>(instance, preprocessor);


}

TEST_F(SgaTest, IntensificationPlusNeighbourhoodExploration) {
    unique_ptr<Individual> global_best = make_unique<Individual>();

    vector<vector<int>> routes = sga->initializer->routes_constructor_with_split();
    Individual ind(instance, preprocessor, routes, instance->compute_total_distance(routes), instance->compute_demand_sum_per_route(routes));


    sga->leader->local_improve(&ind);
    sga->follower->run(&ind);

    if (ind.lower_cost < global_best->lower_cost) {
        global_best = make_unique<Individual>(ind);
//        cout << "Current global best: " << endl << *global_best << endl;
    }

    double boarder = ind.upper_cost * 1.1;
    auto* partial_sol = new PartialSolution();

    // for loop for neighbour exploration
    bool has_moved;
    for (int i = 0; i < 1'000; ++i) {
        has_moved = sga->leader->neighbour_explore(boarder, partial_sol);
        if (has_moved) {
            sga->follower->run(partial_sol);

            sga->leader->export_individual(&ind);
            sga->follower->export_individual(&ind);

            // update the global best lower-level solution
            if (ind.lower_cost < global_best->lower_cost) {
                global_best = make_unique<Individual>(ind);

//                cout << "New global best found: " << endl << *global_best << endl;
            }


        }
    }


    delete partial_sol;

    EXPECT_TRUE(true);
}

TEST_F(SgaTest, Run) {
//    sga->initialize_heuristic();
//    sga->run_heuristic();

    sga->run();

    EXPECT_TRUE(true);

//    cout << "Best solution found: " << endl << *sga->global_best << endl;
}