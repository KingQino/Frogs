//
// Created by Yinghao Qin on 08/04/2025.
//

#include "gtest/gtest.h"
#include "cbma.hpp"

using namespace ::testing;

class CbmaTest : public Test {
protected:
    void SetUp() override {
        string file_name = "E-n22-k4.evrp";
        instance = new Case(file_name);
        params = new Parameters();
        params->enable_logging = true;
        params->stop_criteria = 0;
        preprocessor = new Preprocessor(*instance, *params);

        cbma = new Cbma(preprocessor->params.seed, instance, preprocessor);
    }

    void TearDown() override {
        delete instance;
        delete params;
        delete preprocessor;
        delete cbma;
    }

    Case* instance{};
    Parameters* params{};
    Preprocessor* preprocessor{};
    Cbma* cbma{};
};

TEST_F(CbmaTest, Run) {
    cbma->run();

}