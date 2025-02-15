//
// Created by Yinghao Qin on 15/02/2025.
//

#include "gtest/gtest.h"
#include "preprocessor.hpp"

using namespace ::testing;

class PreprocessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(file_name_);
        params = new Parameters();

    }

    void TearDown() override {
        delete instance;
        delete params;
    }

    Case* instance{};
    Parameters* params{};
};

TEST_F(PreprocessorTest, Construtor) {
    SCOPED_TRACE("Preprocessing...");

    Preprocessor preprocessor(*instance, *params);

    EXPECT_FALSE(preprocessor.correlated_vertices_.empty());
    EXPECT_FALSE(preprocessor.best_stations_.empty());
}

