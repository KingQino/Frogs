//
// Created by Yinghao Qin on 11/02/2025.
//

#include "gtest/gtest.h"
#include "case.hpp"

using namespace ::testing;

class CaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(file_name_);
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
};


TEST_F(CaseTest, Construtor) {
    SCOPED_TRACE("Case constructing...");

    EXPECT_EQ(instance->depot_, 0);
    EXPECT_EQ(instance->num_customer_, 21);
    EXPECT_EQ(instance->num_station_, 8);
    EXPECT_EQ(instance->problem_size_, 30);
    EXPECT_EQ(instance->get_customer_demand_(2), 700);
    EXPECT_TRUE(instance->is_charging_station(0));
    EXPECT_TRUE(instance->is_charging_station(22));
    EXPECT_TRUE(instance->is_charging_station(29));
    EXPECT_FALSE(instance->is_charging_station(13));
    EXPECT_FALSE(instance->is_charging_station(30));
}