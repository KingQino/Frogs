//
// Created by Yinghao Qin on 12/02/2025.
//

#include "gtest/gtest.h"
#include "command_line.hpp"

using namespace ::testing;

TEST(CommandLine, ParseParameters) {
    int argc = 13;
    const char* argv[] = {
            "./frogs",
            "-alg", "Lahc",
            "-ins", "large.evrp",
            "-log", "1",
            "-stp", "0",
            "-mth", "0",
            "-nb_granular", "25"
    };


    CommandLine cmd(argc, const_cast<char**>(argv));
    Parameters params;
    cmd.parse_parameters(params);

    EXPECT_EQ(params.algorithm, Algorithm::Lahc);
    EXPECT_EQ(params.instance, "large.evrp");
    EXPECT_EQ(params.enable_logging, true);
    EXPECT_EQ(params.stop_criteria, 0);
    EXPECT_EQ(params.enable_multithreading, false);
    EXPECT_EQ(params.nb_granular, 25);
}