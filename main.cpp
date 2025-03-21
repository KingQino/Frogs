#include <thread>
#include <mutex>
#include "parameters.hpp"
#include "command_line.hpp"
#include "case.hpp"
#include "preprocessor.hpp"
#include "lahc.hpp"
#include "magic_enum.hpp"

using namespace std;
using namespace magic_enum;

#define MAX_TRIALS 10

std::mutex perf_mutex;

void run_algorithm(int run, const Parameters* params, vector<double>& perf_of_trials) {
    Case* instance = new Case(params->instance);
    auto* preprocessor = new Preprocessor(*instance, *params);

    switch (params->algorithm) {
        case Algorithm::CBMA: {
            // TODO: Implement CBMA
            break;
        }

        case Algorithm::LAHC: {
            Lahc* lahc = new Lahc(run, instance, preprocessor);
            lahc->run();
            {
                std::lock_guard<std::mutex> lock(perf_mutex);
                perf_of_trials[run - 1] = lahc->global_best->lower_cost;
            }
            delete lahc;
            break;
        }
    }

    delete preprocessor;
    delete instance;
}

int main(int argc, char *argv[])
{
    Parameters params;

    CommandLine cmd(argc, argv);
    cmd.parse_parameters(params);

    vector<double> perf_of_trials(MAX_TRIALS, 0.0);
    if (!params.enable_multithreading){
        run_algorithm(1, &params, std::ref(perf_of_trials));
    } else {
        std::vector<std::thread> threads;

        for (int run = 2; run <= MAX_TRIALS; ++run) {
            threads.emplace_back(run_algorithm, run, &params, std::ref(perf_of_trials));
        }
        run_algorithm(1, &params, std::ref(perf_of_trials));

        for (auto& thread : threads) {
            thread.join();
        }
    }


    string stats_file_path = kStatsPath + "/" + static_cast<string>(enum_name(params.algorithm)) + "/" +
                             params.instance.substr(0, params.instance.find('.'));

    StatsInterface::create_directories_if_not_exists(stats_file_path);
    StatsInterface::stats_for_multiple_trials(stats_file_path + "/" + "stats." + params.instance,perf_of_trials);

    return 0;
}
