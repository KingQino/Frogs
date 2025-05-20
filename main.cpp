#include <omp.h>  // OpenMP header
#include "parameters.hpp"
#include "command_line.hpp"
#include "case.hpp"
#include "preprocessor.hpp"
#include "lahc.hpp"
#include "cbma.hpp"
#include "sga.hpp"
#include "magic_enum.hpp"

using namespace std;
using namespace magic_enum;

#define MAX_TRIALS 10

void run_algorithm(int run, const Parameters* params, vector<double>& perf_of_trials) {
    Case* instance = new Case(params->kDataPath, params->instance);
    auto* preprocessor = new Preprocessor(*instance, *params);

    switch (params->algorithm) {
        case Algorithm::CBMA: {
            Cbma* cbma = new Cbma(run, instance, preprocessor);
            cbma->run();

            // Prevent race condition on shared vector
            #pragma omp critical
            {
                perf_of_trials[run - 1] = cbma->global_best->lower_cost;
            }

            delete cbma;
            break;
        }

        case Algorithm::LAHC: {
            Lahc* lahc = new Lahc(run, instance, preprocessor);
            lahc->run();

            // Prevent race condition on shared vector
            #pragma omp critical
            {
                perf_of_trials[run - 1] = lahc->global_best->lower_cost;
            }

            delete lahc;
            break;
        }

        case Algorithm::SGA: {
            Sga* sga = new Sga(run, instance, preprocessor);
            sga->run();

            // Prevent race condition on shared vector
            #pragma omp critical
            {
                perf_of_trials[run - 1] = sga->global_best->lower_cost;
            }

            delete sga;
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
        // OpenMP parallel loop
        #pragma omp parallel for default(none) shared(params, perf_of_trials)
        for (int run = 1; run <= MAX_TRIALS; ++run) {
            run_algorithm(run, &params, perf_of_trials);
        }
    }


    string stats_file_path = params.kStatsPath + "/" + static_cast<string>(enum_name(params.algorithm)) + "/" +
                             params.instance.substr(0, params.instance.find('.'));

    StatsInterface::create_directories_if_not_exists(stats_file_path);
    StatsInterface::stats_for_multiple_trials(stats_file_path + "/" + "stats." + params.instance,perf_of_trials);

    Indicators indicators = StatsInterface::calculate_statistical_indicators(perf_of_trials);
    std::cout << indicators.avg << std::endl;

    return 0;
}
