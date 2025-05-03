#include <mpi.h>  // MPI header
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

double run_algorithm(int run, const Parameters* params) {
    Case* instance = new Case(params->instance);
    auto* preprocessor = new Preprocessor(*instance, *params);

    double result = 0.0;
    switch (params->algorithm) {
        case Algorithm::CBMA: {
            Cbma* cbma = new Cbma(run, instance, preprocessor);
            cbma->run();
            result = cbma->global_best->lower_cost;
            delete cbma;
            break;
        }

        case Algorithm::LAHC: {
            Lahc* lahc = new Lahc(run, instance, preprocessor);
            lahc->run();
            result = lahc->global_best->lower_cost;
            delete lahc;
            break;
        }

        case Algorithm::SGA: {
            Sga* sga = new Sga(run, instance, preprocessor);
            sga->run();
            result = sga->global_best->lower_cost;
            delete sga;
            break;
        }
    }

    delete preprocessor;
    delete instance;

    return result;
}

int main(int argc, char *argv[])
{
    MPI_Init(&argc, &argv);

    int rank, world_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);       // process ID
    MPI_Comm_size(MPI_COMM_WORLD, &world_size); // total processes

    Parameters params;
    CommandLine cmd(argc, argv);
    cmd.parse_parameters(params);

    double local_result = 0.0;

    // 防止超过最大实验次数
    if (rank < MAX_TRIALS) {
        local_result = run_algorithm(rank + 1, &params);
    }

    // 收集结果到主进程
    vector<double> all_results;
    if (rank == 0) {
        all_results.resize(MAX_TRIALS, 0.0);
    }

    MPI_Gather(&local_result, 1, MPI_DOUBLE,
               all_results.data(), 1, MPI_DOUBLE,
               0, MPI_COMM_WORLD);


    if (rank == 0) {
        string stats_file_path = kStatsPath + "/" + static_cast<string>(enum_name(params.algorithm)) + "/" +
                                 params.instance.substr(0, params.instance.find('.'));

        StatsInterface::create_directories_if_not_exists(stats_file_path);
        StatsInterface::stats_for_multiple_trials(stats_file_path + "/" + "stats." + params.instance, all_results);
    }

    MPI_Finalize();

    return 0;
}
