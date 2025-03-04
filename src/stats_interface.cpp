//
// Created by Yinghao Qin on 19/02/2025.
//

#include "stats_interface.hpp"

Indicators StatsInterface::calculate_statistical_indicators(const std::vector<double>& data) {
    Indicators indicators;

    if (data.empty()) return indicators;

    indicators.size = data.size();
    indicators.min = *std::min_element(data.begin(), data.end());
    indicators.max = *std::max_element(data.begin(), data.end());
    const double sum = std::accumulate(data.begin(), data.end(), 0.0);
    indicators.avg = sum / static_cast<double>(indicators.size);

    if (indicators.size == 1) {
        indicators.std = 0.0;
    } else {
        const double variance = std::accumulate(data.begin(), data.end(), 0.0,
                                                [indicators](const double accum, const double value) {
                                                    return accum + (value - indicators.avg) * (value - indicators.avg);
                                                }) / static_cast<double>(indicators.size - 1);
        indicators.std = std::sqrt(variance);
    }

    return indicators;
}

void StatsInterface::calculate_statistical_indicators(const std::vector<double>& data, Indicators& indicators) {
    if (data.empty()) return;

    indicators.size = data.size();
    indicators.min = *std::min_element(data.begin(), data.end());
    indicators.max = *std::max_element(data.begin(), data.end());
    const double sum = std::accumulate(data.begin(), data.end(), 0.0);
    indicators.avg = sum / static_cast<double>(indicators.size);

    if (indicators.size == 1) {
        indicators.std = 0.0;
    } else {
        const double variance = std::accumulate(data.begin(), data.end(), 0.0,
                                          [indicators](const double accum, const double value) {
                                              return accum + (value - indicators.avg) * (value - indicators.avg);
                                          }) / static_cast<double>(indicators.size - 1);
        indicators.std = std::sqrt(variance);
    }
}

bool StatsInterface::create_directories_if_not_exists(const string& directory_path) {
    if (!fs::exists(directory_path)) {
        try {
            fs::create_directories(directory_path);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return false;
        }
    } else {
        return true;
    }
}

void StatsInterface::stats_for_multiple_trials(const std::string& file_path, const std::vector<double>& data) {
    std::ofstream log_stats;

    log_stats.open(file_path);

    std::ostringstream oss;
    for (auto& perf:data) {
        oss << fixed << setprecision(2) << perf << endl;
    }
    Indicators indicators = calculate_statistical_indicators(data);
    oss << "Mean " << indicators.avg << "\t \tStd Dev " << indicators.std << "\t " << endl;
    oss << "Min: " << indicators.min << "\t " << endl;
    oss << "Max: " << indicators.max << "\t " << endl;
    log_stats << oss.str() << flush;

    log_stats.close();
}
