//
// Created by Yinghao Qin on 05/03/2025.
//

#ifndef FROGS_INITIALIZER_HPP
#define FROGS_INITIALIZER_HPP

#include "solution.hpp"

class Initializer {
public:
    Case* instance{};
    Preprocessor* preprocessor{};
    std::mt19937& random_engine;

    Initializer(std::mt19937& engine, Case* instance, Preprocessor* preprocessor);
    ~Initializer();

    [[nodiscard]] vector<vector<int>> prins_split(const vector<int>& chromosome) const;
    vector<vector<int>> hien_clustering();
    void hien_balancing(vector<vector<int>>& routes);
    vector<vector<int>> routes_constructor_with_split();
    vector<vector<int>> routes_constructor_with_hien_method();
    vector<vector<int>> routes_constructor_with_direct_encoding();
};

#endif //FROGS_INITIALIZER_HPP
