//
// Created by Yinghao Qin on 11/02/2025.
//

#ifndef FROGS_CASE_HPP
#define FROGS_CASE_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <cstring>
#include <string>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <numeric>

using namespace std;

const string kDataPath = "../data/";


class Case {
public:
    Case(const string& file_name);
    ~Case();
    void read_problem(const string& file_path);	                                    // reads .evrp file
    static double **generate_2D_matrix_double(int n, int m);                        // generate a 2D matrix of double
    [[nodiscard]] double euclidean_distance(int i, int j) const;                    // calculate the Euclidean distance between two nodes
    [[nodiscard]] int get_customer_demand_(int customer) const;				        // returns the customer demand
    [[nodiscard]] double get_distance(int from, int to);				            // returns the distance
    [[nodiscard]] double get_evals() const;									        // returns the number of evaluations
    [[nodiscard]] bool is_charging_station(int node) const;					        // returns true if node is a charging station
    [[nodiscard]] double compute_total_distance(const vector<vector<int>>& routes); // return the total distance of the given routes
    [[nodiscard]] double compute_total_distance(const vector<int>& route) const;


    string file_name_;
    string instance_name_;

    int depot_{};                           // depot id (usually 0)
    int num_depot_{};
    int num_customer_{};
    int num_station_{};
    int num_vehicle_{};
    int problem_size_{};                    // Total number of customers, charging stations and depot
    int max_vehicle_capa_{};                // maximum capacity of the vehicle
    double max_battery_capa_{};             // maximum energy capacity of the vehicle
    double energy_consumption_rate_{};      // energy consumption rate
    double optimum_{};
    double** distances_{};                  // distance matrix
    double evals_{};                        // number of evaluations used
    vector<int> demand_;                    // size = num_customer_ + 1
    vector<pair<double, double>> positions_;// coordinates of the nodes
};


#endif //FROGS_CASE_HPP
