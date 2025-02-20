//
// Created by Yinghao Qin on 11/02/2025.
//

#include "case.hpp"

Case::Case(const string& file_name) {
    this->file_name_ = file_name;
    this->instance_name_ = file_name.substr(0, file_name.find('.'));

    this->read_problem(kDataPath + file_name);
}


Case::~Case() {
    for (int i = 0; i < problem_size_; i++) {
        delete[] this->distances_[i];
    }
    delete[] this->distances_;
}

void Case::read_problem(const std::string &file_path) {
    this->num_depot_ = 1;

    // Helper lambda to extract values after a colon
    auto extract_value = [&](const string& line) {
        return line.substr(line.find(':') + 1);
    };

    ifstream infile(file_path);
    string line;

    // Read file line by line
    while (getline(infile, line)) {
        stringstream ss;
        if (line.find("DIMENSION:") != string::npos) {
            ss << extract_value(line);
            ss >> this->num_customer_;
            this->num_customer_--;  // Adjusting the customer number
        }
        else if (line.find("STATIONS:") != string::npos) {
            ss << extract_value(line);
            ss >> this->num_station_;
        }
        else if (line.find("VEHICLES:") != string::npos) {
            ss << extract_value(line);
            ss >> this->num_vehicle_;
        }
        else if (line.find("CAPACITY:") != string::npos && line.find("ENERGY") == string::npos) {
            ss << extract_value(line);
            ss >> this->max_vehicle_capa_;
        }
        else if (line.find("ENERGY_CAPACITY:") != string::npos) {
            ss << extract_value(line);
            ss >> this->max_battery_capa_;
        }
        else if (line.find("ENERGY_CONSUMPTION:") != string::npos) {
            ss << extract_value(line);
            ss >> this->energy_consumption_rate_;
        }
        else if (line.find("OPTIMAL_VALUE:") != string::npos) {
            ss << extract_value(line);
            ss >> this->optimum_;
        }
        else if (line.find("NODE_COORD_SECTION") != string::npos) {
            this->problem_size_ = num_depot_ + num_customer_ + num_station_;
            positions_.resize(problem_size_, {0, 0});

            // Reading coordinates
            for (int i = 0; i < problem_size_; ++i) {
                getline(infile, line);
                ss.str(line);
                int ind;
                double x, y;
                ss >> ind >> x >> y;
                positions_[ind - 1] = {x, y};  // Set the position
            }
        }
        else if (line.find("DEMAND_SECTION") != string::npos) {
            int total_number = num_depot_ + num_customer_;
            demand_.resize(total_number, 0);

            // Reading demand_ values
            for (int i = 0; i < total_number; ++i) {
                getline(infile, line);
                ss.clear();
                ss.str(line);
                int ind, c;
                ss >> ind >> c;
                demand_[ind - 1] = c;
                if (c == 0) depot_ = ind - 1;  // Identify depot_
            }
        }
    }
    infile.close();


    this->max_service_time_ = std::numeric_limits<double>::max();
    this->distances_ = generate_2D_matrix_double(problem_size_, problem_size_);
    for (int i = 0; i < problem_size_; i++) {
        for (int j = 0; j < problem_size_; j++) {
            distances_[i][j] = euclidean_distance(i, j);
        }
    }
}

double **Case::generate_2D_matrix_double(int n, int m) {
    auto **matrix = new double *[n];
    for (int i = 0; i < n; i++) {
        matrix[i] = new double[m];
    }
    //initialize the 2-d array
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            matrix[i][j] = 0.0;
        }
    }
    return matrix;
}

double Case::euclidean_distance(const int i, const int j) const {
    return sqrt(pow(positions_[i].first - positions_[j].first, 2) +
                pow(positions_[i].second - positions_[j].second, 2));
}

int Case::get_customer_demand_(const int customer) const {
    return demand_[customer];
}

double Case::get_distance(const int from, const int to) {
    //adds partial evaluation to the overall fitness evaluation count
    //It can be used when local search is used and a whole evaluation is not necessary
    evals_ += (1.0 / problem_size_);

    return distances_[from][to];
}

double Case::get_evals() const {
    return evals_;
}

double Case::calculate_total_dist(const vector<vector<int>>& chromR) const {
    double tour_length = 0.0;

    for (const auto& route : chromR) {
        if (route.empty()) continue;

        tour_length += distances_[depot_][route[0]];
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances_[route[j]][route[j + 1]];
        }
        tour_length += distances_[route.back()][depot_];
    }

    return tour_length;
}

// TODO: modify these two functions below for new Individual structure
double Case::compute_total_distance(const vector<vector<int>> &routes) {
    double tour_length = 0.0;
    for (auto& route : routes) {
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances_[route[j]][route[j + 1]];
        }
    }

    evals_++;

    return tour_length;
}

double Case::compute_total_distance(const vector<int> &route) const {
    double tour_length = 0.0;
    for (int j = 0; j < route.size() - 1; ++j) {
        tour_length += distances_[route[j]][route[j + 1]];
    }

    return tour_length;
}

bool Case::is_charging_station(const int node) const {
    bool flag;
    if (node == depot_ || ( node >= num_depot_ + num_customer_ && node < problem_size_))
        flag = true;
    else
        flag = false;
    return flag;
}