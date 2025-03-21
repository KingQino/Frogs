#include "Split.h" 

void Split::generalSplit(Individual * indiv, int nbMaxVehicles)
{
    // Do not apply Split with fewer vehicles than the trivial (LP) bin packing bound
    maxVehicles = std::max<int>(nbMaxVehicles, static_cast<int>(std::ceil(preprocessor->total_demand_/instance->max_vehicle_capa_)));

    // Initialization of the data structures for the linear split algorithms
    // Direct application of the code located at https://github.com/vidalt/Split-Library
    for (int i = 1; i <= instance->num_customer_; i++)
    {
        cliSplit[i].demand = preprocessor->customers_[indiv->chromT[i - 1]].demand;
        cliSplit[i].serviceTime = preprocessor->customers_[indiv->chromT[i - 1]].service_duration;
        cliSplit[i].d0_x = instance->distances_[0][indiv->chromT[i - 1]];
        cliSplit[i].dx_0 = instance->distances_[indiv->chromT[i - 1]][0];
        cliSplit[i].dnext = (i < instance->num_customer_) ? instance->distances_[indiv->chromT[i - 1]][indiv->chromT[i]] : -1.e30;
        sumLoad[i] = sumLoad[i - 1] + cliSplit[i].demand;
        sumService[i] = sumService[i - 1] + cliSplit[i].serviceTime;
        sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
    }

    // We first try the simple split, and then the Split with limited fleet if this is not successful
    if (splitSimple(indiv) == 0)
        splitLF(indiv);

    // Build up the rest of the Individual structure
    indiv->evaluate_upper_cost();
}

int Split::splitSimple(Individual * indiv)
{
    // Reinitialize the potential structures
    potential[0][0] = 0;
    for (int i = 1; i <= instance->num_customer_; i++)
        potential[0][i] = 1.e30;

    // MAIN ALGORITHM -- Simple Split using Bellman's algorithm in topological order
    // This code has been maintained as it is very simple and can be easily adapted to a variety of constraints, whereas the O(n) Split has a more restricted application scope
    if (preprocessor->is_duration_constraint_)
    {
        for (int i = 0; i < instance->num_customer_; i++)
        {
            double load = 0.;
            double distance = 0.;
            double serviceDuration = 0.;
            for (int j = i + 1; j <= instance->num_customer_ && load <= 1.5 * instance->max_vehicle_capa_; j++)
            {
                load += cliSplit[j].demand;
                serviceDuration += cliSplit[j].serviceTime;
                if (j == i + 1) distance += cliSplit[j].d0_x;
                else distance += cliSplit[j - 1].dnext;
                double cost = distance + cliSplit[j].dx_0
                              + preprocessor->penalty_capacity_ * std::max<double>(load - instance->max_vehicle_capa_, 0.)
                              + preprocessor->penalty_duration_ * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - instance->max_service_time_, 0.);
                if (potential[0][i] + cost < potential[0][j])
                {
                    potential[0][j] = potential[0][i] + cost;
                    pred[0][j] = i;
                }
            }
        }
    }
    else
    {
        Trivial_Deque queue = Trivial_Deque(instance->num_customer_ + 1, 0);
        for (int i = 1; i <= instance->num_customer_; i++)
        {
            // The front is the best predecessor for i
            potential[0][i] = propagate(queue.get_front(), i, 0);
            pred[0][i] = queue.get_front();

            if (i < instance->num_customer_)
            {
                // If i is not dominated by the last of the pile
                if (!dominates(queue.get_back(), i, 0))
                {
                    // then i will be inserted, need to remove whoever is dominated by i.
                    while (queue.size() > 0 && dominatesRight(queue.get_back(), i, 0))
                        queue.pop_back();
                    queue.push_back(i);
                }
                // Check iteratively if front is dominated by the next front
                while (queue.size() > 1 && propagate(queue.get_front(), i + 1, 0) > propagate(queue.get_next_front(), i + 1, 0) - MY_EPSILON)
                    queue.pop_front();
            }
        }
    }

    if (potential[0][instance->num_customer_] > 1.e29)
        throw std::string("ERROR : no Split solution has been propagated until the last node");

    // Filling the chromR structure
    for (int k = preprocessor->route_cap_ - 1; k >= maxVehicles; k--)
        indiv->chromR[k].clear();

    int end = instance->num_customer_;
    for (int k = maxVehicles - 1; k >= 0; k--)
    {
        indiv->chromR[k].clear();
        int begin = pred[0][end];
        for (int ii = begin; ii < end; ii++)
            indiv->chromR[k].push_back(indiv->chromT[ii]);
        end = begin;
    }

    // Return OK in case the Split algorithm reached the beginning of the routes
    return (end == 0);
}

// Split for problems with limited fleet
int Split::splitLF(Individual * indiv)
{
    // Initialize the potential structures
    potential[0][0] = 0;
    for (int k = 0; k <= maxVehicles; k++)
        for (int i = 1; i <= instance->num_customer_; i++)
            potential[k][i] = 1.e30;

    // MAIN ALGORITHM -- Simple Split using Bellman's algorithm in topological order
    // This code has been maintained as it is very simple and can be easily adapted to a variety of constraints, whereas the O(n) Split has a more restricted application scope
    if (preprocessor->is_duration_constraint_)
    {
        for (int k = 0; k < maxVehicles; k++)
        {
            for (int i = k; i < instance->num_customer_ && potential[k][i] < 1.e29 ; i++)
            {
                double load = 0.;
                double serviceDuration = 0.;
                double distance = 0.;
                for (int j = i + 1; j <= instance->num_customer_ && load <= 1.5 * instance->max_vehicle_capa_; j++) // Setting a maximum limit on load infeasibility to accelerate the algorithm
                {
                    load += cliSplit[j].demand;
                    serviceDuration += cliSplit[j].serviceTime;
                    if (j == i + 1) distance += cliSplit[j].d0_x;
                    else distance += cliSplit[j - 1].dnext;
                    double cost = distance + cliSplit[j].dx_0
                                  + preprocessor->penalty_capacity_ * std::max<double>(load - instance->max_vehicle_capa_, 0.)
                                  + preprocessor->penalty_duration_ * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - instance->max_service_time_, 0.);
                    if (potential[k][i] + cost < potential[k + 1][j])
                    {
                        potential[k + 1][j] = potential[k][i] + cost;
                        pred[k + 1][j] = i;
                    }
                }
            }
        }
    }
    else // MAIN ALGORITHM -- Without duration constraints in O(n), from "Vidal, T. (2016). Split algorithm in O(n) for the capacitated vehicle routing problem. C&OR"
    {
        Trivial_Deque queue = Trivial_Deque(instance->num_customer_ + 1, 0);
        for (int k = 0; k < maxVehicles; k++)
        {
            // in the Split problem there is always one feasible solution with k routes that reaches the index k in the tour.
            queue.reset(k);

            // The range of potentials < 1.29 is always an interval.
            // The size of the queue will stay >= 1 until we reach the end of this interval.
            for (int i = k + 1; i <= instance->num_customer_ && queue.size() > 0; i++)
            {
                // The front is the best predecessor for i
                potential[k + 1][i] = propagate(queue.get_front(), i, k);
                pred[k + 1][i] = queue.get_front();

                if (i < instance->num_customer_)
                {
                    // If i is not dominated by the last of the pile
                    if (!dominates(queue.get_back(), i, k))
                    {
                        // then i will be inserted, need to remove whoever he dominates
                        while (queue.size() > 0 && dominatesRight(queue.get_back(), i, k))
                            queue.pop_back();
                        queue.push_back(i);
                    }

                    // Check iteratively if front is dominated by the next front
                    while (queue.size() > 1 && propagate(queue.get_front(), i + 1, k) > propagate(queue.get_next_front(), i + 1, k) - MY_EPSILON)
                        queue.pop_front();
                }
            }
        }
    }

    if (potential[maxVehicles][instance->num_customer_] > 1.e29)
        throw std::string("ERROR : no Split solution has been propagated until the last node");

    // It could be cheaper to use a smaller number of vehicles
    double minCost = potential[maxVehicles][instance->num_customer_];
    int nbRoutes = maxVehicles;
    for (int k = 1; k < maxVehicles; k++)
        if (potential[k][instance->num_customer_] < minCost)
        {minCost = potential[k][instance->num_customer_]; nbRoutes = k;}

    // Filling the chromR structure
    for (int k = preprocessor->route_cap_ - 1; k >= nbRoutes ; k--)
        indiv->chromR[k].clear();

    int end = instance->num_customer_;
    for (int k = nbRoutes - 1; k >= 0; k--)
    {
        indiv->chromR[k].clear();
        int begin = pred[k+1][end];
        for (int ii = begin; ii < end; ii++)
            indiv->chromR[k].push_back(indiv->chromT[ii]);
        end = begin;
    }

    // Return OK in case the Split algorithm reached the beginning of the routes
    return (end == 0);
}

vector<vector<int>> Split::prinsSplit(const vector<int> &chromosome) {
    vector<int> x(chromosome.size() + 1, 0); // a giant tour starts from 0
    copy(chromosome.begin(), chromosome.end(), x.begin() + 1);

    vector<double> vv(x.size(), std::numeric_limits<double>::max()); // value, the accumulated cost of the shortest path from 0 to i
    vector<int> pp(x.size(), 0); // path, record the split routes of the corresponding shortest path
    vv[0] = 0.0;

    for (int i = 1; i < x.size(); ++i) {
        int load = 0;
        double cost = 0;
        int j = i;
        do
        {
            load += instance->get_customer_demand_(x[j]);
            if (i == j) {
                cost = instance->get_distance(instance->depot_, x[j]) * 2;
            } else {
                cost -= instance->get_distance(x[j -1], instance->depot_);
                cost += instance->get_distance(x[j -1], x[j]);
                cost += instance->get_distance(instance->depot_, x[j]);
            }

            if (load <= instance->max_vehicle_capa_) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load >instance->max_vehicle_capa_));
    }

    vector<vector<int>> all_routes;
    auto j = x.size() - 1;
    while (true) {
        int i = pp[j];
        vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
        all_routes.push_back(temp);
        j = i;
        if (i == 0) {
            break;
        }
    }

    return all_routes;
}

void Split::initIndividualWithHienClustering(Individual* ind) {
    vector<int> chromosome = preprocessor->customer_ids_;

    // Clustering
    std::shuffle(chromosome.begin(), chromosome.end(), random_engine);
    vector<vector<int>> routes;
    while (!chromosome.empty()) {
        vector<int> route;
        int anchor = chromosome.front();
        chromosome.erase(chromosome.begin());
        route.push_back(anchor);
        int cap = instance->get_customer_demand_(anchor);

        // Using remove_if for efficient erasing
        chromosome.erase(std::remove_if(chromosome.begin(), chromosome.end(),
                                        [&](int node) {
                                            if (cap + instance->get_customer_demand_(node) > instance->max_vehicle_capa_) {
                                                return false;  // Skip this customer
                                            }
                                            route.push_back(node);
                                            cap += instance->get_customer_demand_(node);
                                            return true;  // Remove from chromosome
                                        }),
                         chromosome.end());

        routes.push_back(std::move(route));  // Move route to avoid unnecessary copying
    }



    // Balance the routes
    vector<int>& lastRoute = routes.back();
    uniform_int_distribution<int> distribution(0, static_cast<int>(lastRoute.size() - 1));
    int customer = lastRoute[distribution(random_engine)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (int node : lastRoute) {
        cap1 += instance->get_customer_demand_(node);
    }

    int size = instance->num_customer_ - 1; // the size of nearby_customers
    for (int i = 0; i < size; ++i) {
        int x = preprocessor->sorted_nearby_customers_[customer][i];

        // Skip if already in lastRoute
        if (find(lastRoute.begin(), lastRoute.end(), x) != lastRoute.end()) {
            continue;
        }

        // Find which route contains x
        auto route2It = find_if(routes.begin(), routes.end(), [x](const vector<int>& route) {
            return find(route.begin(), route.end(), x) != route.end();
        });

        if (route2It == routes.end()) {
            continue;  // x is not in any route
        }

        vector<int>& route2 = *route2It;
        int cap2 = 0;
        for (int node : route2) {
            cap2 += instance->get_customer_demand_(node);
        }

        int demand_X = instance->get_customer_demand_(x);

        if (demand_X + cap1 <= instance->max_vehicle_capa_ && abs((cap1 + demand_X) - (cap2 - demand_X)) < abs(cap1 - cap2)) {
            // Move x from route2 to lastRoute
            route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
            lastRoute.push_back(x);
            cap1 += demand_X;
        } else {
            break; // No further balancing needed
        }
    }

    // Efficient chromT and chromR update
    ind->chromT.clear();
    ind->chromT.reserve(instance->num_customer_);  // Prevent reallocations

    int index = 0;
    for (auto& tour : routes) {
        ind->chromT.insert(ind->chromT.end(), tour.begin(), tour.end());
        ind->chromR[index++] = std::move(tour);
    }

    ind->evaluate_upper_cost();
}

void Split::initIndividualWithDirectEncoding(Individual* ind) {
    vector<int> customers_(preprocessor->customer_ids_);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance->depot_}; // the first route starts from depot_ 0

    vector<vector<int>> all_routes;
    while(!customers_.empty()) {
        vector<int> all_temp;
        for(int i : customers_) {
            if(instance->get_customer_demand_(i) <= instance->max_vehicle_capa_ - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        int remain_total_demand_ = accumulate(customers_.begin(), customers_.end(), 0, [&](int total, int i) {
            return total + instance->get_customer_demand_(i);
        });
        if(remain_total_demand_ <= instance->max_vehicle_capa_ * (instance->num_vehicle_ - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance->depot_); // add depot_ node into the all_temp
        }

        int cur = route.back();
        uniform_int_distribution<> distribution(0, static_cast<int>(all_temp.size()) - 1);
        int next = all_temp[distribution(random_engine)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance->depot_) {
            if (cur == instance->depot_) continue; // fix-bug
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance->get_customer_demand_(next);
            customers_.erase(remove(customers_.begin(), customers_.end(), next), customers_.end());
        }
    }

    all_routes.push_back(route);

    // 🔹 Remove `instance.depot_` from each route
    for (auto& tour : all_routes) {
        tour.erase(remove(tour.begin(), tour.end(), instance->depot_), tour.end());
    }

    int index = 0;
    ind->chromT.clear();  // Clear previous values
    for (const auto& tour : all_routes) {
        ind->chromT.insert(ind->chromT.end(), tour.begin(), tour.end());
        ind->chromR[index++] = tour;
    }

    ind->evaluate_upper_cost();
}


Split::Split(int seed, Case* instance, Preprocessor* preprocessor): instance(instance), preprocessor(preprocessor)
{
	// Structures of the linear Split
	cliSplit = std::vector <ClientSplit>(instance->num_customer_ + 1);
	sumDistance = std::vector <double>(instance->num_customer_ + 1,0.);
	sumLoad = std::vector <double>(instance->num_customer_ + 1,0.);
	sumService = std::vector <double>(instance->num_customer_ + 1, 0.);
	potential = std::vector<vector<double>>(preprocessor->route_cap_ + 1, std::vector<double>(instance->num_customer_ + 1,1.e30));
	pred = std::vector < std::vector <int> >(preprocessor->route_cap_ + 1, std::vector<int>(instance->num_customer_ + 1, 0));
    random_engine = std::default_random_engine(seed);
}
