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
//    indiv->evaluateCompleteCost();
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

//    indiv->upper_cost.penalised_cost = potential[0][instance->num_customer_];
//    indiv->upper_cost.distance = potential[0][instance->num_customer_];

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

Split::Split(Case* instance, Preprocessor* preprocessor): instance(instance), preprocessor(preprocessor)
{
	// Structures of the linear Split
	cliSplit = std::vector <ClientSplit>(instance->num_customer_ + 1);
	sumDistance = std::vector <double>(instance->num_customer_ + 1,0.);
	sumLoad = std::vector <double>(instance->num_customer_ + 1,0.);
	sumService = std::vector <double>(instance->num_customer_ + 1, 0.);
	potential = std::vector<vector<double>>(preprocessor->route_cap_ + 1, std::vector<double>(instance->num_customer_ + 1,1.e30));
	pred = std::vector < std::vector <int> >(preprocessor->route_cap_ + 1, std::vector<int>(instance->num_customer_ + 1, 0));
}
