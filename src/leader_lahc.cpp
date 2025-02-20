//
// Created by Yinghao Qin on 20/02/2025.
//

#include "leader_lahc.hpp"

void LeaderLahc::run(Individual * indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
    this->penaltyCapacityLS = penaltyCapacityLS;
    this->penaltyDurationLS = penaltyDurationLS;
    loadIndividual(indiv);

    // Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
    std::shuffle(orderNodes.begin(), orderNodes.end(), random_engine);
    // Designed to use O(nbGranular x n) time overall to avoid possible bottlenecks
    for (int i = 1; i <= instance->num_customer_; i++) {
        std::uniform_int_distribution<int> distribution(0, preprocessor->nb_granular_ - 1);
        if (distribution(random_engine) == 0) { // Random condition check
            std::shuffle(preprocessor->correlated_vertices_[i].begin(),  preprocessor->correlated_vertices_[i].end(), random_engine);
        }
    }


    searchCompleted = false;
    for (loopID = 0; !searchCompleted; loopID++)
    {
        if (loopID > 1) // Allows at least two loops since some moves involving empty routes are not checked at the first loop
            searchCompleted = true;

//        int count_nodeU = 0;
        /* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
        for (int posU = 0; posU < instance->num_customer_; posU++)
        {
            nodeU = &clients[orderNodes[posU]];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;
//            count_nodeU++;
//            std::cout << "nodeU changed: " << nodeU->cour <<  " | count_nodeU: " << count_nodeU << std::endl;
            for (int posV = 0; posV < (int)preprocessor->correlated_vertices_[nodeU->cour].size(); posV++)
            {
                nodeV = &clients[preprocessor->correlated_vertices_[nodeU->cour][posV]];
//                if(lastTestRINodeU != -1) {
//                    std::cout << "lastTestRINodeU: " << lastTestRINodeU << std::endl;
//                }
                if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
                {
                    // Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
                    setLocalVariablesRouteU();
                    setLocalVariablesRouteV();
                    if (move1()) continue; // RELOCATE
                    if (move2()) continue; // RELOCATE
                    if (move3()) continue; // RELOCATE
                    if (nodeUIndex <= nodeVIndex && move4()) continue; // SWAP
                    if (move5()) continue; // SWAP
                    if (nodeUIndex <= nodeVIndex && move6()) continue; // SWAP
                    if (routeU == routeV && move7()) continue; // 2-OPT
                    if (routeU != routeV && move8()) continue; // 2-OPT*
                    if (routeU != routeV && move9()) continue; // 2-OPT*

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot)
                    {
                        nodeV = nodeV->prev;
                        setLocalVariablesRouteV();
                        if (move1()) continue; // RELOCATE
                        if (move2()) continue; // RELOCATE
                        if (move3()) continue; // RELOCATE
                        if (routeU != routeV && move8()) continue; // 2-OPT*
                        if (routeU != routeV && move9()) continue; // 2-OPT*
                    }
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (loopID > 0 && !emptyRoutes.empty())
            {
                nodeV = routes[*emptyRoutes.begin()].depot;
                setLocalVariablesRouteU();
                setLocalVariablesRouteV();
                if (move1()) continue; // RELOCATE
                if (move2()) continue; // RELOCATE
                if (move3()) continue; // RELOCATE
                if (move9()) continue; // 2-OPT*
            }
        }
    }

    // Register the solution produced by the LS in the individual
    exportIndividual(indiv);
}

void LeaderLahc::setLocalVariablesRouteU()
{
    routeU = nodeU->route;
    nodeX = nodeU->next;
    nodeXNextIndex = nodeX->next->cour;
    nodeUIndex = nodeU->cour;
    nodeUPrevIndex = nodeU->prev->cour;
    nodeXIndex = nodeX->cour;
    loadU    = preprocessor->customers_[nodeUIndex].demand;
    serviceU = preprocessor->customers_[nodeUIndex].service_duration;
    loadX	 = preprocessor->customers_[nodeXIndex].demand;
    serviceX = preprocessor->customers_[nodeXIndex].service_duration;
}

void LeaderLahc::setLocalVariablesRouteV()
{
    routeV = nodeV->route;
    nodeY = nodeV->next;
    nodeYNextIndex = nodeY->next->cour;
    nodeVIndex = nodeV->cour;
    nodeVPrevIndex = nodeV->prev->cour;
    nodeYIndex = nodeY->cour;
    loadV    = preprocessor->customers_[nodeVIndex].demand;
    serviceV = preprocessor->customers_[nodeVIndex].service_duration;
    loadY	 = preprocessor->customers_[nodeYIndex].demand;
    serviceY = preprocessor->customers_[nodeYIndex].service_duration;
}

bool LeaderLahc::move1()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeXIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeUIndex, nodeXIndex);
    double costSuppV = instance->get_distance(nodeVIndex, nodeUIndex) + instance->get_distance(nodeUIndex, nodeYIndex) - instance->get_distance(nodeVIndex, nodeYIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU)
                     + penaltyExcessLoad(routeU->load - loadU)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU)
                     + penaltyExcessLoad(routeV->load + loadU)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeUIndex == nodeYIndex) return false;

    insertNode(nodeU, nodeV);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move2()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeXNextIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeXIndex, nodeXNextIndex);
    double costSuppV = instance->get_distance(nodeVIndex, nodeUIndex) + instance->get_distance(nodeXIndex, nodeYIndex) - instance->get_distance(nodeVIndex, nodeYIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - instance->get_distance(nodeUIndex, nodeXIndex) - serviceU - serviceX)
                     + penaltyExcessLoad(routeU->load - loadU - loadX)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + instance->get_distance(nodeUIndex, nodeXIndex) + serviceU + serviceX)
                     + penaltyExcessLoad(routeV->load + loadU + loadX)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;

    insertNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move3()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeXNextIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeUIndex, nodeXIndex) - instance->get_distance(nodeXIndex, nodeXNextIndex);
    double costSuppV = instance->get_distance(nodeVIndex, nodeXIndex) + instance->get_distance(nodeXIndex, nodeUIndex) + instance->get_distance(nodeUIndex, nodeYIndex) - instance->get_distance(nodeVIndex, nodeYIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU - serviceX)
                     + penaltyExcessLoad(routeU->load - loadU - loadX)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU + serviceX)
                     + penaltyExcessLoad(routeV->load + loadU + loadX)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

    insertNode(nodeX, nodeV);
    insertNode(nodeU, nodeX);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move4()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeVIndex) + instance->get_distance(nodeVIndex, nodeXIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeUIndex, nodeXIndex);
    double costSuppV = instance->get_distance(nodeVPrevIndex, nodeUIndex) + instance->get_distance(nodeUIndex, nodeYIndex) - instance->get_distance(nodeVPrevIndex, nodeVIndex) - instance->get_distance(nodeVIndex, nodeYIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU + serviceV - serviceU)
                     + penaltyExcessLoad(routeU->load + loadV - loadU)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV - serviceV + serviceU)
                     + penaltyExcessLoad(routeV->load + loadU - loadV)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

    swapNode(nodeU, nodeV);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move5()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeVIndex) + instance->get_distance(nodeVIndex, nodeXNextIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeXIndex, nodeXNextIndex);
    double costSuppV = instance->get_distance(nodeVPrevIndex, nodeUIndex) + instance->get_distance(nodeXIndex, nodeYIndex) - instance->get_distance(nodeVPrevIndex, nodeVIndex) - instance->get_distance(nodeVIndex, nodeYIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - instance->get_distance(nodeUIndex, nodeXIndex) + serviceV - serviceU - serviceX)
                     + penaltyExcessLoad(routeU->load + loadV - loadU - loadX)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + instance->get_distance(nodeUIndex, nodeXIndex) - serviceV + serviceU + serviceX)
                     + penaltyExcessLoad(routeV->load + loadU + loadX - loadV)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;

    swapNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move6()
{
    double costSuppU = instance->get_distance(nodeUPrevIndex, nodeVIndex) + instance->get_distance(nodeYIndex, nodeXNextIndex) - instance->get_distance(nodeUPrevIndex, nodeUIndex) - instance->get_distance(nodeXIndex, nodeXNextIndex);
    double costSuppV = instance->get_distance(nodeVPrevIndex, nodeUIndex) + instance->get_distance(nodeXIndex, nodeYNextIndex) - instance->get_distance(nodeVPrevIndex, nodeVIndex) - instance->get_distance(nodeYIndex, nodeYNextIndex);

    if (routeU != routeV)
    {
        costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - instance->get_distance(nodeUIndex, nodeXIndex) + instance->get_distance(nodeVIndex, nodeYIndex) + serviceV + serviceY - serviceU - serviceX)
                     + penaltyExcessLoad(routeU->load + loadV + loadY - loadU - loadX)
                     - routeU->penalty;

        costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + instance->get_distance(nodeUIndex, nodeXIndex) - instance->get_distance(nodeVIndex, nodeYIndex) - serviceV - serviceY + serviceU + serviceX)
                     + penaltyExcessLoad(routeV->load + loadU + loadX - loadV - loadY)
                     - routeV->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON) return false;
    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

    swapNode(nodeU, nodeV);
    swapNode(nodeX, nodeY);
    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV) updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move7()
{
    if (nodeU->position > nodeV->position) return false;

    double cost = instance->get_distance(nodeUIndex, nodeVIndex) + instance->get_distance(nodeXIndex, nodeYIndex) - instance->get_distance(nodeUIndex, nodeXIndex) - instance->get_distance(nodeVIndex, nodeYIndex) + nodeV->cumulatedReversalDistance - nodeX->cumulatedReversalDistance;

    if (cost > -MY_EPSILON) return false;
    if (nodeU->next == nodeV) return false;

    Node * nodeNum = nodeX->next;
    nodeX->prev = nodeNum;
    nodeX->next = nodeY;

    while (nodeNum != nodeV)
    {
        Node * temp = nodeNum->next;
        nodeNum->next = nodeNum->prev;
        nodeNum->prev = temp;
        nodeNum = temp;
    }

    nodeV->next = nodeV->prev;
    nodeV->prev = nodeU;
    nodeU->next = nodeV;
    nodeY->prev = nodeX;

    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    return true;
}

bool LeaderLahc::move8()
{
    double cost = instance->get_distance(nodeUIndex, nodeVIndex) + instance->get_distance(nodeXIndex, nodeYIndex) - instance->get_distance(nodeUIndex, nodeXIndex) - instance->get_distance(nodeVIndex, nodeYIndex)
                  + penaltyExcessDuration(nodeU->cumulatedTime + nodeV->cumulatedTime + nodeV->cumulatedReversalDistance + instance->get_distance(nodeUIndex, nodeVIndex))
                  + penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - instance->get_distance(nodeUIndex, nodeXIndex) + routeU->reversalDistance - nodeX->cumulatedReversalDistance + routeV->duration - nodeV->cumulatedTime - instance->get_distance(nodeVIndex, nodeYIndex) + instance->get_distance(nodeXIndex, nodeYIndex))
                  + penaltyExcessLoad(nodeU->cumulatedLoad + nodeV->cumulatedLoad)
                  + penaltyExcessLoad(routeU->load + routeV->load - nodeU->cumulatedLoad - nodeV->cumulatedLoad)
                  - routeU->penalty - routeV->penalty
                  + nodeV->cumulatedReversalDistance + routeU->reversalDistance - nodeX->cumulatedReversalDistance;

    if (cost > -MY_EPSILON) return false;

    Node * depotU = routeU->depot;
    Node * depotV = routeV->depot;
    Node * depotUFin = routeU->depot->prev;
    Node * depotVFin = routeV->depot->prev;
    Node * depotVSuiv = depotV->next;

    Node * temp;
    Node * xx = nodeX;
    Node * vv = nodeV;

    while (!xx->isDepot)
    {
        temp = xx->next;
        xx->next = xx->prev;
        xx->prev = temp;
        xx->route = routeV;
        xx = temp;
    }

    while (!vv->isDepot)
    {
        temp = vv->prev;
        vv->prev = vv->next;
        vv->next = temp;
        vv->route = routeU;
        vv = temp;
    }

    nodeU->next = nodeV;
    nodeV->prev = nodeU;
    nodeX->next = nodeY;
    nodeY->prev = nodeX;

    if (nodeX->isDepot)
    {
        depotUFin->next = depotU;
        depotUFin->prev = depotVSuiv;
        depotUFin->prev->next = depotUFin;
        depotV->next = nodeY;
        nodeY->prev = depotV;
    }
    else if (nodeV->isDepot)
    {
        depotV->next = depotUFin->prev;
        depotV->next->prev = depotV;
        depotV->prev = depotVFin;
        depotUFin->prev = nodeU;
        nodeU->next = depotUFin;
    }
    else
    {
        depotV->next = depotUFin->prev;
        depotV->next->prev = depotV;
        depotUFin->prev = depotVSuiv;
        depotUFin->prev->next = depotUFin;
    }

    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    updateRouteData(routeV);
    return true;
}

bool LeaderLahc::move9()
{
    double cost = instance->get_distance(nodeUIndex, nodeYIndex) + instance->get_distance(nodeVIndex, nodeXIndex) - instance->get_distance(nodeUIndex, nodeXIndex) - instance->get_distance(nodeVIndex, nodeYIndex)
                  + penaltyExcessDuration(nodeU->cumulatedTime + routeV->duration - nodeV->cumulatedTime - instance->get_distance(nodeVIndex, nodeYIndex) + instance->get_distance(nodeUIndex, nodeYIndex))
                  + penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - instance->get_distance(nodeUIndex, nodeXIndex) + nodeV->cumulatedTime + instance->get_distance(nodeVIndex, nodeXIndex))
                  + penaltyExcessLoad(nodeU->cumulatedLoad + routeV->load - nodeV->cumulatedLoad)
                  + penaltyExcessLoad(nodeV->cumulatedLoad + routeU->load - nodeU->cumulatedLoad)
                  - routeU->penalty - routeV->penalty;

    if (cost > -MY_EPSILON) return false;

    Node * depotU = routeU->depot;
    Node * depotV = routeV->depot;
    Node * depotUFin = depotU->prev;
    Node * depotVFin = depotV->prev;
    Node * depotUpred = depotUFin->prev;

    Node * count = nodeY;
    while (!count->isDepot)
    {
        count->route = routeU;
        count = count->next;
    }

    count = nodeX;
    while (!count->isDepot)
    {
        count->route = routeV;
        count = count->next;
    }

    nodeU->next = nodeY;
    nodeY->prev = nodeU;
    nodeV->next = nodeX;
    nodeX->prev = nodeV;

    if (nodeX->isDepot)
    {
        depotUFin->prev = depotVFin->prev;
        depotUFin->prev->next = depotUFin;
        nodeV->next = depotVFin;
        depotVFin->prev = nodeV;
    }
    else
    {
        depotUFin->prev = depotVFin->prev;
        depotUFin->prev->next = depotUFin;
        depotVFin->prev = depotUpred;
        depotVFin->prev->next = depotVFin;
    }

    nbMoves++; // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    updateRouteData(routeV);
    return true;
}

void LeaderLahc::insertNode(Node * U, Node * V)
{
    U->prev->next = U->next;
    U->next->prev = U->prev;
    V->next->prev = U;
    U->prev = V;
    U->next = V->next;
    V->next = U;
    U->route = V->route;
}

void LeaderLahc::swapNode(Node * U, Node * V)
{
    Node * myVPred = V->prev;
    Node * myVSuiv = V->next;
    Node * myUPred = U->prev;
    Node * myUSuiv = U->next;
    Route * myRouteU = U->route;
    Route * myRouteV = V->route;

    myUPred->next = V;
    myUSuiv->prev = V;
    myVPred->next = U;
    myVSuiv->prev = U;

    U->prev = myVPred;
    U->next = myVSuiv;
    V->prev = myUPred;
    V->next = myUSuiv;

    U->route = myRouteV;
    V->route = myRouteU;
}

void LeaderLahc::updateRouteData(Route * myRoute)
{
    int myplace = 0;
    double myload = 0.;
    double mytime = 0.;
    double myReversalDistance = 0.;
    double cumulatedX = 0.;
    double cumulatedY = 0.;

    Node * mynode = myRoute->depot;
    mynode->position = 0;
    mynode->cumulatedLoad = 0.;
    mynode->cumulatedTime = 0.;
    mynode->cumulatedReversalDistance = 0.;

    bool firstIt = true;
    while (!mynode->isDepot || firstIt)
    {
        mynode = mynode->next;
        myplace++;
        mynode->position = myplace;
        myload += preprocessor->customers_[mynode->cour].demand;
        mytime += instance->get_distance(mynode->prev->cour, mynode->cour) + preprocessor->customers_[mynode->cour].service_duration;
        myReversalDistance += instance->get_distance(mynode->cour, mynode->prev->cour) - instance->get_distance(mynode->prev->cour, mynode->cour) ;
        mynode->cumulatedLoad = myload;
        mynode->cumulatedTime = mytime;
        mynode->cumulatedReversalDistance = myReversalDistance;
        if (!mynode->isDepot)
        {
            cumulatedX += preprocessor->customers_[mynode->cour].coord_x;
            cumulatedY += preprocessor->customers_[mynode->cour].coord_y;
            if (firstIt) myRoute->sector.initialize(preprocessor->customers_[mynode->cour].polar_angle);
            else myRoute->sector.extend(preprocessor->customers_[mynode->cour].polar_angle);
        }
        firstIt = false;
    }

    myRoute->duration = mytime;
    myRoute->load = myload;
    myRoute->penalty = penaltyExcessDuration(mytime) + penaltyExcessLoad(myload);
    myRoute->nbCustomers = myplace-1;
    myRoute->reversalDistance = myReversalDistance;
    // Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
    myRoute->whenLastModified = nbMoves ;

    if (myRoute->nbCustomers == 0)
    {
        myRoute->polarAngleBarycenter = 1.e30;
        emptyRoutes.insert(myRoute->cour);
    }
    else
    {
        myRoute->polarAngleBarycenter = atan2(cumulatedY/(double)myRoute->nbCustomers - preprocessor->customers_[0].coord_y, cumulatedX/(double)myRoute->nbCustomers - preprocessor->customers_[0].coord_x);
        emptyRoutes.erase(myRoute->cour);
    }
}

void LeaderLahc::loadIndividual(Individual * indiv)
{
    emptyRoutes.clear();
    nbMoves = 0;
    for (int r = 0; r < preprocessor->route_cap_; r++)
    {
        Node * myDepot = &depots[r];
        Node * myDepotFin = &depotsEnd[r];
        Route * myRoute = &routes[r];
        myDepot->prev = myDepotFin;
        myDepotFin->next = myDepot;
        if (!indiv->chromR[r].empty())
        {
            Node * myClient = &clients[indiv->chromR[r][0]];
            myClient->route = myRoute;
            myClient->prev = myDepot;
            myDepot->next = myClient;
            for (int i = 1; i < (int)indiv->chromR[r].size(); i++)
            {
                Node * myClientPred = myClient;
                myClient = &clients[indiv->chromR[r][i]];
                myClient->prev = myClientPred;
                myClientPred->next = myClient;
                myClient->route = myRoute;
            }
            myClient->next = myDepotFin;
            myDepotFin->prev = myClient;
        }
        else
        {
            myDepot->next = myDepotFin;
            myDepotFin->prev = myDepot;
        }
        updateRouteData(&routes[r]);
        routes[r].whenLastTestedSWAPStar = -1;
    }

    for (int i = 1; i <= instance->num_customer_; i++) // Initializing memory structures
        clients[i].whenLastTestedRI = -1;
}

void LeaderLahc::exportIndividual(Individual * indiv)
{
    std::vector < std::pair <double, int> > routePolarAngles ;
    routePolarAngles.reserve(preprocessor->route_cap_);
    for (int r = 0; r < preprocessor->route_cap_; r++)
        routePolarAngles.emplace_back(routes[r].polarAngleBarycenter, r);
    std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

    int pos = 0;
    for (int r = 0; r < preprocessor->route_cap_; r++)
    {
        indiv->chromR[r].clear();
        Node * node = depots[routePolarAngles[r].second].next;
        while (!node->isDepot)
        {
            indiv->chromT[pos] = node->cour;
            indiv->chromR[r].push_back(node->cour);
            node = node->next;
            pos++;
        }
    }

    indiv->evaluate_upper_cost();
}

LeaderLahc::LeaderLahc(Case* instance, Preprocessor* preprocessor) : instance(instance), preprocessor(preprocessor)
{
    random_engine = std::default_random_engine(preprocessor->params.seed);
    clients = std::vector < Node >(instance->num_customer_ + 1);
    routes = std::vector < Route >(preprocessor->route_cap_);
    depots = std::vector < Node >(preprocessor->route_cap_);
    depotsEnd = std::vector < Node >(preprocessor->route_cap_);

    for (int i = 0; i <= instance->num_customer_; i++)
    {
        clients[i].cour = i;
        clients[i].isDepot = false;
    }
    for (int i = 0; i < preprocessor->route_cap_; i++)
    {
        routes[i].cour = i;
        routes[i].depot = &depots[i];
        depots[i].cour = 0;
        depots[i].isDepot = true;
        depots[i].route = &routes[i];
        depotsEnd[i].cour = 0;
        depotsEnd[i].isDepot = true;
        depotsEnd[i].route = &routes[i];
    }
    for (int i = 1 ; i <= instance->num_customer_ ; i++) orderNodes.push_back(i);
}

