//
// Created by Yinghao Qin on 20/02/2025.
//

#ifndef FROGS_LEADER_LAHC_HPP
#define FROGS_LEADER_LAHC_HPP

#include "individual.hpp"

struct Node ;

// Structure containing a route
struct Route
{
    int cour;							// Route index
    int nbCustomers;					// Number of customers visited in the route
    int whenLastModified;				// "When" this route has been last modified
    int whenLastTestedSWAPStar;			// "When" the SWAP* moves for this route have been last tested
    Node * depot;						// Pointer to the associated depot
    double duration;					// Total time on the route
    double load;						// Total load on the route
    double reversalDistance;			// Difference of cost if the route is reversed
    double penalty;						// Current sum of load and duration penalties
    double polarAngleBarycenter;		// Polar angle of the barycenter of the route
    CircleSector sector;				// Circle sector associated to the set of customers
};

struct Node
{
    bool isDepot;						// Tells whether this node represents a depot or not
    int cour;							// Node index
    int position;						// Position in the route
    int whenLastTestedRI;				// "When" the RI moves for this node have been last tested
    Node * next;						// Next node in the route order
    Node * prev;						// Previous node in the route order
    Route * route;						// Pointer towards the associated route
    double cumulatedLoad;				// Cumulated load on this route until the customer (including itself)
    double cumulatedTime;				// Cumulated time on this route until the customer (including itself)
    double cumulatedReversalDistance;	// Difference of cost if the segment of route (0...cour) is reversed (useful for 2-opt moves with asymmetric problems)
    double deltaRemoval;				// Difference of cost in the current route if the node is removed (used in SWAP*)
};

// Main local search structure
class LeaderLahc
{

private:

    Case* instance;                             // Problem instance information
    Preprocessor* preprocessor;                 // Preprocessed data
    std::default_random_engine random_engine;   // Random number generator
    bool searchCompleted;						// Tells whether all moves have been evaluated without success
    int nbMoves;								// Total number of moves (RI and SWAP*) applied during the local search. Attention: this is not only a simple counter, it is also used to avoid repeating move evaluations
    std::vector < int > orderNodes;				// Randomized order for checking the nodes in the RI local search
    std::set < int > emptyRoutes;				// indices of all empty routes
    int loopID;									// Current loop index

    /* THE SOLUTION IS REPRESENTED AS A LINKED LIST OF ELEMENTS */
    std::vector < Node > clients;				// Elements representing clients (clients[0] is a sentinel and should not be accessed)
    std::vector < Node > depots;				// Elements representing depots
    std::vector < Node > depotsEnd;				// Duplicate of the depots to mark the end of the routes
    std::vector < Route > routes;				// Elements representing routes

    /* TEMPORARY VARIABLES USED IN THE LOCAL SEARCH LOOPS */
    // nodeUPrev -> nodeU -> nodeX -> nodeXNext
    // nodeVPrev -> nodeV -> nodeY -> nodeYNext
    Node * nodeU ;
    Node * nodeX ;
    Node * nodeV ;
    Node * nodeY ;
    Route * routeU ;
    Route * routeV ;
    int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex ;
    int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex ;
    double loadU, loadX, loadV, loadY;
    double serviceU, serviceX, serviceV, serviceY;
    double penaltyCapacityLS, penaltyDurationLS ;

    void setLocalVariablesRouteU(); // Initializes some local variables and distances associated to routeU to avoid always querying the same values in the distance matrix
    void setLocalVariablesRouteV(); // Initializes some local variables and distances associated to routeV to avoid always querying the same values in the distance matrix

    // Functions in charge of excess load and duration penalty calculations
    // #define penaltyExcessDuration(x) _penaltyExcessDuration(x)
#define penaltyExcessDuration(x) 0. // <--- Use this line instead of the previous one to save some CPU time if your problem does not include duration constraints
    inline double _penaltyExcessDuration(double myDuration) {return std::max<double>(0., myDuration - instance->max_service_time_)*penaltyDurationLS;}
    inline double penaltyExcessLoad(double myLoad) {return std::max<double>(0., myLoad - instance->max_vehicle_capa_)*penaltyCapacityLS;}

    /* RELOCATE MOVES */
    // (Legacy notations: move1...move9 from Prins 2004)
    bool move1 (); // If U is a client node, remove U and insert it after V
    bool move2 (); // If U and X are client nodes, remove them and insert (U,X) after V
    bool move3 (); // If U and X are client nodes, remove them and insert (X,U) after V

    /* SWAP MOVES */
    bool move4 (); // If U and V are client nodes, swap U and V
    bool move5 (); // If U, X and V are client nodes, swap (U,X) and V
    bool move6 (); // If (U,X) and (V,Y) are client nodes, swap (U,X) and (V,Y)

    /* 2-OPT and 2-OPT* MOVES */
    bool move7 (); // If route(U) == route(V), replace (U,X) and (V,Y) by (U,V) and (X,Y)
    bool move8 (); // If route(U) != route(V), replace (U,X) and (V,Y) by (U,V) and (X,Y)
    bool move9 (); // If route(U) != route(V), replace (U,X) and (V,Y) by (U,Y) and (V,X)

    /* ROUTINES TO UPDATE THE SOLUTIONS */
    static void insertNode(Node * U, Node * V);		// Solution update: Insert U after V
    static void swapNode(Node * U, Node * V) ;		// Solution update: Swap U and V
    void updateRouteData(Route * myRoute);			// Updates the preprocessed data of a route

public:

    // Run the local search with the specified penalty values
    void run(Individual * indiv, double penaltyCapacityLS, double penaltyDurationLS);

    // Loading an initial solution into the local search
    void loadIndividual(Individual * indiv);

    // Exporting the LS solution into an individual and calculating the penalized cost according to the original penalty weights from Params
    void exportIndividual(Individual * indiv);

    // Constructor
    LeaderLahc(Case* instance, Preprocessor* preprocessor);
};


#endif //FROGS_LEADER_LAHC_HPP
