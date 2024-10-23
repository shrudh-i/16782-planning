/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <limits>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

using namespace std;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

/* Each node of the tree:
		* joint_angles: joint configuration of the arm 
		* parent: pointer to node's parent in tree
*/
struct Node{
	vector<double> joint_angles;
	Node* parent;

	Node(vector<double> joint_angles):
		joint_angles(joint_angles),
		parent(nullptr)
	{};
};


class RRTAlgo{
	public:
		/* tree from start node */
		vector<Node*> start_tree;

		/* tree from goal node */
		vector<Node*> goal_tree;

		vector<double> start; // start configuration
		vector<double> goal; // goal configuration
		int K; // numofIterations
		double epsilon; //step size
		double *map;
		int numofDOFs;
		int x_size;
    	int y_size;
		double goalThreshold = 1e-2; 

		RRTAlgo(vector<double> start, vector<double> goal, int K, double epsilon, double* map, int numofDOFs, int x_size, int y_size):
			start(start),
			goal(goal),
			K(K),
			epsilon(epsilon),
			map(map),
			numofDOFs(numofDOFs),
			x_size(x_size),
			y_size(y_size)
		{}

		/* Class Functions: */
		void addChild(vector<double>& new_joint_angles, bool at_start=true);
		void addEdge(Node* parent, Node* child);
		Node* findNearestNode(vector<double>& qRand, bool at_start=true);
		double euclideanDistance(vector<double> node, vector<double> q);
		bool newConfig(vector<double>& qRand, vector<double>& qNear, vector<double>& qNew);

		/* RRT */
		Node* buildRRT();
		pair<bool, Node*> extendRRT(vector<double>& qRand, bool at_start=true);
		void retraceRRTPath(Node* result, double ***plan, int *planlength);

		/* RRT Connect */
		Node* connectTree(vector<double>& qNew, bool at_start);
		pair<Node*,Node*> buildRRTConnect();
		void retraceRRTConnectPath(Node* startNode, Node* goalNode, double ***plan, int *planlength);


		/* RRT Star */
};


void RRTAlgo::addChild(vector<double>& new_joint_angles, bool at_start){
	vector<Node*>& tree = at_start ? start_tree : goal_tree;
	Node* new_node = new Node(new_joint_angles);

	// add the node to the start tree
	tree.push_back(new_node);
}

void RRTAlgo::addEdge(Node* parent, Node* child){
	child->parent = parent;
}

double RRTAlgo::euclideanDistance(vector<double> node, vector<double> q){
	double distance = 0.0;
	
	for(size_t i = 0; i<numofDOFs; i++){
		distance += pow(node[i] - q[i], 2);
	}

	return sqrt(distance);
}

Node* RRTAlgo::findNearestNode(vector<double>& qRand, bool at_start){
	vector<Node*>& tree = at_start ? start_tree : goal_tree;
	Node* nearest = nullptr;
	// init: set the minimum distance to a very high value
	double minDist = numeric_limits<double>::max();

	// iterate through the tree 
	for(Node* n : tree){
		// double dist = 0;
		double dist = euclideanDistance(n->joint_angles, qRand);
		
		// set the new minimum distance and nearest node
		if (dist < minDist){
			minDist = dist;
			nearest = n;
		}
	}

	return nearest;
}

bool RRTAlgo::newConfig(vector<double>& qRand, vector<double>& qNear, vector<double>& qNew){
	/* Return the status of 
			true: ADVANCED, REACHED; 
			false: TRAPPED
	*/
	bool status;
	double temp_joint_angles[numofDOFs];
	double dist = euclideanDistance(qNear, qRand);

	// Scaling factor: helps control how far to move towards the target configuration
	double stepSize = MIN(epsilon, dist)/dist;

	// Interpolate towards qNear; delta steps
	for (double delta = stepSize; delta <= 1.0; delta+=stepSize){
		/*
			scales the difference between the target and current configuration by alpha
		*/
		for(size_t i=0; i<numofDOFs; i++){
			temp_joint_angles[i] = qNear[i] + delta*(qRand[i] - qNear[i]);
		}

		// update qNew & keep going; ADVANCED or REACHED
		if(IsValidArmConfiguration(temp_joint_angles, numofDOFs, map, x_size, y_size)){
			// copy the temporary configuration to qNew
			copy(temp_joint_angles, temp_joint_angles + numofDOFs, qNew.begin());
			status = true;
		}
		// stop if we hit a collision; TRAPPED
		else{
			status = false;
			break;
		}
	}

	return status;
}

pair<bool, Node*> RRTAlgo::extendRRT(vector<double>& qRand, bool at_start){
	vector<Node*>& tree = at_start ? start_tree : goal_tree;

	Node* qNear = findNearestNode(qRand); //find the nearest node on the tree
	vector<double> qNew(numofDOFs, 0);

	// ADVANCED or REACHED; handled in newConfig
	if(newConfig(qRand, qNear->joint_angles, qNew)){
		addChild(qNew);
		Node* qNewNode = tree.back();
		addEdge(qNear, qNewNode);
		return make_pair(true, qNewNode);
	}
	// TRAPPED; handled in newConfig
	else{
		return make_pair(false, qNear);
	}
}

/*
	inputs: K - number of iterations that RRT should run for
	outputs: final tree node
*/
Node* RRTAlgo::buildRRT(){
	vector<double> qInit = start;
	vector<double> qGoal = goal;

	addChild(qInit);

	for(int k=0; k<K; k++){
		
		vector<double> qRand(numofDOFs, 0);

		double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1

        // 10% bias towards the goal - CAN BE TUNED
        if (biasProbability <= 0.1) {
			// cout<<"i'm biased"<<endl;
            qRand = qGoal;
        } else {
			// Generate random configuration between 0 and 2*pi
			for(int i=0; i<numofDOFs; i++){
				qRand[i] = ((double) rand() / (RAND_MAX + 1.0)) * M_PI * 2;
			}
		}
		
		auto result = extendRRT(qRand).second;
		if(euclideanDistance(result->joint_angles, qGoal) <= goalThreshold){
			// cout<<"found a solution"<<endl;
			return result;
		}
	}

	// could not find a path
	return nullptr;
}

void RRTAlgo::retraceRRTPath(Node* result, double ***plan, int *planlength){

	// find path length by backtracking from the goal
    int len = 0;
    Node* current = result;
    while (current != nullptr) {
        len++;
        current = current->parent;
    }

	// extract the path
	current = result;
	*plan = (double**) malloc(len*sizeof(double));
	if (*plan == nullptr) {
        *planlength = 0;
        return; // Handle allocation failure
    }

	for (int i = len - 1; i >= 0; i--) {
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
		
		// Handle allocation failure; free previously allocated memory
        if ((*plan)[i] == nullptr) {
            for (int k = len - 1; k > i; k--) {
                free((*plan)[k]);
            }
            free(*plan);
            *planlength = 0;
            return;
        }
        
		// Copy values manually
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[i][j] = current->joint_angles[j];
        }
        current = current->parent;
    }

    *planlength = len;

}

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	// planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
	
	// defining start & goal configuration of the arm
	vector<double> start; start.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goal; goal.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);


	int numOfIterations = 100000;
	double epsilon = 0.5;

	RRTAlgo rrt(start, goal, numOfIterations, epsilon, map, numofDOFs, x_size, y_size);

	// start RRT!!
	Node* result = rrt.buildRRT();

	// retrace & extract path
	if(result){
		rrt.retraceRRTPath(result, plan, planlength);
	}
	else{
		cout<<"No path found!"<<endl;
	}

	cout << "Path Length: " << *planlength << endl;

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

// Node* RRTAlgo::connectTree(vector<double>& qNew, bool at_start){
// 	pair<bool, Node*> S;

// 	while(true){
// 		S = extendRRT(qNew, at_start);
// 		if(S.first){
// 			/*REACHED*/
// 			if(euclideanDistance(S.second->joint_angles, qNew) <= goalThreshold){
// 			// cout<<"found the connect node"<<endl;
// 			return S.second;
// 			}
// 		}
// 		else{
// 			/*TRAPPED*/
// 			return nullptr;
// 		}
// 	};
// }

// pair<Node*,Node*> RRTAlgo::buildRRTConnect(){
// 	bool at_start = true;

// 	vector<double> qInit = start;
// 	vector<double> qGoal = goal;

// 	// add start & goal nodes to start_tree & goal_tree respectively
// 	addChild(qInit, true);
// 	addChild(qGoal, false);

// 	for(int k=0; k<K; k++){
		
// 		vector<double> qRand(numofDOFs, 0);

// 		double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1

//         // 10% bias towards the goal - CAN BE TUNED
//         if (biasProbability <= 0.1) {
// 			// cout<<"i'm biased"<<endl;
//             qRand = at_start ? qGoal : qInit;
//         } else {
// 			// Generate random configuration between 0 and 2*pi
// 			for(int i=0; i<numofDOFs; i++){
// 				qRand[i] = ((double) rand() / (RAND_MAX + 1.0)) * M_PI * 2;
// 			}
// 		}
		
// 		Node* resultNode = extendRRT(qRand, at_start).second; 

// 		// extend from the other tree until resultNode
// 		Node* connectNode = connectTree(resultNode->joint_angles, !at_start); 

// 		if(connectNode){
// 			// Always connect the start_tree to the goal_tree
// 			pair<Node*,Node*> complete = at_start ? make_pair(resultNode,connectNode) : make_pair(connectNode,resultNode);
// 			return complete;
// 		}

// 		// swap sides
// 		at_start = ! at_start;
// 	}

// 	// could not find a path
// 	return make_pair(nullptr, nullptr);

// }

// void RRTAlgo::retraceRRTConnectPath(Node* startNode, Node* goalNode, double ***plan, int *planlength){

// 	vector<Node*> startPath, goalPath;

// 	// find path length by backtracking from connectNode to startNode
//     Node* current = startNode;
//     while (current != nullptr) {
// 		startPath.push_back(current);
//         current = current->parent;
//     }
	
// 	// find path length by backtracking from connectNode to goalNode
// 	current = goalNode;
//     while (current != nullptr) {
// 		goalPath.push_back(current);
//         current = current->parent;
//     }

// 	*planlength = startPath.size() + goalPath.size();

// 	// extract the path
// 	*plan = (double**) malloc(*planlength*sizeof(double));
// 	if (*plan == nullptr) {
//         *planlength = 0;
//         return; // Handle allocation failure
//     }

// 	 // Pointer to the current position in the plan
//     double** plan_ptr = *plan;

//     // Add startPath in reverse order (from start to connectNode)
//     for (int i = startPath.size() - 1; i >= 0; i--) {
//         *plan_ptr = (double*) malloc(numofDOFs * sizeof(double));
//         for (int j = 0; j < numofDOFs; j++) {
//             (*plan_ptr)[j] = startPath[i]->joint_angles[j];
//         }
//         plan_ptr++;  // Move the pointer to the next element in the plan
//     }

//     // Add goalPath (from connectNode to goal)
//     for (Node* node : goalPath) {
//         *plan_ptr = (double*) malloc(numofDOFs * sizeof(double));
//         for (int j = 0; j < numofDOFs; j++) {
//             (*plan_ptr)[j] = node->joint_angles[j];
//         }
//         plan_ptr++;  // Move the pointer to the next element in the plan
//     }
// }


static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);


}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
