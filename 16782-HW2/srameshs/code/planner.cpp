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
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <thread>
#include <mutex>

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

// helper function to write results to a CSV file
void writeResultsToCSV(const string& planner, int planLength, double pathQuality, 
						double numVertices, float planningTime, string under_five, const vector<double>& start, 
						const vector<double>& goal, int numofDOFs) {
    
	ofstream outfile("../output/path_quality.csv", ios::app);

    // Check if the file exists and is empty
    bool isEmpty = false;
    ifstream checkFile("path_quality.csv");
    if (checkFile.good()) { // Check if file exists
        checkFile.seekg(0, ios::end); // Move to the end of the file
        isEmpty = (checkFile.tellg() == 0); // Check if file size is 0
    }
    checkFile.close(); // Close the file

    if (outfile.is_open()) {
        // Write header if the file is empty
        if (isEmpty) {
            outfile << "Planner,Path Length,Path Quality,numVertices,Planning Time,under_five,Start Position,End Position\n"; // Write the header
        }

        // Prepare start and goal positions as strings
        stringstream startStream, goalStream;
        for (int i = 0; i < numofDOFs; ++i) {
            startStream << start[i];
            goalStream << goal[i];
            if (i < numofDOFs - 1) {
                startStream << ";"; // Separate angles with a semicolon
                goalStream << ";";
            }
        }

        // Format the output to ensure it's CSV compliant
        outfile << planner << "," 
                << planLength << "," 
                << pathQuality << ","
				<< numVertices << ","
				<< planningTime << ","
				<< under_five << ","
                << "\"" << startStream.str() << "\"," // Quoting the start position
                << "\"" << goalStream.str() << "\"" // Quoting the end position
                << "\n"; // New line for the next entry

        outfile.close(); // Close the file
    } else {
        cerr << "Unable to open file!" << endl;
    }
}

// joint angles sum
double getPlanQuality(double*** plan, int* planlength, int numofDOFs) {
    double cost = 0;
    for (int i = 0; i < *planlength - 1; i++) {
        double* current = (*plan)[i];
        double* next = (*plan)[i+1];
        double diff = 0;
        for (int j = 0; j < numofDOFs; j++) {
            diff = abs(current[j] - next[j]);
		    diff = min(diff, 2*M_PI-diff);
        }
        cost += diff;
    }
    return cost;
}

/* Each node of the tree:
		* joint_angles: joint configuration of the arm 
		* parent: pointer to node's parent in tree
*/
struct Node{
	vector<double> joint_angles;
	Node* parent;
	double cost;

	Node(vector<double> joint_angles):
		joint_angles(joint_angles),
		parent(nullptr),
		cost(0.0)
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
		double goalThreshold = 1e-3; 

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
		void addEdge(Node* parent, Node* child, bool rrtStar = false);
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
		Node* buildRRTStar();
		pair<bool, Node*> extendRRTStar(vector<double>& qRand);
		vector<Node*> nearbyNodes(vector<double>& qNew, double radius);
		
};


void RRTAlgo::addChild(vector<double>& new_joint_angles, bool at_start){
	vector<Node*>& tree = at_start ? start_tree : goal_tree;
	Node* new_node = new Node(new_joint_angles);

	// add the node to the start tree
	tree.push_back(new_node);
}

void RRTAlgo::addEdge(Node* parent, Node* child, bool rrtStar){
	child->parent = parent;
	if(rrtStar){
		// cost = euclideanDistance
		child->cost = parent->cost + euclideanDistance(child->joint_angles, parent->joint_angles);
	}
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
	bool status = false;
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
			break;
		}
	}
	return status;
}

pair<bool, Node*> RRTAlgo::extendRRT(vector<double>& qRand, bool at_start){
	vector<Node*>& tree = at_start ? start_tree : goal_tree;
	// cout<<"size of the tree start?: "<<at_start<<"size:"<<tree.size()<<endl;

	Node* qNear = findNearestNode(qRand, at_start); //find the nearest node on the tree
	vector<double> qNew(numofDOFs, 0);

	// ADVANCED or REACHED; handled in newConfig
	if(newConfig(qRand, qNear->joint_angles, qNew)){
		addChild(qNew, at_start);
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

	auto start_time = chrono::high_resolution_clock::now();
	
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

	auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

	float planningTime = total_time.count();
    string under_five = (planningTime < 5000) ? "Yes" : "No";
	double pathQuality = getPlanQuality(plan, planlength, numofDOFs);
	double numVertices = rrt.start_tree.size() + rrt.goal_tree.size();

	cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
	cout << "Planning Time: " << planningTime << " milliseconds" << endl;
	cout << "Number of Vertices: " << numVertices << endl;
	cout << "Path Length: " << *planlength << endl;
	cout << "Path Quality: " << pathQuality << endl;

	// Write results to CSV
    writeResultsToCSV("RRT", *planlength, pathQuality, numVertices, planningTime, under_five, start, goal, numofDOFs);

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

Node* RRTAlgo::connectTree(vector<double>& qNew, bool at_start){
	pair<bool, Node*> S;

	while(true){
		S = extendRRT(qNew, at_start);
		if(S.first){
			/*REACHED*/
			if(euclideanDistance(S.second->joint_angles, qNew) <= goalThreshold){
				// cout<<"found the connect node"<<endl;
				return S.second;
			}
		}
		else{
			/*TRAPPED*/
			return nullptr;
		}
	};
}

pair<Node*,Node*> RRTAlgo::buildRRTConnect(){
	bool at_start = true;

	vector<double> qInit = start;
	vector<double> qGoal = goal;

	// add start & goal nodes to start_tree & goal_tree respectively
	addChild(qInit, true);
	addChild(qGoal, false);

	for(int k=0; k<K; k++){
		
		vector<double> qRand(numofDOFs, 0);

		double biasProbability = static_cast<double>(rand()) / RAND_MAX; // random value between 0 and 1

        // 10% bias towards the goal - CAN BE TUNED
        if (biasProbability <= 0.1) {
			// cout<<"i'm biased"<<endl;
            qRand = at_start ? qGoal : qInit;
        } else {
			// Generate random configuration between 0 and 2*pi
			for(int i=0; i<numofDOFs; i++){
				qRand[i] = ((double) rand() / (RAND_MAX)) * M_PI * 2;
			}
		}
		
		Node* resultNode = extendRRT(qRand, at_start).second; 

		// extend from the other tree until resultNode
		Node* connectNode = connectTree(resultNode->joint_angles, !at_start); 

		if(connectNode){
			// Always connect the start_tree to the goal_tree
			pair<Node*,Node*> complete = at_start ? make_pair(resultNode,connectNode) : make_pair(connectNode,resultNode);
			return complete;
		}

		// swap sides
		at_start = !at_start;
	}

	// could not find a path
	return make_pair(nullptr, nullptr);

}

void RRTAlgo::retraceRRTConnectPath(Node* startNode, Node* goalNode, double ***plan, int *planlength){

	vector<Node*> startPath, goalPath;

	// find path length by backtracking from connectNode to startNode
    Node* current = startNode;
    while (current != nullptr) {
		startPath.push_back(current);
        current = current->parent;
    }
	
	// find path length by backtracking from connectNode to goalNode
	current = goalNode;
    while (current != nullptr) {
		goalPath.push_back(current);
        current = current->parent;
    }

	*planlength = startPath.size() + goalPath.size();

	// extract the path
	*plan = (double**) malloc(*planlength*sizeof(double));
	if (*plan == nullptr) {
        *planlength = 0;
        return; // Handle allocation failure
    }

	 // Pointer to the current position in the plan
    double** plan_ptr = *plan;

    // Add startPath in reverse order (from start to connectNode)
    for (int i = startPath.size() - 1; i >= 0; i--) {
        *plan_ptr = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan_ptr)[j] = startPath[i]->joint_angles[j];
        }
        plan_ptr++;  // Move the pointer to the next element in the plan
    }

    // Add goalPath (from connectNode to goal)
    for (Node* node : goalPath) {
        *plan_ptr = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan_ptr)[j] = node->joint_angles[j];
        }
        plan_ptr++;  // Move the pointer to the next element in the plan
    }
}


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
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	auto start_time = chrono::high_resolution_clock::now();

	// defining start & goal configuration of the arm
	vector<double> start; start.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goal; goal.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);


	int numOfIterations = 100000;
	double epsilon = 0.5;
	
	RRTAlgo rrt(start, goal, numOfIterations, epsilon, map, numofDOFs, x_size, y_size);

	// start RRT!!
	pair<Node*, Node*> result = rrt.buildRRTConnect();

	// retrace & extract path
	if(result.first && result.second){
		rrt.retraceRRTConnectPath(result.first, result.second, plan, planlength);
	}
	else{
		cout<<"No path found!"<<endl;
	}

	auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

	float planningTime = total_time.count();
	string under_five = (planningTime < 5000) ? "Yes" : "No";
	double pathQuality = getPlanQuality(plan, planlength, numofDOFs);
	double numVertices = rrt.start_tree.size() + rrt.goal_tree.size();

	cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
	cout << "Planning Time: " << planningTime << " milliseconds" << endl;
	cout << "Number of Vertices: " << numVertices << endl;
	cout << "Path Length: " << *planlength << endl;
	cout << "Path Quality: " << pathQuality << endl;

	// Write results to CSV
    writeResultsToCSV("RRTConnect", *planlength, pathQuality, numVertices, planningTime, under_five, start, goal, numofDOFs);

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<Node*> RRTAlgo::nearbyNodes(vector<double>& qNew, double radius){
	// vector storing the nearyby nodes
	vector<Node*> nodesNear;

	for(Node* node : start_tree) {
		if(euclideanDistance(node->joint_angles, qNew) < radius){
			nodesNear.push_back(node);
		}
	}
	return nodesNear;
}

pair<bool, Node*> RRTAlgo::extendRRTStar(vector<double>& qRand){

	Node* qNear = findNearestNode(qRand); //find the nearest node on the tree
	vector<double> qNew(numofDOFs, 0);

	// ADVANCED or REACHED; handled in newConfig
	if(newConfig(qRand, qNear->joint_angles, qNew)){
		// TODO: START HERE
		double minCost = qNear->cost + euclideanDistance(qNear->joint_angles, qNew);
		Node* qMin = qNear;

		// check for nodes nearby
		auto nodesNear = nearbyNodes(qNew, 10);

		for(Node* neighbourNode : nodesNear){
			double cost = neighbourNode->cost + euclideanDistance(neighbourNode->joint_angles, qNew);
			if (cost < minCost){
				minCost = cost;
				qMin = neighbourNode;
			}
		}

		addChild(qNew);
		Node* qNewNode = start_tree.back();
		qNewNode->cost = minCost;
		addEdge(qNear, qNewNode, true);

		// rewire the tree
		for (Node* neighbourNode : nodesNear){
			double cost = qNewNode->cost + euclideanDistance(qNewNode->joint_angles, neighbourNode->joint_angles);

			if(cost < neighbourNode->cost){
				neighbourNode->parent = qNewNode;
				neighbourNode->cost = cost;
			}
		}
		
		return make_pair(true, qNewNode);
	}
	// TRAPPED; handled in newConfig
	else{
		return make_pair(false, qNear);
	}
}

Node* RRTAlgo::buildRRTStar(){
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
		
		auto result = extendRRTStar(qRand).second;

		if(result && euclideanDistance(result->joint_angles, qGoal) <= goalThreshold){
			// cout<<"found a solution"<<endl;
			return result;
		}
	}

	// could not find a path
	return nullptr;
	
}

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
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	auto start_time = chrono::high_resolution_clock::now();

	// defining start & goal configuration of the arm
	vector<double> start; start.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goal; goal.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);


	int numOfIterations = 100000;
	double epsilon = 0.5;

	RRTAlgo rrt(start, goal, numOfIterations, epsilon, map, numofDOFs, x_size, y_size);

	// start RRT!!
	Node* result = rrt.buildRRTStar();

	// retrace & extract path
	if(result){
		rrt.retraceRRTPath(result, plan, planlength);
	}
	else{
		cout<<"No path found!"<<endl;
	}

	auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

	float planningTime = total_time.count();
	string under_five = (planningTime < 5000) ? "Yes" : "No";
	double pathQuality = getPlanQuality(plan, planlength, numofDOFs);
	double numVertices = rrt.start_tree.size() + rrt.goal_tree.size();

	cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
	cout << "Planning Time: " << planningTime << " milliseconds" << endl;
	cout << "Number of Vertices: " << numVertices << endl;
	cout << "Path Length: " << *planlength << endl;
	cout << "Path Quality: " << pathQuality << endl;

	// Write results to CSV
    writeResultsToCSV("RRTStar", *planlength, pathQuality, numVertices, planningTime, under_five, start, goal, numofDOFs);

    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

// function to get euclidean distance between two joint configs
double calculateDistance(const double* node1, const double* node2, int numofDOFs) {
    double sum = 0.0;
    for (int i = 0; i < numofDOFs; ++i) {
        double diff = abs(node1[i] - node2[i]);
        sum += pow(min(diff, 2 * M_PI - diff), 2);
    }
    return sqrt(sum);
}

// neighbourhood points
vector<double*> findNearbyNodes(double radius, double* vertex, unordered_map<int, double*> nodes, int numofDOFs){
	vector<double*> nearbyNodes;
    for (const auto& n : nodes) {
        if (calculateDistance(vertex, n.second, numofDOFs) <= radius) {
            nearbyNodes.push_back(n.second);
        }
    }
    return nearbyNodes;
}

bool isValidEdge(const double* start, 
               const double* end, 
               int numofDOFs, 
               int steps,
               double* map,
               int x_size,
               int y_size)
{
    // vector to store the interpolated configuration
	vector<double> config(numofDOFs); 

    for (int i = 0; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps;

        // interpolation calculation
        for (int j = 0; j < numofDOFs; ++j) {
            config[j] = start[j] + alpha * (end[j] - start[j]);
        }

        // Check if this interpolated configuration is valid
        if (!IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size)) {
            return false;
        }
    }
    return true;
}

int lookupNodeIndex(const unordered_map<int, double*>& nodes, double* node){
	for (const auto& n: nodes){
		if (n.second == node){
			return n.first;
		}
	}
	cout << "node not found" << endl;
	return -1; 
}

static void addEdge(unordered_map<int, unordered_set<int>>& edges, int alpha_i, int q_i) {
    if (alpha_i == q_i) {
        return;  // Avoid self-loop
    }
    
    // Add bidirectional edges between alpha_i and q_i
    edges[alpha_i].emplace(q_i);
    edges[q_i].emplace(alpha_i);
}

void connectClosest(double* vertex, 
                    unordered_map<int, double*>& nodes, 
                    int numofDOFs, 
                    unordered_map<int, unordered_set<int>>& edges, 
                    int index, 
                    bool start) 
{
    // Find the closest node
    auto closest = min_element(nodes.begin(), nodes.end(), 
        [vertex, numofDOFs](const auto& a, const auto& b) {
            return calculateDistance(vertex, a.second, numofDOFs) < calculateDistance(vertex, b.second, numofDOFs);
        });
    
    int closestIndex = closest->first;

    // Add the edge based on the 'start' condition
    addEdge(edges, start ? index : closestIndex, start ? closestIndex : index);

    // Insert the new node
    nodes[index] = vertex;
}

/*
	A-Star
*/

struct AStarNode {
    int index;
    double cost; 
    double heuristic;
    int parent;
};

struct CompareAStarNode {
    bool operator()(const AStarNode& n1, const AStarNode& n2) const {
        return (n1.cost + n1.heuristic) > (n2.cost + n2.heuristic);
    }
};

// Function to search the graph using the A* algorithm
vector<int> searchGraph(int startIndex, 
                        int goalIndex,
                        const unordered_map<int, unordered_set<int>>& edges,
                        const unordered_map<int, double*>& nodes,
                        int numofDOFs) {

    // Check for existence of start and goal nodes
    if (nodes.find(startIndex) == nodes.end() || nodes.find(goalIndex) == nodes.end()) {
        cerr << "Error: Start or goal node does not exist." << endl;
        return {};
    }

    using Node = AStarNode; // For clarity
    priority_queue<Node, vector<Node>, CompareAStarNode> openList;
    unordered_set<int> closedList;
    unordered_map<int, Node> parentMap;

    // Initialize the open list with the starting node
    openList.push({startIndex, 0, calculateDistance(nodes.at(startIndex), nodes.at(goalIndex), numofDOFs), -1});

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // Check if the goal has been reached
        if (current.index == goalIndex) {
            cout << "FOUND GOAL" << endl;
            vector<int> plan;

            // Backtrack to construct the path
            while (current.parent != -1) {
                plan.push_back(current.index);
                current = parentMap[current.parent];
            }
            plan.push_back(startIndex);
            reverse(plan.begin(), plan.end());
            return plan;
        }

        // Skip already-visited nodes
        if (!closedList.insert(current.index).second) {
            continue;
        }

        parentMap[current.index] = current;

        // Explore neighbors
        auto it = edges.find(current.index);
        if (it != edges.end()) {
            for (int neighbor : it->second) {
                if (closedList.count(neighbor) == 0 && nodes.count(neighbor) > 0) {
                    double newCost = current.cost + calculateDistance(nodes.at(current.index), nodes.at(neighbor), numofDOFs);
                    double heuristic = calculateDistance(nodes.at(neighbor), nodes.at(goalIndex), numofDOFs);
                    openList.push({neighbor, newCost, heuristic, current.index});
                }
            }
        }
    }

    return {}; // Return an empty vector if no path is found
}

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
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	vector<double> start; start.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
	vector<double> goal; goal.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

	auto start_time = chrono::high_resolution_clock::now();

    int iter = 0;
    int steps = 10;
    double radius = 5;
    unordered_map<int, unordered_set<int>> edges;
    unordered_map<int, double*> nodes;

    while (iter < 1000){
        double* alpha = new double[numofDOFs];

        // Generate random configuration between 0 and 2*pi
        for(int i=0; i<numofDOFs; i++){
            alpha[i] = ((double) rand() / (RAND_MAX + 1.0)) * M_PI * 2;
        }

        if (IsValidArmConfiguration(alpha, numofDOFs, map, x_size, y_size)){
            nodes.insert(make_pair(iter, alpha));

            // check neighbourhood points
            vector<double*> neighbors = findNearbyNodes(radius, alpha, nodes, numofDOFs);

			// add edges
    		for (const auto& q : neighbors) {
				if (isValidEdge(alpha, q, numofDOFs, steps, map, x_size, y_size)){
					int q_i = lookupNodeIndex(nodes, q);
                    if (edges[iter].size() < 10){
                        addEdge(edges, q_i, iter);
                    }
				}
			}
			iter++;
        }
    }

	// connect closest nodes to start and goal
    int startIndex = iter+1;
    int goalIndex = iter+2;
    connectClosest(armstart_anglesV_rad, nodes, numofDOFs, edges, startIndex, true);
    connectClosest(armgoal_anglesV_rad, nodes, numofDOFs, edges, goalIndex, false);

    // search graph using A*
    vector<int> pathIndices = searchGraph(startIndex, goalIndex, edges, nodes, numofDOFs);

	// planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);

	// populate plan
    if (!pathIndices.empty()) {
        *planlength = pathIndices.size();
        *plan = (double**) malloc(*planlength * sizeof(double*));
        
        for (int i = 0; i < *planlength; ++i) {
            (*plan)[i] = nodes[pathIndices[i]];
        }
    }
	else {
        cout << "Failed to find a path." << endl;
    }

	// full solution time
    auto end_time = chrono::high_resolution_clock::now();
    auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

	float planningTime = planning_time.count();
	double pathQuality = getPlanQuality(plan, planlength, numofDOFs);
	double numVertices = nodes.size();

	string under_five = (total_time.count() < 5000) ? "Yes" : "No";
	cout << "Generated Solution in Under Five Seconds: " << under_five << endl;
	cout << "Planning Time: " << planningTime  << " milliseconds" << endl;
	cout << "Number of Vertices: " << numVertices << endl;
	cout << "Path Length: " << *planlength << endl;
	cout << "Path Quality: " << pathQuality << endl;

	// Write results to CSV
    writeResultsToCSV("PRM", *planlength, pathQuality, numVertices, planningTime, under_five, start, goal, numofDOFs);
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
	srand(static_cast<unsigned int>(time(0)));

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