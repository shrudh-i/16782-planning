/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

#include <queue>
#include <stack>
#include <chrono>
#include <math.h>
#include <climits>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

/*queue node structure*/
struct node {
    int map_index;
    int g;
    int h;
    int f;
    int time;
    node* parent;

    node() : parent(nullptr), g(INT_MAX), f(INT_MAX) {}
    node(int ind, int hVal, int t) : parent(nullptr), g(INT_MAX), f(INT_MAX) {
        map_index = ind;
        h = hVal;
        time = t;
    }
};

/*comparator for the priority queue*/
struct compareNode {
    bool operator()(const node* n1, const node* n2) {
        return n1->f > n2->f;
    }
};

// Globals
int prevX, prevY;
bool firstCall = true;
stack<int> action_stack;
unordered_set<int> closed;
unordered_map<int, int> goals;
unordered_map<int, node*> nodes;
unordered_map<int, pair<int, int>> heuristics;
chrono::time_point<chrono::steady_clock> startTime;
priority_queue<node*, vector<node*>, compareNode> openQueue;

/*backward dijkstra's heuristic*/
void computeHeuristics(int x_size, int y_size, int dX[], int dY[], int* map, int collision_thresh) {
    while (!openQueue.empty()) {
        node* s = openQueue.top();
        openQueue.pop();
        if (heuristics.find(s->map_index) != heuristics.end()) continue;
        heuristics[s->map_index] = make_pair(s->g, s->time);

        int rX = (s->map_index % x_size) + 1;
        int rY = (s->map_index / x_size) + 1;

        for (int dir = 0; dir < NUMOFDIRS; ++dir) {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int new_index = GETMAPINDEX(newx, newy, x_size, y_size);

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && heuristics.find(new_index) == heuristics.end()) {
                int cost = map[new_index];
                if (cost >= 0 && cost < collision_thresh) {
                    if (nodes.find(new_index) == nodes.end()) {
                        node* n = new node(new_index, 0, s->time);
                        nodes[new_index] = n;
                    }
                    if (nodes[new_index]->g > s->g + cost) {
                        nodes[new_index]->g = s->g + cost;
                        nodes[new_index]->f = nodes[new_index]->g + nodes[new_index]->h;
                        nodes[new_index]->parent = s;
                        openQueue.push(nodes[new_index]);
                    }
                }
            }
        }
    }
}

/*function to compute the path*/
void computePath(int x_size, int y_size, int dX[], int dY[], int* map, int collision_thresh, int* target_traj, int target_steps, int robotposeX, int robotposeY, int targetposeX, int targetposeY) {
    while (!openQueue.empty()) {
        int timeElapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - startTime).count();
        node* s = openQueue.top();
        openQueue.pop();

        int spacing = (s->time == 0) ? 0 : (int)(log10(s->time) + 1);
        int new_index_4map = s->map_index * ((int)pow(10, spacing)) + s->time;

        if (closed.find(new_index_4map) != closed.end()) continue;
        closed.insert(new_index_4map);

        int rX = (s->map_index % x_size) + 1;
        int rY = (s->map_index / x_size) + 1;

        /*backtrack and push out actions*/
        if (goals.find(s->map_index) != goals.end() && s->time == (goals[s->map_index] - timeElapsed)) {
            while (s) {
                action_stack.push(s->map_index);
                s = s->parent;
            }
            action_stack.pop(); //remove the start node
            return;
        }

        int time = s->time + 1;
        if (time > target_steps) {
            continue;
        }

        for (int dir = 0; dir < (NUMOFDIRS + 1); ++dir) {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int new_index = GETMAPINDEX(newx, newy, x_size, y_size);
            int spacing = (time == 0) ? 0 : (int)(log10(time) + 1);
            int new_index_4map = new_index * ((int)pow(10, spacing)) + time;

            // Check if the new coordinates are within valid boundaries
            if (newx < 1 || newx > x_size || newy < 1 || newy > y_size) {
                continue; // Skip invalid moves
            }

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && closed.find(new_index_4map) == closed.end()) {
                int cost = map[new_index];

                if (cost < collision_thresh) {
                    if (nodes.find(new_index_4map) == nodes.end()) {
                        int totalTime = timeElapsed + time;
                        int h = (goals.find(new_index) != goals.end() && totalTime <= goals[new_index]) ? cost * (goals[new_index] - totalTime) : heuristics[new_index].first +abs(heuristics[new_index].second - totalTime);
                        node* n = new node(new_index, h, time);
                        nodes[new_index_4map] = n;
                    }
                    if (nodes[new_index_4map]->g > s->g + cost) {
                        nodes[new_index_4map]->g = s->g + cost;
                        nodes[new_index_4map]->f = nodes[new_index_4map]->g + 1.2 * nodes[new_index_4map]->h;
                        nodes[new_index_4map]->parent = s;
                        openQueue.push(nodes[new_index_4map]);
                    }
                }
            }
        }
    }
}

void planner(int* map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps, int* target_traj, int targetposeX, int targetposeY, int curr_time, int* action_ptr) {
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    prevX = robotposeX;
    prevY = robotposeY;

    if (firstCall) {
        startTime = chrono::steady_clock::now();
        firstCall = false;

        for (int i = 0; i < target_steps; ++i) {
            int gIndex = GETMAPINDEX(target_traj[i], target_traj[target_steps + i], x_size, y_size);
            /*excluding the last time step it takes to reach the target*/
            goals[gIndex] = MAX(0, i - 1);

            if (i > (target_steps / 2)) {
                node* a = new node(gIndex, 0, MAX(0, i - 1));
                a->g = 0;
                a->f = a->g + a->h;
                nodes[gIndex] = a;
                openQueue.push(a);
            }
        }

        computeHeuristics(x_size, y_size, dX, dY, map, collision_thresh);
        nodes.clear();

        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int h = heuristics[index].first;
        node* b = new node(index, h, 0);
        b->g = 0;
        b->f = b->g + b->h;
        nodes[index] = b;
        openQueue.push(b);
        computePath(x_size, y_size, dX, dY, map, collision_thresh, target_traj, target_steps, robotposeX, robotposeY, targetposeX, targetposeY);
    }

    if (!action_stack.empty()) {
        int nextIndex = action_stack.top();
        action_stack.pop();
        prevX = (nextIndex % x_size) + 1;
        prevY = (nextIndex / x_size) + 1;
    }

    action_ptr[0] = prevX;
    action_ptr[1] = prevY;

    return;
}
