#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <chrono>
#include <math.h>


#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;


class GroundedCondition
{
    string predicate; //name of the condition
    list<string> arg_values; //argument values
    bool truth = true; //condition negated?

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (const string& l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (const string& l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
    string predicate; //name of the condition
    list<string> args; // arguments
    bool truth; //condition negated?

public:
    Condition(const string &pred, const list<string>& args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string& ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
    string name; //name of the action
    list<string> args; // action arguments
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions; //preconditions
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects; //effects

public:
    Action(const string &name, const list<string>& args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (const string& l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition& pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition& pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition& precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition& effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string& l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string& symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string>& symbols)
    {
        for (const string& l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action& action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string& name) const {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (const string& s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition& s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition& g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action& g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

class GroundedAction
{
    string name; //name of the action
    list<string> arg_values; //action arguments

public:
    GroundedAction(const string &name, const list<string>& arg_values)
    {
        this->name = name;
        for (const string& ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

/*
    Supporting functions for the planner:
*/

// Structure to store each state
struct State {
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
    string actionName;
    list<string> actionArgs;

    bool operator==(const State& s) const {
        return conditions == s.conditions && conditions == s.conditions;
    }
    bool operator!=(const State& s) const {
        return conditions != s.conditions || conditions != s.conditions;
    }
};

// Hash function between two states
struct StateHasher {
    size_t operator()(const State& state) const {
        const size_t prime = 31;  // Prime number for hashing
        size_t hash = 0;

        for (const auto& condition : state.conditions) {
            GroundedConditionHasher cond_hasher;
            size_t condition_hash = cond_hasher(condition);
            hash ^= (hash << 5) + condition_hash + (hash >> 2);  // Mix the hash using bitwise operations

            // Optional: Incorporate a unique factor to each condition to make hashes more distinct
            hash = (hash * prime) ^ (condition_hash + 0x9e3779b9 + (hash >> 6));
        }

        return hash;
    }
};

// Number of conditions that need to be satisfied until goal condition is reached
int getHeuristic(const State& current, const State& goal) {
    int not_satisfied = 0;
    const auto& currentConditions = current.conditions;

    // Use the efficient std::set::count instead of find
    for (const auto& condition : goal.conditions) {
        // If the condition is not found in currentConditions
        if (!currentConditions.count(condition)) {
            ++not_satisfied;
        }
    }

    return not_satisfied;
}

// Comparison function for min-heap
struct CompareSmaller {
    constexpr bool operator()(const pair<double, State>& a, const pair<double, State>& b) const noexcept {
        return a.first > b.first;
    }
};

// Recursive function to get all permutations of symbols
vector<vector<string>> getAllPermutations(const vector<string>& symbols, int num_args) {
    vector<vector<string>> result;
    vector<string> current;
    vector<bool> used(symbols.size(), false);

    function<void()> generatePermutations = [&]() {
        if (current.size() == num_args) {
            result.push_back(current);
            return;
        }

        for (int i = 0; i < symbols.size(); ++i) {
            if (!used[i]) {
                used[i] = true;
                current.push_back(symbols[i]);
                generatePermutations();
                current.pop_back();  // backtrack
                used[i] = false;
            }
        }
    };

    generatePermutations();
    return result;
}

// Verify if the preconditions are satisfied
bool checkPreconditions(const State& current, const Action& action, const vector<string>& symbol_perm) {
    const auto& preConditions = action.get_preconditions();
    const auto& actionArgs = action.get_args();

    for (const auto& condition : preConditions) {
        list<string> newArgs;
        for (const auto& condArg : condition.get_args()) {
            auto it = find(actionArgs.begin(), actionArgs.end(), condArg);
            newArgs.push_back(it != actionArgs.end() ? symbol_perm[distance(actionArgs.begin(), it)] : condArg);
        }

        GroundedCondition gr_cond(condition.get_predicate(), newArgs, condition.get_truth());

        if (current.conditions.find(gr_cond) == current.conditions.end()) {
            return false;
        }
    }

    return true;
}

// Create the next state after applying the action
State getNextState(const State& current, const Action& action, const vector<string>& symbol_perm, const unordered_set<string>& symbols_set) {
    // Extract action arguments
    auto actionArgs = action.get_args();

    // Initialize a new state structure
    State sPrime;
    list<string> argsList(symbol_perm.begin(), symbol_perm.end());
    sPrime.actionArgs = argsList;
    sPrime.actionName = action.get_name();
    
    // Copy the current state conditions
    auto newConds = current.conditions;

    // Process each effect condition in the action's effects
    for (const auto& effectCondition : action.get_effects()) {
        auto actual_truth = effectCondition.get_truth();
        auto pred = effectCondition.get_predicate();
        list<string> newArgs;

        // Ground the arguments of the effect condition
        for (const auto& effectArg : effectCondition.get_args()) {
            // If the argument is a known symbol (not a variable), use it directly
            if (symbols_set.find(effectArg) != symbols_set.end()) {
                newArgs.push_back(effectArg);
            } else {
                // Otherwise, look for the argument in the action arguments and get its grounded value
                auto it = find(actionArgs.begin(), actionArgs.end(), effectArg);
                if (it != actionArgs.end()) {
                    int index = distance(actionArgs.begin(), it);
                    newArgs.push_back(symbol_perm[index]);
                }
            }
        }

        // Create a grounded condition with the new arguments
        GroundedCondition groundCond(pred, newArgs);

        // Check if the grounded condition is already in the state conditions
        if (newConds.find(groundCond) != newConds.end()) {
            // If the truth value has changed, remove or keep the condition accordingly
            if (actual_truth != groundCond.get_truth()) {
                newConds.erase(groundCond);
            }
        } else {
            // Otherwise, add the new grounded condition
            newConds.insert(groundCond);
        }
    }

    // Update the conditions in the new state
    sPrime.conditions = newConds;

    return sPrime;
}

// Check if all goal conditions are satisfied in the current state
bool goalConditionsSatisfied(const State& current, const State& goal) {
    // Iterate through each goal condition and check if it's present in the current state
    for (const auto& goal_cond : goal.conditions) {
        if (current.conditions.find(goal_cond) == current.conditions.end()) {
            return false;  // Goal condition not found in current state
        }
    }
    return true;  // All goal conditions are satisfied
}


pair<State, unordered_map<State, State, StateHasher>> astar(
    State goal,
    State start,
    unordered_set<Action, ActionHasher, ActionComparator> actions,
    unordered_set<string> symbols_set
)
{
    bool goalReached = false;
    State finalGoal;

    // Initialize open and closed lists, and other data structures
    priority_queue<pair<double, State>, vector<pair<double, State>>, CompareSmaller> openList; // f(s), s
    unordered_set<State, StateHasher> closedList;
    unordered_map<State, State, StateHasher> parentList; // Maps child state to parent state
    unordered_map<State, double, StateHasher> gValues;

    // Set the starting state and its initial g value
    gValues[start] = 0;
    // openList.push({gValues[start], start}); //without heuristic
    openList.push({gValues[start] + getHeuristic(start, goal), start});

    // A* algorithm loop
    while (!openList.empty()) {
        State currState = openList.top().second;
        openList.pop();

        // If the current state is the goal, stop searching
        if (goalConditionsSatisfied(currState, goal)) {
            goalReached = true;
            finalGoal = currState;
            break;
        }

        // Skip if the current state is already in the closed list
        if (closedList.count(currState)) {
            continue;
        }
        closedList.insert(currState);

        // Explore each action
        for (const auto& action : actions) {
            
            // Get all the symbol permutations to iterate through
            vector<string> symbols(symbols_set.begin(), symbols_set.end());
            vector<vector<string>> symbolPerms = getAllPermutations(symbols, action.get_args().size());

            // Check each symbol permutation
            for (const auto& currSymbolPerm : symbolPerms) {
                if (checkPreconditions(currState, action, currSymbolPerm)) {
                    // Generate the next state
                    State nextState = getNextState(currState, action, currSymbolPerm, symbols_set);

                    // If the next state is not in the closed list, process it
                    if (!closedList.count(nextState)) {
                        double tentGValue = gValues[currState] + 1; // Assuming each action has equal cost

                        // If a better path is found, update the g values and parent list
                        if (gValues.count(nextState) == 0 || tentGValue < gValues[nextState]) {
                            gValues[nextState] = tentGValue;
                            parentList[nextState] = currState;

                            // Add next state to the open list with its f value
                            // openList.push({tentGValue, nextState}); // without heuristic
                            openList.push({tentGValue + getHeuristic(nextState, goal), nextState});
                        }
                    }
                }
            }
        }
    }

    // Report if no goal was found
    if (!goalReached) {
        cout << "Goal not found." << endl;
    }

    // Return the final goal and the parent list mapping
    cout << "States expanded: " << closedList.size() << endl;
    return {finalGoal, parentList};
}

list<GroundedAction> backtrack(const State& start, const State& goal, 
                               const unordered_map<State, State, StateHasher>& parent_list)
{
    list<GroundedAction> actions;

    // Start from the goal and backtrack to the start
    State current = goal;

    // Reconstruct the path by following parent states
    while (current != start) {
        // Add the action that led to the current state
        actions.push_back(GroundedAction(current.actionName, current.actionArgs));

        // Move to the parent state
        current = parent_list.at(current);
    }

    // Reverse the actions list to get the path from start to goal
    actions.reverse();

    return actions;
}

list<GroundedAction> planner(Env* env)
{
    // Start timing the planning process
    auto start_time = chrono::high_resolution_clock::now();

    // Retrieve initial and goal conditions, actions, and symbols from the environment
    State start, goal;
    start.conditions = env->get_initial_conditions();
    goal.conditions = env->get_goal_conditions();
    unordered_set<Action, ActionHasher, ActionComparator> actions = env->get_actions();
    unordered_set<string> symbols = env->get_symbols();

    // Perform A* search to find the optimal path
    auto results = astar(goal, start, actions, symbols);

    // Backtrack to reconstruct the plan from start to goal
    // returns: [final_goal; parent_list]
    list<GroundedAction> plan = backtrack(start, results.first, results.second);

    // Measure and output planning time
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;

    return plan;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* env_file = static_cast<char *>("example.txt");
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char* filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (const GroundedAction& gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}