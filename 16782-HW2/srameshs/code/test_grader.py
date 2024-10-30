import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer
import random

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################
def generate_random_values_set(n):
    return ",".join([f"{random.uniform(0, 2 * 3.141592):.6f}" for _ in range(n)])

def graderMain(executablePath, gradingCSV):
    # problems = [["map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
    #                             # "0.392699,2.356194,3.141592,2.8274328,4.712388"],
    #                             "0.392699,2.356194,3.141592,2.8274328,4.712388"],
    #         ["map2.txt", "0.392699,2.356194,3.141592",
    #                             "1.570796,0.785398,1.570796"]]
    
    # '''
    # test problems
    problems = [
                ["map2.txt", "1.570796,0.785398,1.570796",
                                "1.426800,3.004259,0.744009"], #1
                ["map2.txt", "0.208604,0.942672,1.643999",
                                "1.426800,3.004259,0.744010"], #2
                ["map2.txt", "0.208604,0.942672,1.644000",
                                "0.591497,0.829352,4.911901"], #3
                ["map2.txt", "0.078024,2.517531,2.605214",
                                "0.591497,0.829352,4.911902"], #4
                ["map2.txt", "0.078024,2.517531,2.605215",
                                "1.386892,2.908482,2.192150"], #5
                ["map2.txt", "1.648069,1.783545,4.581647",
                                "1.386892,2.908482,2.192151"], #6
                ["map2.txt", "1.648069,1.783545,4.581648",
                                "0.779721,2.694798,4.610462"], #7
                ["map2.txt", "0.336436,0.156208,3.397241",
                                "0.779721,2.694798,4.610462"], #8
                ["map2.txt", "0.336436,0.156208,3.397242",
                                "0.090319,0.810931,3.406299"], #9
                ["map2.txt", "1.221424,1.739161,5.628866",
                                "0.090319,0.810931,3.406299"], #10
                ["map2.txt", "0.953469,1.892977,0.299746",
                                "1.275997,1.792085,3.872807"], #11
                ["map2.txt", "1.242128,1.881624,0.449559",
                                "1.275997,1.792085,3.872807"], #12
                ["map2.txt", "0.761430,0.516888,5.838021",
                                "1.840280,1.403200,4.466922"], #13
                ["map2.txt", "1.601606,5.419748,1.965229",
                                "1.840280,1.403200,4.466922"], #14
                ["map2.txt", "1.601606,5.419748,1.965229",
                                "1.238338,1.384861,6.093400"], #15
                ["map2.txt", "0.862221,0.241067,2.143457",
                                "1.238338,1.384861,6.093400"], #16
                ["map2.txt", "0.862221,0.241067,2.143457",
                                "0.532289,0.344078,1.072242"], #17
                ["map2.txt", "0.932833,2.342022,3.859671",
                                "0.532289,0.344078,1.072242"], #18
                ["map2.txt", "0.932833,2.342022,3.859671",
                                "1.580465,4.517694,1.128095"], #19
                ["map2.txt", "0.953469,1.892977,0.299746",
                                "1.580465,4.517694,1.128095"]] #20
    # '''

    # extra credit configuration:
    # problems = [["map2.txt", "0.953469,1.892977,0.299746",
    #                             "1.275997,1.792085,3.872807"]]

    # problems = [["map2.txt", "0.953469,1.892977,0.299746", generate_random_values_set(3)]]
    # problems = [["map2.txt", generate_random_values_set(3), "1.275997,1.792085,3.872807"]]
    # print("RANDOM CONFIG",problems)
    
    scores = []
    for aPlanner in [0, 1, 2, 3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "../output/grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./../build/verifier {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                # ### Visualize their results
                # commandViz = "python3 visualizer.py ../output/grader_out/tmp.txt --gifFilepath=../output/grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                # commandViz += " --incPrev=1"
                # subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./../build/planner", "../output/grader_out/grader_results.csv")
    # graderMain("./../build/planner", "../output/grader_out/extraCredit.csv")