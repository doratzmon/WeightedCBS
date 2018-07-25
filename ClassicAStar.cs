﻿using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{   
    /// <summary>
    /// This is an implementation of the classic A* algorithm for the MAPF problem.
    /// </summary>
    public class ClassicAStar : ICbsSolver
    {
        protected ProblemInstance instance;
        protected HeuristicCalculator heuristic;
        public OpenList openList;
        public Dictionary<WorldState, WorldState> closedList;
        protected int solutionDepth;
        protected Dictionary<int, int> conflictCounts;
        protected Dictionary<int, List<int>> conflictTimes;
        protected int expanded;
        protected int generated;
        protected int reopened;
        protected int reopenedWithOldH;
        protected int noReopenHUpdates;
        protected int maxExpansionDelay;
        protected int closedListHits;
        protected int accExpanded;
        protected int accGenerated;
        protected int accReopened;
        protected int accReopenedWithOldH;
        protected int accNoReopenHUpdates;
        protected int accMaxExpansionDelay;
        protected int accClosedListHits;
        public int totalCost;
        public int numOfAgents;
        protected int maxCost;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<CbsConstraint> constraints;
        /// <summary>
        /// An array of dictionaries that map constrained timesteps to must constraints.
        /// </summary>
        protected Run runner;
        protected Plan solution;
        /// <summary>
        /// For CBS/A*
        /// </summary>
        //protected int minDepth;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public ClassicAStar(HeuristicCalculator heuristic = null)
        {
            this.closedList = new Dictionary<WorldState, WorldState>();
            this.openList = new OpenList(this);
            this.heuristic = heuristic;

            this.queryConstraint = new CbsConstraint();
            this.queryConstraint.queryInstance = true;

        }


        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost)
        {
            this.instance = problemInstance;
            this.runner = runner;
            WorldState root = this.CreateSearchRoot(minDepth, minCost);
            root.h = (int)this.heuristic.h(root); // g was already set in the constructor
            this.openList.Add(root);
            this.closedList.Add(root, root);
            this.ClearPrivateStatistics();
            this.generated++; // The root
            this.totalCost = 0;
            this.singleCosts = null;
            this.solution = null;
            this.singlePlans = null;
            this.conflictCounts = null;
            this.conflictTimes = null;
            this.solutionDepth = -1;
            this.numOfAgents = problemInstance.m_vAgents.Length;


            // Store parameters used by IndependenceDetection's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey("ID-max-cost"))
                this.maxCost = (int)(problemInstance.parameters["ID-max-cost"]);
            else
                this.maxCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey("ID - reserved") &&
                ((HashSet<TimedMove>)problemInstance.parameters["ID - reserved"]).Count != 0)
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters["ID - reserved"]);
            else
                this.illegalMoves = null;

            if (problemInstance.parameters.ContainsKey(CBS.CONSTRAINTS) &&
                ((HashSet_U<CbsConstraint>)problemInstance.parameters[CBS.CONSTRAINTS]).Count != 0)
                 this.constraints = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS.CONSTRAINTS];
        }

        /// <summary>
        /// Factory method. Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1)
        {
            return new WorldState(this.instance.m_vAgents, minDepth, minCost);
        }

        /// <summary>
        /// Clears the relevant data structures and variables to free memory.
        /// </summary>
        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.illegalMoves = null;
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.heuristic = heuristic;
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.heuristic;
        }

        public virtual String GetName()
        {
            return "A*";

        }

        public override string ToString()
        {
            string ret = this.GetName() + "/" + this.heuristic;
            if (this.openList.GetType() != typeof(OpenList))
                ret += " with " + this.openList;
            return ret;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public Dictionary<int, int> GetExternalConflictCounts()
        {
            return this.conflictCounts;
        }


        public Dictionary<int, List<int>> GetConflictTimes()
        {
            return this.conflictTimes;
        }


        protected void ClearPrivateStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
            this.reopened = 0;
            this.reopenedWithOldH = 0;
            this.noReopenHUpdates = 0;
            this.closedListHits = 0;
            this.maxExpansionDelay = -1;
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened With Old H");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " H Updated From Other Area");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max expansion delay");
            output.Write(Run.RESULTS_DELIMITER);


            this.heuristic.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes: {0}", this.GetExpanded());
            Console.WriteLine("Total Generated Nodes: {0}", this.GetGenerated());
            Console.WriteLine("Total Reopened Nodes: {0}", this.reopened);
            Console.WriteLine("Closed list hits: {0}", this.closedListHits);
            Console.WriteLine("Reopened Nodes With Old H: {0}", this.reopenedWithOldH);
            Console.WriteLine("No Reopen H Updates: {0}", this.noReopenHUpdates);
            Console.WriteLine("Max expansion delay: {0}", this.maxExpansionDelay);


            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write(this.reopened + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.reopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.noReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.maxExpansionDelay + Run.RESULTS_DELIMITER);

            this.heuristic.OutputStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            this.ClearPrivateStatistics();
            this.heuristic.ClearStatistics();
            this.openList.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
            this.accReopened = 0;
            this.accClosedListHits = 0;
            this.accReopenedWithOldH = 0;
            this.accNoReopenHUpdates = 0;
            this.accMaxExpansionDelay = 0;

            this.heuristic.ClearAccumulatedStatistics();
            this.openList.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accReopened += this.reopened;
            this.accClosedListHits += this.closedListHits;
            this.accReopenedWithOldH += this.reopenedWithOldH;
            this.accNoReopenHUpdates += this.noReopenHUpdates;
            this.accMaxExpansionDelay = Math.Max(this.accMaxExpansionDelay, this.maxExpansionDelay);

            this.heuristic.AccumulateStatistics();

            this.openList.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGenerated);
            Console.WriteLine("{0} Accumulated Reopened Nodes (Low-Level): {1}", this, this.accReopened);
            Console.WriteLine("{0} Accumulated Closed list hits (Low-Level): {1}", this, this.accClosedListHits);
            Console.WriteLine("{0} Accumulated Reopened Nodes With Old H (Low-Level): {1}", this, this.accReopenedWithOldH);
            Console.WriteLine("{0} Accumulated No Reopen H Updates (Low-Level): {1}", this, this.accNoReopenHUpdates);
            Console.WriteLine("{0} Accumulated Max expansion delay (Low-Level): {1}", this, this.accMaxExpansionDelay);


            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accReopened + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.accReopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.accNoReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.accMaxExpansionDelay + Run.RESULTS_DELIMITER);



            this.openList.OutputAccumulatedStatistics(output);
        }

        public bool debug = false;

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public virtual bool Solve()
        {
            int initialEstimate = ((WorldState)openList.Peek()).h; // g=0 initially

            int lastF = -1;
            WorldState lastNode = null;
  
            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.solutionDepth = ((WorldState)openList.Peek()).f - initialEstimate; // A minimum estimate
                    this.Clear();
                    return false;
                }

                var currentNode = (WorldState)openList.Remove();

                if (debug)
                {
                    Debug.WriteLine("");
                    Debug.WriteLine("Expanding node: " + currentNode);
                }


               
                lastF = currentNode.f;
                lastNode = currentNode;

                // Calculate expansion delay
                int expansionDelay = this.expanded - currentNode.expandedCountWhenGenerated - 1; // -1 to make the delay zero when a node is expanded immediately after being generated.
                maxExpansionDelay = Math.Max(maxExpansionDelay, expansionDelay);

                // Check if node is the goal, or knows how to get to it
                if (currentNode.GoalTest())
                {
                    //((IReadOnlyDictionary<TimedMove, List<int>>)instance.parameters[CBS_LocalConflicts.CAT]), conflictRange);
                    /*List<TimedMove> newList = new List<TimedMove>();
                    WorldState current = currentNode;
                    while(current != null)
                    {
                        newList.Add(current.)
                    }*/
                    this.conflictTimes = currentNode.conflictTimes;

                    this.totalCost = currentNode.GetGoalCost();
                    this.singleCosts = currentNode.GetSingleCosts();
                    this.solution = currentNode.GetPlan();
                    this.singlePlans = currentNode.GetSinglePlans();
                    this.conflictCounts = currentNode.cbsInternalConflicts; // TODO: Technically, could be IndependenceDetection's count. Merge them.
                    this.conflictTimes = currentNode.conflictTimes;
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.Clear();
                    return true;
                }

                
                Expand(currentNode);
                expanded++;

                
            }

            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implemented together with the closed list.
        /// </summary>
        /// <param name="node"></param>
        public virtual void Expand(WorldState node)
        {
            var intermediateNodes = new List<WorldState>();
            intermediateNodes.Add(node);

            for (int agentIndex = 0; agentIndex < this.instance.m_vAgents.Length ; ++agentIndex)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;

                //Debug.Print("Moving {0} agent", agentIndex);

                intermediateNodes = ExpandOneAgent(intermediateNodes, agentIndex);
            }
            var finalGeneratedNodes = intermediateNodes;

            foreach (var currentNode in finalGeneratedNodes)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;
                currentNode.CalculateG();
                currentNode.makespan++;
                currentNode.h = (int)this.heuristic.h(currentNode);

                if (currentNode.g < currentNode.minCost)
                {
                    if (currentNode.h == 0)
                        currentNode.h = 2; // Otherwise waiting at goal would expand to waiting at the goal for the same too low cost,
                                           // which would expand to waiting at the goal, etc.
                                           // +2 because you need a step out of the goal and another step into it.
                    currentNode.h = Math.Max(currentNode.h, currentNode.minCost - currentNode.g);
                }
            }

            

            // Enter the generated nodes into the open list
            foreach (var child in finalGeneratedNodes)
            {
               /* if (Program.TO_EXECUTE && Run.replanStopwath.ElapsedMilliseconds > Run.TIMEOUT)
                    throw new Exception("4");*/
                ProcessGeneratedNode(child);
            }

            if (this.debug)
                Debug.Print("\n");
        }
        
        /// <summary>
        /// Expands a single agent in the nodes.
        /// This includes:
        /// - Generating the children
        /// - Inserting them into OPEN
        /// - Insert node into CLOSED
        /// Returns the child nodes
        /// </summary>
        protected virtual List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            /*if (Program.TO_EXECUTE && Run.replanStopwath != null && Run.replanStopwath.ElapsedMilliseconds > Run.TIMEOUT)
                throw new Exception("3");*/
            var GeneratedNodes = new List<WorldState>();
            WorldState childNode;

            foreach (var currentNode in intermediateNodes)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    break;

                // Try all legal moves of the agents
                foreach (TimedMove agentLocation in currentNode.allAgentsState[agentIndex].lastMove.GetNextMoves())
                {
                    WorldState origNode = agentIndex == 0? currentNode : currentNode.prevStep;
                    bool moveIsValid = true;
                    //moveIsValid = this.IsValid(agentLocation, currentNode.currentMoves, currentNode.makespan + 1, agentIndex, origNode, currentNode);
                    //if (moveIsValid == false)
                    //    continue;

                    //----------------Begin pasting isValid method
                    TimedMove possibleMove = agentLocation;
                    IReadOnlyDictionary<TimedMove, int> currentMoves = currentNode.currentMoves;
                    int makespan = currentNode.makespan + 1;
                    WorldState fromNode = origNode;
                    WorldState intermediateMode = currentNode;
                    int agentNum = fromNode.allAgentsState[agentIndex].agent.agentNum;

                    // Check if the proposed move is reserved in the plan of another agent.
                    // This is used in IndependenceDetection's ImprovedID.
                    if (this.illegalMoves != null)
                    {
                        if (possibleMove.IsColliding(illegalMoves))
                            continue;
                    } // FIXME: Also checked in instance.IsValid later.

                    if (this.constraints != null)
                    {
                        queryConstraint.Init(agentNum, possibleMove);

                        if (this.constraints.Contains(queryConstraint))
                            continue;
                    }

                   

                    // If the tile is not free (out of the grid or with an obstacle)
                    if (this.instance.IsValid(possibleMove) == false)
                        continue;

                    // Check against all the agents that have already moved to see if current move collides with their move
                    bool collision;

                    collision = possibleMove.IsColliding(currentMoves);

                    if (collision)
                        continue;
                    //----------------end paste from isValid

                    childNode = CreateSearchNode(currentNode);
                    childNode.allAgentsState[agentIndex].MoveTo(agentLocation);

                    childNode.prevStep = currentNode.prevStep; // Skip temporary node objects used during expansion process.
                    if (agentIndex == 0)
                        childNode.prevStep = currentNode;
                    if (agentIndex < currentNode.allAgentsState.Length - 1) // More agents need to move
                        childNode.currentMoves.Add(agentLocation, agentIndex);
                    else // Moved the last agent
                        childNode.currentMoves.Clear(); // To reduce memory load and lookup times, even though it's correct to leave the old moves since they're timed.

                    GeneratedNodes.Add(childNode);
                }
            }
            return GeneratedNodes;
        }

        /// <summary>
        /// Factory method.
        /// </summary>
        /// <param name="from"></param>
        /// <returns></returns>
        protected virtual WorldState CreateSearchNode(WorldState from)
        {
            return new WorldState(from);
        }

        /// <summary>
        /// Just an optimization
        /// </summary>
        private CbsConstraint queryConstraint;

        /// <summary>
        /// Check if the move is valid, i.e. not colliding into walls or other agents.
        /// This method is here instead of in ProblemInstance to enable algorithmic tweaks.
        /// </summary>
        /// <param name="possibleMove">The move to check if possible</param>
        /// <returns>true, if the move is possible.</returns>
        protected virtual bool IsValid(TimedMove possibleMove, IReadOnlyDictionary<TimedMove, int> currentMoves, int makespan, int agentIndex, WorldState fromNode, WorldState intermediateMode)
        {
            int agentNum = fromNode.allAgentsState[agentIndex].agent.agentNum;

            // Check if the proposed move is reserved in the plan of another agent.
            // This is used in IndependenceDetection's ImprovedID.
            if (this.illegalMoves != null)
            {
                if (possibleMove.IsColliding(illegalMoves))
                    return false;
            } // FIXME: Also checked in instance.IsValid later.

            if (this.constraints != null)
            {
                this.queryConstraint.Init(agentNum, possibleMove);

                if (this.constraints.Contains(queryConstraint))
                    return false;
            }


            // If the tile is not free (out of the grid or with an obstacle)
            if (this.instance.IsValid(possibleMove) == false)
                return false;

            // Check against all the agents that have already moved to see if current move collides with their move
            bool collision;

            
            collision = possibleMove.IsColliding(currentMoves);
                
            return collision == false;
        }

        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        protected SinglePlan[] singlePlans;

        public virtual SinglePlan[] GetSinglePlans()
        {
            return this.singlePlans;
        }

        protected int[] singleCosts;

        public virtual int[] GetSingleCosts()
        {
            return this.singleCosts;
        }

        public int getExpanded() { return this.expanded; }
        public int getGenerated() { return this.generated; }
        public int GetSolutionDepth() { return this.solutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }

        public void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.Setup(problemInstance, -1, runner, -1);
        }

        /// <summary>
        /// Returns whether the node was inserted into the open list.
        /// </summary>
        /// <param name="currentNode"></param>
        /// <returns></returns>
        protected virtual bool ProcessGeneratedNode(WorldState currentNode)
        {
            if (currentNode.h + currentNode.g <= this.maxCost)
            // Assuming h is an admissable heuristic, no need to generate nodes that won't get us to the goal
            // within the budget
            {
                if (instance.parameters.ContainsKey("ID-ConflictAvoidance"))
                {
                    // Accumulating the conflicts count from parent to child
                    // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
                    currentNode.cbsInternalConflicts = new Dictionary<int, int>(currentNode.prevStep.cbsInternalConflicts);
                    currentNode.conflictTimes = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimes)
                        currentNode.conflictTimes[kvp.Key] = new List<int>(kvp.Value);
                    currentNode.conflictTimesBias = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimesBias)
                        currentNode.conflictTimesBias[kvp.Key] = new List<int>(kvp.Value);
                    currentNode.conflictProbability = new Dictionary<int, List<double>>();
                    foreach (var kvp in currentNode.prevStep.conflictProbability)
                        currentNode.conflictProbability[kvp.Key] = new List<double>(kvp.Value);

                    currentNode.UpdateConflictCounts(
                        ((IReadOnlyDictionary<TimedMove, List<int>>)instance.parameters["ID-ConflictAvoidance"]));
                    // We're counting conflicts along the entire path, so the parent's conflicts count
                    // is added to the child's.

                    currentNode.potentialConflictsCount = currentNode.cbsInternalConflicts.Count;

                    // FIXME: The above code duplication with the CBS CAT. Some of the vars above are actually from CBS now.
                }

                if (instance.parameters.ContainsKey(CBS.CAT))
                {
                    // Accumulating the conflicts count from parent to child.
                    // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
                    currentNode.cbsInternalConflicts = new Dictionary<int,int>(currentNode.prevStep.cbsInternalConflicts);
                    currentNode.conflictTimes = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimes)
                        currentNode.conflictTimes[kvp.Key] = new List<int>(kvp.Value);
                    currentNode.conflictTimesBias = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimesBias)
                        currentNode.conflictTimesBias[kvp.Key] = new List<int>(kvp.Value);
                    currentNode.conflictProbability = new Dictionary<int, List<double>>();
                    foreach (var kvp in currentNode.prevStep.conflictProbability)
                        currentNode.conflictProbability[kvp.Key] = new List<double>(kvp.Value);

                    currentNode.UpdateConflictCounts(
                        ((IReadOnlyDictionary<TimedMove, List<int>>)instance.parameters[CBS.CAT]));

                    // Count one for every agent the path conflicts with any number of times:
                    currentNode.cbsInternalConflictsCount = currentNode.cbsInternalConflicts.Count;
                }

                

                // If in closed list - only reopen if F is lower or node is otherwise preferred
                if (this.closedList.ContainsKey(currentNode) == true)
                {
                    ++this.closedListHits;
                    WorldState inClosedList = this.closedList[currentNode];
                    // Notice the agents may have gotten to their location from a different direction in this node.

                    

                    // Since the nodes are equal, give them both the max of their H
                    bool improvedHOfThisNode = false;
                    bool improvedHOfOldNode = false;
                    if (currentNode.h < inClosedList.h)
                    {
                        currentNode.hBonus += inClosedList.h - currentNode.h;
                        currentNode.h = inClosedList.h;
                        improvedHOfThisNode = true;
                    }
                    if (inClosedList.h < currentNode.h)
                    {
                        inClosedList.hBonus += currentNode.h - inClosedList.h;
                        inClosedList.h = currentNode.h;
                        improvedHOfOldNode = true;
                    }

                    byte hackData = 0;


                    int compareVal = currentNode.CompareTo(inClosedList);


                    if (compareVal == -1) // Enables re-trying a node with different paths for the agents
                    {
                        this.reopened++;
                        this.closedList.Remove(inClosedList);
                        this.openList.Remove(inClosedList);
                        // Items are searched for in the heap using their binaryHeapIndex, which is only initialized when they're put into it,
                        // and not their hash or their Equals or CompareTo methods, so it's important to call Remove with inClosedList,
                        // which might be in the heap, and not currentNode, which may be Equal to it, but was never in the heap so it
                        // doesn't have a binaryHeapIndex initialized.
                        if (improvedHOfThisNode)
                            ++reopenedWithOldH;
                    }
                    else if (improvedHOfOldNode)
                    {
                        // Reinsert old node with new higher F, if it's still in the open list.
                        // This pushes it further back in the open list so it certainly won't be smaller than the currently expanded node, so monotonicity is maintained.
                        if (this.openList.Remove(inClosedList)) // Cheap if it isn't there
                        {
                            inClosedList.Clear();
                            this.openList.Add(inClosedList);
                            ++noReopenHUpdates;
                        }
                    }
                }

                if (this.closedList.ContainsKey(currentNode) == false)
                { 

                    this.closedList.Add(currentNode, currentNode);
                    this.generated++; // Reopened nodes are also recounted here.
                    this.openList.Add(currentNode);
                    currentNode.expandedCountWhenGenerated = this.expanded;
                    if (this.debug)
                        Debug.Print("Generated node {0}", currentNode);
                    return true;
                }
                else
                {
                    if (this.debug)
                        Debug.Print("NOT generating node {0}. It already exists.", currentNode);
                }

                // What if in open list? This implementation immediately puts _generated_ nodes in the closed list,
                // so it only needs to check it and not the open list.
                // That actually makes a lot of sense: membership tests in heaps are expensive, and in hashtables are cheap.
                // This way we only need to _search_ the open list if we encounter a node that was already visited.
            }
            return false;
        }

        ///// <summary>
        ///// 
        ///// </summary>
        ///// <param name="agentIndex"></param>
        ///// <param name="fromNode"></param>
        ///// <returns>Whether the shuffle succeeded</returns>
        //bool RMStarShuffleIndividualPath(CbsConflict conflict, bool agentA, WorldState fromNode)
        //{
        //    int agentIndex = agentA ? conflict.agentAIndex : conflict.agentBIndex;
        //    //WorldState node = fromNode.individualMStarPlanBases[agentIndex];
        //    WorldState node = fromNode;

        //    if (this.mstarPlanBasesToTheirConstraints.ContainsKey(node) == false)
        //        this.mstarPlanBasesToTheirConstraints[node] = new HashSet<CbsConstraint>[this.instance.GetNumOfAgents()];
        //    if (this.mstarPlanBasesToTheirConstraints[node][agentIndex] == null)
        //        this.mstarPlanBasesToTheirConstraints[node][agentIndex] = new HashSet<CbsConstraint>();

        //    return solveOneAgentForMstar(node, conflict, agentA);
        //}

        //protected bool solveOneAgentForMstar(WorldState node, CbsConflict conflict, bool agentA)
        //{
        //    int agentIndex = agentA ? conflict.agentAIndex : conflict.agentBIndex;
        //    HashSet_U<CbsConstraint> constraints = null;
        //    HashSet<CbsConstraint> newConstraints = null;
        //    int oldMaxCost = int.MaxValue;
        //    if (this.instance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
        //        constraints = (HashSet_U<CbsConstraint>)this.instance.parameters[CBS_LocalConflicts.CONSTRAINTS];
        //    else
        //    {
        //        constraints = new HashSet_U<CbsConstraint>();
        //        this.instance.parameters[CBS_LocalConflicts.CONSTRAINTS] = constraints;
        //    }

        //    if (this.instance.parameters.ContainsKey(CBS_LocalConflicts.CAT) == false)
        //        this.instance.parameters[CBS_LocalConflicts.CAT] = new Dictionary_U<TimedMove, int>(); // Indicate TO CBS that another level is running above it

        //    if (this.debug)
        //    {
        //        Debug.Print("Planning for agent index: " + agentIndex + " in node: " + node);
        //    }
            
        //    newConstraints = this.mstarPlanBasesToTheirConstraints[node][agentIndex];
        //    CbsConstraint newConstraint = new CbsConstraint(conflict, this.instance, agentA);
        //    newConstraints.Add(newConstraint);
        //    constraints.Join(newConstraints);

        //    if (this.debug)
        //    {
        //        Debug.Print("Constraints: ");
        //        foreach (var constraint in newConstraints)
        //        {
        //            Debug.Print(constraint.ToString());
        //        }
        //    }
            
        //    oldMaxCost = this.maxCost;
        //    //this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = this.mstarPlanBasesToTheirPlans[node][agentIndex].GetSize() - 1;
        //    this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = node.individualMStarPlans[agentIndex].GetSize() - 1;

        //    bool success = solveOneAgentForMstar(node, agentIndex);

        //    constraints.Separate(newConstraints);
        //    this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = oldMaxCost;
        //    this.instance.parameters.Remove(CBS_LocalConflicts.CAT);

        //    return success;
        //}

        //protected bool solveOneAgentForMstar(WorldState node, int agentIndex)
        //{
        //    AgentState[] thisAgentOnly = new AgentState[1];
        //    thisAgentOnly[0] = node.allAgentsState[agentIndex];
        //    var subProblem = this.instance.Subproblem(thisAgentOnly);

        //    ClassicAStar astar = new ClassicAStar(this.heuristic);
        //    ICbsSolver solver = new CBS_LocalConflicts(astar, astar); // Uses a precomputed solution if possible
        //    solver.Setup(subProblem, this.runner);
        //    bool success = solver.Solve();

        //    if (success)
        //    {
        //        //this.mstarPlanBasesToTheirPlans[node][agentIndex] = solver.GetSinglePlans()[0];
        //        node.individualMStarPlans[agentIndex] = solver.GetSinglePlans()[0];
        //    }
        //    // else nothing. Don't null the old plan yet - it might be saved by a successful replan of the other agent

        //    return success;
        //}

        /// <summary>
        /// NOT the algorithm in the M* journal paper.
        /// They want each node to propagate its entire collision set, not just the new conflict that began the process.
        /// This implementation may be suitable for the M* algorithm as appears in the paper,
        /// but it isn't suitable when we want to backpropagate from a closed list hit,
        /// because then we don't have a specific conflict to propagate.
        /// </summary>
        /// <param name="conflict"></param>
        /// <param name="fromNode">
        /// Not the node where the collision happened, because it was never generated.
        /// The node from where the colliding moves were made.
        /// </param>
        void RMStarCollisionBackPropagation(CbsConflict conflict, WorldState fromNode)
        {
            if (this.debug)
                Debug.Print("Back prop!!");
            var queue = new Queue<WorldState>();
            queue.Enqueue(fromNode);

            while (queue.Count != 0)
            {
                var node = queue.Dequeue();

                bool onlyUnitedNow = node.collisionSets.Union(conflict.agentAIndex, conflict.agentBIndex);

                if (onlyUnitedNow)
                {
                    if (this.debug)
                        Console.WriteLine("Re-opening node {0} with an updated collision set", node);
                    this.reinsertIntoOpenList(node);

                    foreach (var next in node.backPropagationSet)
                        queue.Enqueue(next);
                }
            }
        }

        /// <summary>
        /// Not the paper's version either, since it always propagates the same collision sets.
        /// </summary>
        /// <param name="colSets">The collision sets of the _child_ of fromNode</param>
        /// <param name="fromNode"></param>
        void RMStarCollisionBackPropagation(DisjointSets<int> colSets, WorldState fromNode)
        {
            if (this.debug)
                Debug.Print("Back prop!!");
            var queue = new Queue<WorldState>();
            queue.Enqueue(fromNode);

            while (queue.Count != 0)
            {
                var node = queue.Dequeue();

                bool onlyUnitedNow = node.collisionSets.CopyUnions(colSets);

                if (onlyUnitedNow)
                {
                    if (this.debug)
                        Console.WriteLine("Re-opening node {0} with an updated collision set", node);
                    this.reinsertIntoOpenList(node);

                    foreach (var next in node.backPropagationSet)
                        queue.Enqueue(next);
                }
            }
        }

        void reinsertIntoOpenList(WorldState node)
        {
            //if (this.openList.Contains(node)) // Node is partially expanded in the open list. Need to restart its expansion
            //{
            //    this.openList.Remove(node);
            //    node.Clear();
            //}
            this.openList.Remove(node);
            node.Clear();
            this.openList.Add(node); // Re-insert into open list
        }

        public int GetExpanded() { return expanded; }
        public int GetGenerated() { return generated; }
        public int GetAccumulatedExpanded() { return accExpanded; }
        public int GetAccumulatedGenerated() { return accGenerated; }
        public int GetMaxGroupSize() { return numOfAgents; }

        public virtual float GetEffectiveBranchingFactor()
        {
            return ((float)this.GetGenerated() - 1) / this.GetExpanded();
        }
    }
}

