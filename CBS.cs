using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using System.Linq;


namespace CPF_experiment
{
    public class CBS : ICbsSolver
    {
        /// <summary>
        /// The key of the constraints list used for each CBS node
        /// </summary>
        public static readonly string CONSTRAINTS = "constraints";
        /// <summary>
        /// The key of the must constraints list used for each CBS node
        /// </summary>        /// <summary>
        /// The key of the internal CAT for CBS, used to favor A* nodes that have fewer conflicts with other routes during tie-breaking.
        /// Also used to indicate that CBS is running.
        /// </summary>
        public static readonly string CAT = "CBS CAT";

        protected ProblemInstance instance;
        public OpenList openList;
        /// <summary>
        /// Might as well be a HashSet. We don't need to retrive from it.
        /// </summary>
        public Dictionary<CbsNode, CbsNode> closedList;
        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int closedListHits;
        protected int pruningSuccesses;
        protected int pruningFailures;
        protected int nodesExpandedWithGoalCost;
        protected int nodesPushedBack;
        protected int accHLExpanded;
        protected int accHLGenerated;
        protected int accClosedListHits;
        protected int accPartialExpansions;
        protected int accBypasses;
        protected int accPruningSuccesses;
        protected int accPruningFailures;
        protected int accNodesExpandedWithGoalCost;
        protected int accNodesPushedBack;

        public int totalCost;
        protected int solutionDepth;
        public Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        /// <summary>
        /// Nodes with with a higher cost aren't generated
        /// </summary>
        protected int maxCost;
        /// <summary>
        /// Search is stopped when the minimum cost passes the target
        /// </summary>
        public int targetCost {set; get;}
        /// <summary>
        /// Search is stopped when the low level generated nodes count exceeds the cap
        /// </summary>
        public int lowLevelGeneratedCap { set; get; }
        /// <summary>
        /// Search is stopped when the millisecond count exceeds the cap
        /// </summary>
        public int milliCap { set; get; }
        protected ICbsSolver solver;
        protected ICbsSolver singleAgentSolver;

        /// <summary>
        /// TODO: Shouldn't this be called minTimeStep?
        /// </summary>
        protected int minDepth;
        protected int maxSizeGroup;
        protected int accMaxSizeGroup;
        /// <summary>
        /// Used to know when to clear problem parameters.
        /// </summary>
        public bool topMost;


        public CBS(ICbsSolver singleAgentSolver, ICbsSolver generalSolver)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new OpenList(this);
            this.solver = generalSolver;
            this.singleAgentSolver = singleAgentSolver;
        }
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="minDepth"></param>
        /// <param name="runner"></param>
        /// <param name="minCost">Not taken into account</param>
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost = -1)
        {
            this.instance = problemInstance;
            this.runner = runner;
            this.ClearPrivateStatistics();
            this.totalCost = 0;
            this.solutionDepth = -1;
            this.targetCost = int.MaxValue;
            this.lowLevelGeneratedCap = int.MaxValue;
            this.milliCap = int.MaxValue;
            this.goalNode = null;
            this.solution = null;

            this.maxCost = int.MaxValue;
            this.topMost = this.SetGlobals();

            this.minDepth = minDepth;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, this.solver, this.singleAgentSolver, this); // Problem instance and various strategy data is all passed under 'this'.
            // Solve the root node
            bool solved = root.Solve(minDepth);
            
            if (solved && root.totalCost <= this.maxCost)
            {
                this.openList.Add(root);
                this.highLevelGenerated++;
                this.closedList.Add(root, root);
            }
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.Setup(problemInstance, 0, runner);
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.solver.SetHeuristic(heuristic);
        }


        public HeuristicCalculator GetHeuristic()
        {
            return this.solver.GetHeuristic();
        }



        public Dictionary<int, int> GetExternalConflictCounts()
        {
            throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
        }

        public Dictionary<int, List<int>> GetConflictTimes()
        {
            throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
        }



        public ProblemInstance GetProblemInstance()
        {
            return this.instance;
        }

        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.solver.Clear();
            // Statistics are reset on Setup.
        }

        public virtual string GetName() 
        {
            string lowLevelSolvers;
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver))
                lowLevelSolvers = "(" + this.singleAgentSolver + ")";
            else
                lowLevelSolvers = "(single:" + singleAgentSolver + " multi:" + solver + ")";
            string variants = "";
            

            return "Basic-CBS/" + lowLevelSolvers + variants;
        }

        public override string ToString()
        {
            return GetName();
        }

        public int GetSolutionCost() { return this.totalCost; }

        protected void ClearPrivateStatistics()
        {
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.closedListHits = 0;
            this.pruningSuccesses = 0;
            this.pruningFailures = 0;
            this.nodesExpandedWithGoalCost = 0;
            this.nodesPushedBack = 0;
            this.maxSizeGroup = 1;
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Pruning Successes (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Pruning Failures (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Nodes Expanded With Goal Cost (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Nodes Pushed Back (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group Size (HL)");
            output.Write(Run.RESULTS_DELIMITER);

            this.solver.OutputStatisticsHeader(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Closed List Hits (High-Level): {0}", this.closedListHits);
            Console.WriteLine("Pruning successes (High-Level): {0}", this.pruningSuccesses);
            Console.WriteLine("Pruning failures (High-Level): {0}", this.pruningFailures);
            Console.WriteLine("Nodes expanded with goal cost (High-Level): {0}", this.nodesExpandedWithGoalCost);
            Console.WriteLine("Nodes Pushed Back (High-Level): {0}", this.nodesPushedBack);
            Console.WriteLine("Max Group Size (High-Level): {0}", this.maxSizeGroup);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.pruningSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.pruningFailures + Run.RESULTS_DELIMITER);
            output.Write(this.nodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
            output.Write(this.nodesPushedBack + Run.RESULTS_DELIMITER);
            output.Write(this.maxSizeGroup + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputAccumulatedStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                int numSolverStats = this.solver.NumStatsColumns;
                if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                    numSolverStats += this.singleAgentSolver.NumStatsColumns;
                return 17 + numSolverStats + this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            if (this.topMost)
            {
                this.solver.ClearAccumulatedStatistics(); // Is this correct? Or is it better not to do it?
                if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                    this.singleAgentSolver.ClearAccumulatedStatistics();
            }
            this.ClearPrivateStatistics();
            this.openList.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accHLExpanded = 0;
            this.accHLGenerated = 0;
            this.accClosedListHits = 0;
            this.accPruningSuccesses = 0;
            this.accPruningFailures = 0;
            this.accNodesExpandedWithGoalCost = 0;
            this.accNodesPushedBack = 0;
            this.accMaxSizeGroup = 1;

            this.solver.ClearAccumulatedStatistics();
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.ClearAccumulatedStatistics();

            this.openList.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accHLExpanded += this.highLevelExpanded;
            this.accHLGenerated += this.highLevelGenerated;
            this.accClosedListHits += this.closedListHits;
            this.accPruningSuccesses += this.pruningSuccesses;
            this.accPruningFailures += this.pruningFailures;
            this.accNodesExpandedWithGoalCost += this.nodesExpandedWithGoalCost;
            this.accNodesPushedBack += this.nodesPushedBack;
            this.accMaxSizeGroup = Math.Max(this.accMaxSizeGroup, this.maxSizeGroup);

            // this.solver statistics are accumulated every time it's used.

            this.openList.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, this.accHLExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, this.accHLGenerated);
            Console.WriteLine("{0} Accumulated Closed List Hits (High-Level): {1}", this, this.accClosedListHits);
            Console.WriteLine("{0} Accumulated Pruning Successes (High-Level): {1}", this, this.accPruningSuccesses);
            Console.WriteLine("{0} Accumulated Pruning Failures (High-Level): {1}", this, this.accPruningFailures);
            Console.WriteLine("{0} Accumulated Nodes Expanded With Goal Cost (High-Level): {1}", this, this.accNodesExpandedWithGoalCost);
            Console.WriteLine("{0} Accumulated Nodes Pushed Back (High-Level): {1}", this.accNodesPushedBack);
            Console.WriteLine("{0} Max Group Size (High-Level): {1}", this, this.accMaxSizeGroup);

            output.Write(this.accHLExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accHLGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.accPruningSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.accPruningFailures + Run.RESULTS_DELIMITER);
            output.Write(this.accNodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
            output.Write(this.accNodesPushedBack + Run.RESULTS_DELIMITER);
            output.Write(this.accMaxSizeGroup + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        public bool debug = false;
        private bool equivalenceWasOn;

        /// <summary>
        /// 
        /// </summary>
        /// <returns>Whether this is the top-most CBS</returns>
        protected bool SetGlobals()
        {
            this.equivalenceWasOn = (AgentState.EquivalenceOverDifferentTimes == true);
            AgentState.EquivalenceOverDifferentTimes = false;

            if (this.instance.parameters.ContainsKey(CBS.CAT) == false) // Top-most CBS solver
            {
                this.instance.parameters[CBS.CAT] = new Dictionary_U<TimedMove, int>(); // Dictionary_U values are actually lists of ints.
                this.instance.parameters[CBS.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                return true;
            }
            else
                return false;
        }

        protected void CleanGlobals()
        {
            if (this.equivalenceWasOn)
                AgentState.EquivalenceOverDifferentTimes = true;
            if (this.topMost) // Clear problem parameters. Done for clarity only, since the Union structures are expected to be empty at this point.
            {
                this.instance.parameters.Remove(CBS.CAT);
                this.instance.parameters.Remove(CBS.CONSTRAINTS);
                // Don't remove must constraints:
                // A) It usually wasn't CBS that added them (they're only used for Malte's variant).
                // B) Must constraints only appear in temporary problems. There's no danger of leaking them to other solvers.
                // C) We don't have the information to re-create them later.
            }
        }

        public bool Solve()
        {
            //this.SetGlobals(); // Again, because we might be resuming a search that was stopped.

            int initialEstimate = 0;
            if (openList.Count > 0)
                initialEstimate = ((CbsNode)openList.Peek()).totalCost;

            int maxExpandedNodeCostPlusH = -1;
            int currentCost = -1;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    this.totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.solutionDepth = ((CbsNode)openList.Peek()).totalCost - initialEstimate; // A minimum estimate
                    this.Clear(); // Total search time exceeded - we're not going to resume this search.
                    this.CleanGlobals();
                    return false;
                }

                var currentNode = (CbsNode)openList.Remove();
                currentNode.ChooseConflict();

                // A cardinal conflict may have been found, increasing the h of the node.
                // Check if the node needs to be pushed back into the open list.
                if (this.openList.Count != 0 &&
                    currentNode.f > ((CbsNode)this.openList.Peek()).f)
                {
                    if (this.debug)
                        Debug.Print("Pushing back the node into the open list with an increased h.");
                    this.openList.Add(currentNode);
                    this.nodesPushedBack++;
                    continue;
                    // Notice that even though we may iterate over conflicts later,
                    // even there is a conflict that we can identify as cardinal,
                    // then the first conflict chosen _will_ be cardinal, so this is
                    // the only place we need allow pushing nodes back.
                    // We may discover cardinal conflicts in hindsight later, but there
                    // would be no point in pushing their node back at that point,
                    // as we would've already made the split by then.
                }

                this.addToGlobalConflictCount(currentNode.GetConflict()); // TODO: Make CBS_GlobalConflicts use nodes that do this automatically after choosing a conflict

                if (debug)
                    currentNode.Print();

                if (currentNode.totalCost > currentCost) // Needs to be here because the goal may have a cost unseen before
                {
                    currentCost = currentNode.totalCost;
                    this.nodesExpandedWithGoalCost = 0;
                }
                else if (currentNode.totalCost == currentCost) // check needed because macbs node cost isn't exactly monotonous
                {
                    this.nodesExpandedWithGoalCost++;
                }

                // Check if node is the goal
                if (currentNode.GoalTest())
                {
                    //Debug.Assert(currentNode.totalCost >= maxExpandedNodeCostPlusH, "CBS goal node found with lower cost than the max cost node ever expanded: " + currentNode.totalCost + " < " + maxExpandedNodeCostPlusH);
                    // This is subtle, but MA-CBS may expand nodes in a non non-decreasing order:
                    // If a node with a non-optimal constraint is expanded and we decide to merge the agents,
                    // the resulting node can have a lower cost than before, since we ignore the non-optimal constraint
                    // because the conflict it addresses is between merged nodes.
                    // The resulting lower-cost node will have other constraints, that will raise the cost of its children back to at least its original cost,
                    // since the node with the non-optimal constraint was only expanded because its competitors that had an optimal
                    // constraint to deal with the same conflict apparently found the other conflict that I promise will be found,
                    // and so their cost was not smaller than this sub-optimal node.
                    // To make MA-CBS costs non-decreasing, we can choose not to ignore constraints that deal with conflicts between merged nodes.
                    // That way, the sub-optimal node will find a sub-optimal merged solution and get a high cost that will push it deep into the open list.
                    // But the cost would be to create a possibly sub-optimal merged solution where an optimal solution could be found instead, and faster,
                    // since constraints make the low-level heuristic perform worse.
                    // For an example for this subtle case happening, see problem instance 63 of the random grid with 4 agents,
                    // 55 grid cells and 9 obstacles.

                    if (debug)
                        Debug.WriteLine("-----------------");
                    this.totalCost = currentNode.totalCost;
                    this.solution = currentNode.CalculateJointPlan();
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.goalNode = currentNode; // Saves the single agent plans and costs
                    // The joint plan is calculated on demand.
                    this.Clear(); // Goal found - we're not going to resume this search
                    this.CleanGlobals();
                    return true;
                }

                if (currentNode.totalCost >= this.targetCost || // Node is good enough
                    //(this.targetCost != int.MaxValue &&
                     //this.lowLevelGenerated > Math.Pow(Constants.NUM_ALLOWED_DIRECTIONS, this.instance.m_vAgents.Length))
                    this.solver.GetAccumulatedGenerated() > this.lowLevelGeneratedCap || // Stop because this is taking too long.
                                                                                         // We're looking at _generated_ low level nodes since that's an indication to the amount of work done,
                                                                                         // while expanded nodes is an indication of the amount of good work done.
                                                                                         // b**k is the maximum amount of nodes we'll generate if we expand this node with A*.
                    (this.milliCap != int.MaxValue && // (This check is much cheaper than the method call)
                     this.runner.ElapsedMilliseconds() > this.milliCap)) // Search is taking too long.
                {
                    if (debug)
                        Debug.WriteLine("-----------------");
                    this.totalCost = maxExpandedNodeCostPlusH; // This is the min possible cost so far.
                    this.openList.Add(currentNode); // To be able to continue the search later
                    this.CleanGlobals();
                    return false;
                }

                if (maxExpandedNodeCostPlusH < currentNode.totalCost + currentNode.h)
                {
                    maxExpandedNodeCostPlusH = currentNode.totalCost + currentNode.h;
                    if (debug)
                        Debug.Print("New max F: {0}", maxExpandedNodeCostPlusH);
                }
                
                // Expand
                bool wasUnexpandedNode = (currentNode.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED &&
                                         currentNode.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED);
                Expand(currentNode);
                if (wasUnexpandedNode)
                    highLevelExpanded++;
                // Consider moving the following into Expand()
                if (currentNode.agentAExpansion == CbsNode.ExpansionState.EXPANDED &&
                    currentNode.agentBExpansion == CbsNode.ExpansionState.EXPANDED) // Fully expanded
                    currentNode.Clear();
            }

            this.totalCost = Constants.NO_SOLUTION_COST;
            this.Clear(); // unsolvable problem - we're not going to resume it
            this.CleanGlobals();
            return false;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="node"></param>
        /// <param name="children"></param>
        /// <param name="adoptBy">If not given, adoption is done by expanded node</param>
        /// <returns>true if adopted - need to rerun this method, ignoring the returned children from this call, bacause adoption was performed</returns>
        protected bool ExpandImpl(CbsNode node, out IList<CbsNode> children, out bool reinsertParent)
        {
            CbsConflict conflict = node.GetConflict();
            children = new List<CbsNode>();

            CbsNode child;
            reinsertParent = false;
            int closedListHitChildCost;
            bool leftSameCost = false; // To quiet the compiler
            bool rightSameCost = false;
          

            // Generate left child:
            child = ConstraintExpand(node, true, out closedListHitChildCost);
            if (child != null)
            {
                if (child == node) // Expansion deferred
                    reinsertParent = true;
                else // New child
                {
                    children.Add(child);
                    leftSameCost = child.totalCost == node.totalCost;
                }
            }
            else  // A timeout occured, or the child was already in the closed list.
            {
                if (closedListHitChildCost != -1)
                    leftSameCost = closedListHitChildCost == node.totalCost;
            }

            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                return false;
            
            // Generate right child:
            child = ConstraintExpand(node, false, out closedListHitChildCost);
            if (child != null)
            {
                if (child == node) // Expansion deferred
                    reinsertParent = true;
                else // New child
                {
                    children.Add(child);
                    rightSameCost = child.totalCost == node.totalCost;
                }
            }
            else  // A timeout occured, or the child was already in the closed list.
            {
                if (closedListHitChildCost != -1)
                    rightSameCost = closedListHitChildCost == node.totalCost;
            }
            

            return false;
        }

        public virtual void Expand(CbsNode node)
        {
            ushort parentCost = node.totalCost;
            ushort parentH = node.h;
            IList<CbsNode> children = null; // To quiet the compiler
            bool reinsertParent = false; // To quiet the compiler

 
            this.ExpandImpl(node, out children, out reinsertParent);


            // Both children considered. None adopted. Add them to the open list, and re-insert the partially expanded parent too if necessary.
            if (reinsertParent)
                this.openList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts

            foreach (var child in children)
            {
                closedList.Add(child, child);

                if (child.totalCost == parentCost) // Total cost didn't increase (yet)
                {
                    child.h = parentH;


                }

                if (child.totalCost <= this.maxCost)
                {
                    this.highLevelGenerated++;
                    openList.Add(child);
                }
            }
            
            
        }

        

        protected CbsNode ConstraintExpand(CbsNode node, bool doLeftChild, out int closedListHitChildCost)
        {
            CbsConflict conflict = node.GetConflict();
            int conflictingAgentIndex = doLeftChild? conflict.agentAIndex : conflict.agentBIndex;
            CbsNode.ExpansionState expansionsState = doLeftChild ? node.agentAExpansion : node.agentBExpansion;
            CbsNode.ExpansionState otherChildExpansionsState = doLeftChild ? node.agentBExpansion : node.agentAExpansion;
            string agentSide = doLeftChild? "left" : "right";
            int planSize = node.allSingleAgentPlans[conflictingAgentIndex].GetSize();
            int groupSize = node.GetGroupSize(conflictingAgentIndex);
            closedListHitChildCost = -1;

            if ((Constants.Variant == Constants.ProblemVariant.ORIG &&
                expansionsState == CbsNode.ExpansionState.NOT_EXPANDED && conflict.vertex == true &&
                 conflict.timeStep >= node.allSingleAgentCosts[conflictingAgentIndex] && // TODO: Can't just check whether the node is at its goal - the plan may involve it passing through its goal and returning to it later because of preexisting constraints.
                 node.h < conflict.timeStep + 1 - node.allSingleAgentCosts[conflictingAgentIndex] && // Otherwise we won't be increasing its h and there would be no reason to delay expansion
                 groupSize == 1) || // Otherwise an agent in the group can be forced to take a longer route without increasing the group's cost because another agent would be able to take a shorter route.
               (Constants.Variant == Constants.ProblemVariant.NEW &&
                expansionsState == CbsNode.ExpansionState.NOT_EXPANDED && conflict.vertex == true &&
                ((conflict.timeStep > planSize - 1 && node.h < 2) ||
                 (conflict.timeStep == planSize - 1 && node.h < 1)) &&
                groupSize == 1)) // Otherwise an agent in the group can be forced to take a longer route without increasing the group's cost because another agent would be able to take a shorter route.
            // Conflict happens when or after the agent reaches its goal, and the agent is in a single-agent group.
            // With multi-agent groups, banning the goal doesn't guarantee a higher cost solution,
            // since if an agent is forced to take a longer route it may enable another agent in the group
            // to take a shorter route, getting an alternative solution of the same cost
            // The child would cost a lot because:
            // A) All WAIT moves in the goal before leaving it now add to the g (if we're in the original problem variant).
            // B) We force the low level to compute a path longer than the optimal,
            //    and with a bad suprise towards the end in the form of a constraint,
            //    so the low-level's SIC heuristic performs poorly.
            // C) We're banning the GOAL from all directions (since this is a vertex conflict),
            //    so any alternative plan will at least cost 1 more.
            //    We're ignoring edge conflicts because they can only happen at the goal when reaching it,
            //    and aren't guaranteed to increase the cost because the goal can still be possibly reached from another edge.
            {
                if (otherChildExpansionsState == CbsNode.ExpansionState.DEFERRED)
                        throw new Exception("Unexpected: Expansion of both children deffered, but this is a vertex conflict so that means the targets for the two agents are equal, which is illegal");

                if (debug)
                    Debug.WriteLine("Skipping " + agentSide + " child for now");
                if (doLeftChild)
                    node.agentAExpansion = CbsNode.ExpansionState.DEFERRED;
	            else
                    node.agentBExpansion = CbsNode.ExpansionState.DEFERRED;
                // Add the minimal delta in the child's cost:
                // since we're banning the goal at conflict.timeStep, it must at least do conflict.timeStep+1 steps
                if (Constants.Variant == Constants.ProblemVariant.ORIG)
                    node.h = (ushort)(conflict.timeStep + 1 - node.allSingleAgentCosts[conflictingAgentIndex]);
                else if (Constants.Variant == Constants.ProblemVariant.NEW)
                {
                    if (conflict.timeStep > planSize - 1) // Agent will need to step out and step in to the goal, at least
                        node.h = 2;
                    else // Conflict is just when agent enters the goal, it'll have to at least wait one timestep.
                        node.h = 1;
                }
                return node;
            }
            else if (expansionsState != CbsNode.ExpansionState.EXPANDED)
            // Agent expansion already skipped in the past or not forcing it from its goal - finally generate the child:
            {
                if (debug)
                    Debug.WriteLine("Generating " + agentSide +" child");

                if (doLeftChild)
                    node.agentAExpansion = CbsNode.ExpansionState.EXPANDED;
                else
                    node.agentBExpansion = CbsNode.ExpansionState.EXPANDED;
                
                var newConstraint = new CbsConstraint(conflict, instance, doLeftChild);
                CbsNode child = new CbsNode(node, newConstraint, conflictingAgentIndex);

                if (closedList.ContainsKey(child) == false)
                {
                    int minCost = -1;
                    //if (this.useMddHeuristic)
                    //    minCost = node.GetGroupCost(conflictingAgentIndex) + 1;
                    bool success = child.Replan(conflictingAgentIndex, this.minDepth, null, -1, minCost); // The node takes the max between minDepth and the max time over all constraints.

                    if (success == false)
                        return null; // A timeout probably occured

                    if (debug)
                    {
                        Debug.WriteLine("Child hash: " + child.GetHashCode());
                        Debug.WriteLine("Child cost: " + child.totalCost);
                        Debug.WriteLine("Child min ops to solve: " + child.minOpsToSolve);
                        Debug.WriteLine("Child num of agents that conflict: " + child.totalInternalAgentsThatConflict);
                        Debug.WriteLine("Child num of internal conflicts: " + child.totalConflictsBetweenInternalAgents);
                        Debug.WriteLine("");
                    }

                    if (child.totalCost < node.totalCost && groupSize == 1) // Catch the error early
                    {
                        child.Print();
                        Debug.WriteLine("Child plan: (cost {0})", child.allSingleAgentCosts[conflictingAgentIndex]);
                        child.allSingleAgentPlans[conflictingAgentIndex].PrintPlan();
                        Debug.WriteLine("Parent plan: (cost {0})", node.allSingleAgentCosts[conflictingAgentIndex]);
                        node.allSingleAgentPlans[conflictingAgentIndex].PrintPlan();
                        Debug.Assert(false, "Single agent node with lower cost than parent! " + child.totalCost + " < " + node.totalCost);
                    }

                    return child;
                }
                else
                {
                    this.closedListHits++;
                    closedListHitChildCost = this.closedList[child].totalCost;
                    if (debug)
                        Debug.WriteLine("Child already in closed list!");
                }
            }
            else
            {
                if (debug)
                    Debug.WriteLine("Child already generated before");
            }

            return null;
        }


        protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

        public virtual Plan GetPlan()
        {
            if (this.solution == null)
                this.solution = this.goalNode.CalculateJointPlan();
            return this.solution;
        }

        public int GetSolutionDepth() { return this.solutionDepth; }
        
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        
        public SinglePlan[] GetSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }

        public virtual int[] GetSingleCosts()
        {
            return goalNode.allSingleAgentCosts;
        }

        public int GetHighLevelExpanded() { return highLevelExpanded; }
        public int GetHighLevelGenerated() { return highLevelGenerated; }
        public int GetLowLevelExpanded() { return this.solver.GetAccumulatedExpanded(); }
        public int GetLowLevelGenerated() { return this.solver.GetAccumulatedGenerated(); }
        public int GetExpanded() { return highLevelExpanded; }
        public int GetGenerated() { return highLevelGenerated; }
        public int GetAccumulatedExpanded() { return accHLExpanded; }
        public int GetAccumulatedGenerated() { return accHLGenerated; }
        public int GetMaxGroupSize() { return this.maxSizeGroup; }
    }
    
}
