using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

using System.Text;
using System.IO;

namespace CPF_experiment
{
    [DebuggerDisplay("hash = {GetHashCode()}, f = {f}")]
    public class CbsNode : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        public ushort totalCost;
        public ushort h;
        public SinglePlan[] allSingleAgentPlans;
        public int[] allSingleAgentCosts;
        /// <summary>
        /// A lower estimate of the number of operations (replanning or merging) needed to solve the node.
        /// Used for tie-breaking.
        /// </summary>
        public int minOpsToSolve;
        /// <summary>
        /// For each agent in the problem instance, saves the number of agents from the problem instance that it conflicts with.
        /// Used for choosing the next conflict to resolve by replanning/merging/shuffling, and for tie-breaking.
        /// </summary>
        public int[] countsOfInternalAgentsThatConflict;
        /// <summary>
        /// Counts the number of external agents this node conflicts with.
        /// Used for tie-breaking.
        /// </summary>
        public int totalExternalAgentsThatConflict;
        /// <summary>
        /// Used for tie-breaking.
        /// </summary>
        public int totalConflictsWithExternalAgents;
        /// <summary>
        /// For each agent in the problem instance, maps agent _nums_ it conflicts with, internal or external,
        /// to the number of conflicts betweem them.
        /// Used for book-keeping to maintain countsOfInternalAgentsThatConflict,
        /// totalExternalAgentsThatConflict and minOpsToSolve, and other counts.
        /// </summary>
        public Dictionary<int, int>[] conflictCountsPerAgent;
        /// <summary>
        /// For each agent in the problem instance, maps agent _nums_ of agents it collides with to the time of their first collision.
        /// </summary>
        public Dictionary<int, List<int>>[] conflictTimesPerAgent;
        /// For each agent in the problem instance, maps agent _nums_ of agents it collides with to the time bias of their first collision. (for range conflict)
        /// </summary>
        private int binaryHeapIndex;
        public CbsConflict conflict;
        public CbsConstraint constraint;
        /// <summary>
        /// Forcing an agent to be at a certain place at a certain time
        /// </summary>
        public CbsNode prev;
        public ushort depth;
        public ushort[] agentsGroupAssignment;
        public ushort replanSize;
        public enum ExpansionState: byte
        {
            NOT_EXPANDED = 0,
            DEFERRED,
            EXPANDED
        }
        /// <summary>
        /// For partial expansion
        /// </summary>
        public ExpansionState agentAExpansion;
        /// <summary>
        /// For partial expansion
        /// </summary>
        public ExpansionState agentBExpansion;
        //public ProblemInstance problem;
        protected ICbsSolver solver;
        protected ICbsSolver singleAgentSolver;
        protected CBS cbs;
        public Dictionary<int, int> agentNumToIndex;
        public bool parentAlreadyLookedAheadOf;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int totalInternalAgentsThatConflict;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int largerConflictingGroupSize;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int totalConflictsBetweenInternalAgents;

        public CbsNode(int numberOfAgents, ICbsSolver solver, ICbsSolver singleAgentSolver, CBS cbs, ushort[] agentsGroupAssignment = null)
        {
            this.cbs = cbs;
            allSingleAgentPlans = new SinglePlan[numberOfAgents];
            allSingleAgentCosts = new int[numberOfAgents];
            countsOfInternalAgentsThatConflict = new int[numberOfAgents];
            conflictCountsPerAgent = new Dictionary<int, int>[numberOfAgents]; // Populated after Solve()
            conflictTimesPerAgent = new Dictionary<int, List<int>>[numberOfAgents]; // Populated after Solve()
            if (agentsGroupAssignment == null)
            {
                this.agentsGroupAssignment = new ushort[numberOfAgents];
                for (ushort i = 0; i < numberOfAgents; i++)
                    this.agentsGroupAssignment[i] = i;
            }
            else
                this.agentsGroupAssignment = agentsGroupAssignment.ToArray<ushort>();
            agentNumToIndex = new Dictionary<int, int>();
            for (int i = 0; i < numberOfAgents; i++)
            {
                agentNumToIndex[this.cbs.GetProblemInstance().m_vAgents[i].agent.agentNum] = i;
            }
            depth = 0;
            replanSize = 1;
            agentAExpansion = ExpansionState.NOT_EXPANDED;
            agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.prev = null;
            this.constraint = null;
            this.solver = solver;
            this.singleAgentSolver = singleAgentSolver;
        }

        public int agentToReplan;

        /// <summary>
        /// Child from branch action constructor
        /// </summary>
        /// <param name="father"></param>
        /// <param name="newConstraint"></param>
        /// <param name="agentToReplan"></param>
        public CbsNode(CbsNode father, CbsConstraint newConstraint, int agentToReplan)
        {
            this.agentToReplan = agentToReplan;
            this.allSingleAgentPlans = father.allSingleAgentPlans.ToArray<SinglePlan>();
            this.allSingleAgentCosts = father.allSingleAgentCosts.ToArray<int>();
            this.countsOfInternalAgentsThatConflict = father.countsOfInternalAgentsThatConflict.ToArray<int>();
            this.conflictCountsPerAgent = new Dictionary<int, int>[father.conflictCountsPerAgent.Length];
            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                this.conflictCountsPerAgent[i] = new Dictionary<int, int>(father.conflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            this.conflictTimesPerAgent = new Dictionary<int, List<int>>[father.conflictTimesPerAgent.Length];
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                this.conflictTimesPerAgent[i] = new Dictionary<int, List<int>>(); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
                foreach (var kvp in father.conflictTimesPerAgent[i])
                    this.conflictTimesPerAgent[i][kvp.Key] = new List<int>(kvp.Value);
            }
           
            this.agentsGroupAssignment = father.agentsGroupAssignment.ToArray<ushort>();
            this.agentNumToIndex = father.agentNumToIndex;
            this.prev = father;
            this.constraint = newConstraint;
            this.depth = (ushort)(this.prev.depth + 1);
            this.agentAExpansion = ExpansionState.NOT_EXPANDED;
            this.agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.replanSize = 1;
            this.solver = father.solver;
            this.singleAgentSolver = father.singleAgentSolver;
            this.cbs = father.cbs;
        }

        /// <summary>
        /// Child from merge action constructor. FIXME: Code dup with previous constructor.
        /// </summary>
        /// <param name="father"></param>
        /// <param name="mergeGroupA"></param>
        /// <param name="mergeGroupB"></param>
        public CbsNode(CbsNode father, int mergeGroupA, int mergeGroupB)
        {
            this.allSingleAgentPlans = father.allSingleAgentPlans.ToArray<SinglePlan>();
            this.allSingleAgentCosts = father.allSingleAgentCosts.ToArray<int>();
            this.countsOfInternalAgentsThatConflict = father.countsOfInternalAgentsThatConflict.ToArray<int>();
            this.conflictCountsPerAgent = new Dictionary<int, int>[father.conflictCountsPerAgent.Length];
            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                this.conflictCountsPerAgent[i] = new Dictionary<int, int>(father.conflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            this.conflictTimesPerAgent = new Dictionary<int, List<int>>[father.conflictTimesPerAgent.Length];
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                this.conflictTimesPerAgent[i] = new Dictionary<int, List<int>>(); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
                foreach (var kvp in father.conflictTimesPerAgent[i])
                    this.conflictTimesPerAgent[i][kvp.Key] = new List<int>(kvp.Value);
            }
           
            this.agentsGroupAssignment = father.agentsGroupAssignment.ToArray<ushort>();
            this.agentNumToIndex = father.agentNumToIndex;
            this.prev = father;
            this.constraint = null;
            this.depth = (ushort)(this.prev.depth + 1);
            this.agentAExpansion = ExpansionState.NOT_EXPANDED;
            this.agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.replanSize = 1;
            this.solver = father.solver;
            this.singleAgentSolver = father.singleAgentSolver;
            this.cbs = father.cbs;

        }

        public int f
        {
            get { return this.totalCost + this.h; }
        }

        /// <summary>
        /// Solves the entire node - finds a plan for every agent group.
        /// Since this method is only called for the root of the constraint tree, every agent is in its own group.
        /// </summary>
        /// <param name="depthToReplan"></param>
        /// <returns></returns>
        public bool Solve(int depthToReplan)
        {
            this.totalCost = 0;
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var internalCAT = new ConflictAvoidanceTable();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints(); // Probably empty as this is probably the root of the CT.
            var CAT = (Dictionary_U<TimedMove, int>)problem.parameters[CBS.CAT];
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS.CONSTRAINTS];


            Dictionary<int, int> agentsWithConstraints = null;
            if (constraints.Count != 0)
            {
                int maxConstraintTimeStep = constraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
                agentsWithConstraints = constraints.Select<CbsConstraint, int>(constraint => constraint.agentNum).Distinct().ToDictionary<int, int>(x => x); // ToDictionary because there's no ToSet...
            }

            constraints.Join(newConstraints);
            CAT.Join(internalCAT);

            // This mechanism of adding the constraints to the possibly pre-existing constraints allows having
            // layers of CBS solvers, each one adding its own constraints and respecting those of the solvers above it.

            // Find all the agents groups:
            var subGroups = new List<AgentState>[problem.m_vAgents.Length];
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (subGroups[i] == null)
                    subGroups[i] = new List<AgentState>();
                subGroups[this.agentsGroupAssignment[i]].Add(problem.m_vAgents[i]);
            }

            bool success = true;

            int maxPlanSize = 0;
            for (int i = 0; i < problem.m_vAgents.Length; i++)
            {
                if (this.agentsGroupAssignment[i] != i) // This isn't the first agent in its group - we've already solved its group.
                    continue;
                List<AgentState> subGroup = subGroups[i];

                bool agentGroupHasConstraints = (agentsWithConstraints != null) && subGroup.Any<AgentState>(state => agentsWithConstraints.ContainsKey(state.agent.agentNum));

                // Solve for a single agent:
                if (agentGroupHasConstraints == false  &&
                    subGroup.Count == 1) // Top-most CBS with no must constraints on this agent. Shortcut available (that doesn't consider the CAT, though)
                {
                    allSingleAgentPlans[i] = new SinglePlan(problem.m_vAgents[i]); // All moves up to starting pos
                    allSingleAgentPlans[i].agentNum = problem.m_vAgents[this.agentsGroupAssignment[i]].agent.agentNum; // Use the group's representative
                    SinglePlan optimalPlan = problem.GetSingleAgentOptimalPlan(
                                    problem.m_vAgents[i],
                                    out this.conflictCountsPerAgent[i], out this.conflictTimesPerAgent[i]);
                    allSingleAgentPlans[i].ContinueWith(optimalPlan);
                    allSingleAgentCosts[i] = problem.m_vAgents[i].g + problem.GetSingleAgentOptimalCost(problem.m_vAgents[i]);
                    totalCost += (ushort)allSingleAgentCosts[i];

                    this.UpdateAtGoalConflictCounts(i, maxPlanSize, CAT);   
                }
                else
                {
                    success = this.Replan(i, depthToReplan, subGroup, maxPlanSize);

                    if (!success) // Usually means a timeout occured.
                        break;
                }
                // Add plan to the internalCAT
                foreach (AgentState agentState in subGroup)
                {
                    maxPlanSize = Math.Max(maxPlanSize, allSingleAgentPlans[this.agentNumToIndex[agentState.agent.agentNum]].GetSize());
                    internalCAT.AddPlan(allSingleAgentPlans[this.agentNumToIndex[agentState.agent.agentNum]]);
                }
            }

            CAT.Separate(internalCAT);
            constraints.Separate(newConstraints);

            if (!success)
                return false;
            if (Run.toPrint)
            {
                printLinkedList(singleAgentPathsToList(allSingleAgentPlans));
                printConflicts(allSingleAgentPlans);
                Console.WriteLine("");
            }
            // Update conflict counts: All agents but the last saw an incomplete CAT. Update counts backwards.
            for (int i = this.conflictCountsPerAgent.Length - 1; i >= 0; i--)
            {
                foreach (KeyValuePair<int, int> pair in this.conflictCountsPerAgent[i])
                {
                    if (this.agentNumToIndex.ContainsKey(pair.Key) && // An internal conflict, rather than external
                        this.agentNumToIndex[pair.Key] < i)                                 // Just an optimization. Would also be correct without this check.
                    {
                        this.conflictCountsPerAgent[this.agentNumToIndex[pair.Key]] // Yes, index here, num there
                            [problem.m_vAgents[i].agent.agentNum] = pair.Value; // Collisions are symmetrical, and agent "key" didn't see the route for agent "i" when planning.

                        if(this.conflictTimesPerAgent[i][pair.Key] != null)
                        {
                            this.conflictTimesPerAgent[this.agentNumToIndex[pair.Key]]
                            [problem.m_vAgents[i].agent.agentNum] = new List<int>(this.conflictTimesPerAgent[i][pair.Key]);
                        }


                        
                    }
                }
            }
            if(Run.toPrint)
                printConflicts(allSingleAgentPlans);
            this.CountConflicts();

            //debug
            List<TimedMove> listTime = new List<TimedMove>();
            foreach(TimedMove tm in internalCAT.timedMovesToAgentNumList.Keys)
            {
                listTime.Add(tm);
            }
            //end debug

            this.CalcMinOpsToSolve();

            this.isGoal = this.countsOfInternalAgentsThatConflict.All(i => i == 0);
            return true;
        }

        

        private int getMaxSinglePathSize (SinglePlan[] allSingleAgentPlans)
        {
            int max = 0;
            foreach(SinglePlan sp in allSingleAgentPlans)
            {
                if (sp != null && sp.locationAtTimes.Count > max)
                    max = sp.locationAtTimes.Count;
            }
            return max;
        }


        private void printConflicts(SinglePlan[] allSingleAgentPlans)
        {
            for (int agentDictionaryIndex = 0; agentDictionaryIndex < conflictCountsPerAgent.Count(); agentDictionaryIndex ++ )
            {
                Dictionary<int,int> agentCountDictionary                    = conflictCountsPerAgent[agentDictionaryIndex];
                Dictionary<int, List<int>> agentTimesDictionary             = conflictTimesPerAgent[agentDictionaryIndex];
                
                foreach(int key in agentTimesDictionary.Keys)
                {
                    List<int> agentTimesDictionaryList       = agentTimesDictionary[key];
                    for(int i = 0; i < agentTimesDictionaryList.Count; i++)
                    {
                        Move move;
                        if (agentTimesDictionaryList[i] >= allSingleAgentPlans[agentDictionaryIndex].locationAtTimes.Count)
                            move = allSingleAgentPlans[agentDictionaryIndex].locationAtTimes[allSingleAgentPlans[agentDictionaryIndex].locationAtTimes.Count - 1];
                        else
                            move = allSingleAgentPlans[agentDictionaryIndex].locationAtTimes[agentTimesDictionaryList[i]];
                        Console.WriteLine("Agent " + agentDictionaryIndex + " Collinding Agent " + key + " At Time " + agentTimesDictionaryList[i] + " Location " + move);
                    }
                }
            }
        }

        private LinkedList<List<Move>> singleAgentPathsToList(SinglePlan[] paths)
        {
            LinkedList<List<Move>> list = new LinkedList<List<Move>>();
            for(int i = 0; i < paths.OrderByDescending(x => x.locationAtTimes.Count).First().locationAtTimes.Count; i++)
            {
                List<Move> current = new List<Move>();
                foreach(SinglePlan p in paths)
                {
                    if (p.locationAtTimes.Count > i)
                        current.Add(p.locationAtTimes[i]);
                    else
                        current.Add(p.locationAtTimes[p.locationAtTimes.Count - 1]);
                }
                list.AddLast(current);
            }
            return list;
        }


        private void printLinkedList(LinkedList<List<Move>> toPrint, bool writeToFile = false)
        {
            if (toPrint.Count == 0)
                return;
            PrintLine(writeToFile);
            LinkedListNode<List<Move>> node = toPrint.First;
            string[] columns = new string[node.Value.Count + 1];
            columns[0] = "";
            for (int agentNumber = 1; agentNumber < node.Value.Count + 1; agentNumber++)
            {
                columns[agentNumber] = (agentNumber - 1).ToString();

            }
            node = toPrint.First;
            PrintRow(writeToFile, columns);
            PrintLine(writeToFile);

            int time = 0;
            while (node != null)
            {
                columns = new string[node.Value.Count + 1];
                columns[0] = time.ToString();
                time++;
                List<Move> currentMoves = node.Value;
                for (int i = 0; i < currentMoves.Count; i++)
                {
                    Move currentMove = currentMoves[i];
                    columns[i + 1] = currentMove.x + "," + currentMove.y;
                }
                PrintRow(writeToFile, columns);
                node = node.Next;
            }
            PrintLine(writeToFile);
        }
        static int tableWidth = 200;

        static void PrintLine(bool writeToFile)
        {
            if (!writeToFile)
                Console.WriteLine(new string('-', tableWidth));
            else
            {
                string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
                string filePath = pathDesktop + "\\RobustLog.txt";
                using (StreamWriter file = File.AppendText(filePath))
                //using (System.IO.StreamWriter file = new System.IO.StreamWriter(filePath))
                {
                    file.WriteLine(new string('-', tableWidth));
                }
            }

        }

        static void PrintRow(bool writeToFile, params string[] columns)
        {
            int width = (tableWidth - columns.Length) / columns.Length;
            string row = "|";

            foreach (string column in columns)
            {
                row += AlignCentre(column, width) + "|";
            }
            if (!writeToFile)
                Console.WriteLine(row);
            else
            {
                string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
                string filePath = pathDesktop + "\\RobustLog.txt";
                using (StreamWriter file = File.AppendText(filePath))
                {
                    file.WriteLine(row);
                }
            }

        }

        static string AlignCentre(string text, int width)
        {
            text = text.Length > width ? text.Substring(0, width - 3) + "..." : text;

            if (string.IsNullOrEmpty(text))
            {
                return new string(' ', width);
            }
            else
            {
                return text.PadRight(width - (width - text.Length) / 2).PadLeft(width);
            }
        }

        /// <summary>
        /// Replan for a given agent (when constraints for that agent have changed).
        /// </summary>
        /// <param name="agentForReplan"></param>
        /// <param name="depthToReplan">CBS's minDepth param. !@# Should be called minTimeStep?</param>
        /// <param name="subGroup">If given, assume CAT is already populated and use this subGroup</param>
        /// <param name="maxPlanSize">If given, use it instead of computing it</param>
        /// <returns></returns>
        public bool Replan(int agentForReplan, int depthToReplan, List<AgentState> subGroup = null, int maxPlanSize = -1, int minCost = -1)
        {
            ProblemInstance problem = this.cbs.GetProblemInstance();
            Dictionary_U<TimedMove, int> CAT = (Dictionary_U<TimedMove, int>)problem.parameters[CBS.CAT];

            ConflictAvoidanceTable internalCAT = null; // To quiet the compiler
            int groupNum = this.agentsGroupAssignment[agentForReplan];
            bool underSolve = true;

            if (subGroup == null)
            {
                underSolve = false;
                // Construct the subgroup of agents that are of the same group as agentForReplan,
                // and add the plans of all other agents to CAT
                internalCAT = new ConflictAvoidanceTable();
                subGroup = new List<AgentState>();
                maxPlanSize = this.allSingleAgentPlans.Max<SinglePlan>(plan => plan.GetSize());
                for (int i = 0; i < agentsGroupAssignment.Length; i++)
                {
                    if (this.agentsGroupAssignment[i] == groupNum)
                        subGroup.Add(problem.m_vAgents[i]);
                    else
                        internalCAT.AddPlan(allSingleAgentPlans[i]);
                }
                
                CAT.Join(internalCAT);
            }
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS.CONSTRAINTS];
            
            this.replanSize = (ushort)subGroup.Count;

            ICbsSolver relevantSolver = this.solver;
            if (subGroup.Count == 1)
                relevantSolver = this.singleAgentSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());

            constraints.Join(newConstraints);
            

            Dictionary<int, int> subGroupAgentNums = subGroup.Select<AgentState, int>(state => state.agent.agentNum).ToDictionary<int, int>(num => num); // No need to call Distinct(). Each agent appears at most once
            IEnumerable<CbsConstraint> myConstraints = constraints.Where<CbsConstraint>(constraint => subGroupAgentNums.ContainsKey(constraint.agentNum)); // TODO: Consider passing only myConstraints to the low level to speed things up.
            if (myConstraints.Count<CbsConstraint>() != 0)
            {
                int maxConstraintTimeStep = myConstraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }

           
            relevantSolver.Setup(subProblem, depthToReplan, this.cbs.runner, minCost);
            bool solved = relevantSolver.Solve();
            

            relevantSolver.AccumulateStatistics();
            relevantSolver.ClearStatistics();

            if (solved == false) // Usually means a timeout occured.
            {
                if (underSolve == false)
                    CAT.Separate(internalCAT); // Code dup, but if solved the CAT is needed for a bit longer.
                constraints.Separate(newConstraints);

                return false;
            }

            // Copy the SinglePlans for the solved agent group from the solver to the appropriate places in this.allSingleAgentPlans
            SinglePlan[] singlePlans = relevantSolver.GetSinglePlans();
            int[] singleCosts = relevantSolver.GetSingleCosts();
            Dictionary<int, int> perAgent = relevantSolver.GetExternalConflictCounts();
            Dictionary<int, List<int>> conflictTimes = relevantSolver.GetConflictTimes();
            
            for (int i = 0; i < subGroup.Count; i++)
            {
                int agentNum = subGroup[i].agent.agentNum;
                int agentIndex = this.agentNumToIndex[agentNum];
                this.allSingleAgentPlans[agentIndex] = singlePlans[i];
                this.allSingleAgentPlans[agentIndex].agentNum = problem.m_vAgents[groupNum].agent.agentNum; // Use the group's representative
                this.allSingleAgentCosts[agentIndex] = singleCosts[i];
                if (i == 0) // This is the group representative
                {
                    foreach (int conflictAgent in this.conflictTimesPerAgent[agentIndex].Keys)
                        for(int timeIndex = 0; timeIndex < conflictTimesPerAgent[agentIndex][conflictAgent].Count; timeIndex++)
                        {
                            int time = conflictTimesPerAgent[agentIndex][conflictAgent][timeIndex];
                            if(time == 0)
                            {
                                if (perAgent.Keys.Contains(conflictAgent))
                                {
                                    perAgent[conflictAgent]++;
                                    conflictTimes[conflictAgent].Add(0);
                                    
                                }
                                else
                                {
                                    perAgent.Add(conflictAgent, 1);
                                    List<int> newTimeList = new List<int>();
                                    newTimeList.Add(0);
                                    conflictTimes.Add(conflictAgent, newTimeList);
                                }
                            }
                        }
                    this.conflictCountsPerAgent[agentIndex] = perAgent;
                    this.conflictTimesPerAgent[agentIndex] = conflictTimes;
                }
                else
                {
                    if (underSolve == false)
                    {
                        this.conflictCountsPerAgent[agentIndex].Clear(); // Don't over-count. Leave it to the group's representative.
                        this.conflictTimesPerAgent[agentIndex].Clear();
                    }
                    else
                    {
                        this.conflictCountsPerAgent[agentIndex] = new Dictionary<int, int>();
                        this.conflictTimesPerAgent[agentIndex] = new Dictionary<int, List<int>>();
                    }
                }
            }

            // Update conflict counts with what happens after the plan finishes
            foreach (var agentNumAndAgentNum in subGroupAgentNums)
            {
                int i = this.agentNumToIndex[agentNumAndAgentNum.Key];
                this.UpdateAtGoalConflictCounts(i, maxPlanSize, CAT);
            }

            if (underSolve == false)
            {
                // Update conflictCountsPerAgent and conflictTimes for all agents
                int representativeAgentNum = subGroup[0].agent.agentNum;
                for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
			    {
                    int agentNum = problem.m_vAgents[i].agent.agentNum;
                    if (perAgent.ContainsKey(agentNum))
                    {
                        this.conflictCountsPerAgent[i][representativeAgentNum] = perAgent[agentNum];
                        this.conflictTimesPerAgent[i][representativeAgentNum] = new List<int>();

                        for (int time = 0; time < conflictTimes[agentNum].Count; time++)
                        {
                            this.conflictTimesPerAgent[i][representativeAgentNum].Add(conflictTimes[agentNum][time]);
                        }
                    }
                    else
                    {
                        this.conflictCountsPerAgent[i].Remove(representativeAgentNum);
                        this.conflictTimesPerAgent[i].Remove(representativeAgentNum);
                    }
			    }

                this.CountConflicts();
                this.CalcMinOpsToSolve();
                CAT.Separate(internalCAT);
            }

            constraints.Separate(newConstraints);

            // Calc totalCost
            this.totalCost = (ushort) Math.Max(this.allSingleAgentCosts.Sum(), this.totalCost); // Conserve totalCost from partial expansion if it's higher (only happens when shuffling a partially expanded node)

            this.isGoal = this.countsOfInternalAgentsThatConflict.All(i => i == 0);
            return true;
        }

       

        public void Print()
        {
            Debug.WriteLine("");
            Debug.WriteLine("");
            Debug.WriteLine("Node hash: " + this.GetHashCode());
            Debug.WriteLine("Total cost so far: " + this.totalCost);
            Debug.WriteLine("h: " + this.h);
            Debug.WriteLine("Min estimated ops needed: " + this.minOpsToSolve);
            Debug.WriteLine("Expansion state: " + this.agentAExpansion + ", " + this.agentBExpansion);
            Debug.WriteLine("Num of external agents that conflict: " + totalExternalAgentsThatConflict);
            Debug.WriteLine("Num of internal agents that conflict: " + totalInternalAgentsThatConflict);
            Debug.WriteLine("Num of conflicts between internal agents: " + totalConflictsBetweenInternalAgents);
            Debug.WriteLine("Node depth: " + this.depth);
            if (this.prev != null)
                Debug.WriteLine("Parent hash: " + this.prev.GetHashCode());
            IList<CbsConstraint> constraints = this.GetConstraintsOrdered();
            Debug.WriteLine(constraints.Count.ToString() + " relevant internal constraints so far: ");
            foreach (CbsConstraint constraint in constraints)
            {
                Debug.WriteLine(constraint);
            }
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var externalConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS.CONSTRAINTS];
            Debug.WriteLine(externalConstraints.Count.ToString() + " external constraints: ");
            foreach (CbsConstraint constraint in externalConstraints)
            {
                Debug.WriteLine(constraint);
            }
            Debug.WriteLine("Conflict: " + this.GetConflict());
            Debug.Write("Agent group assignments: ");
            for (int j = 0; j < this.agentsGroupAssignment.Length; j++)
            {
                Debug.Write(" " + this.agentsGroupAssignment[j]);
            }
            Debug.WriteLine("");
            Debug.Write("Single agent costs: ");
            for (int j = 0; j < this.allSingleAgentCosts.Length; j++)
            {
                Debug.Write(" " + this.allSingleAgentCosts[j]);
            }
            Debug.WriteLine("");
            Debug.Write("Internal agents that conflict with each agent: ");
            for (int j = 0; j < this.countsOfInternalAgentsThatConflict.Length; j++)
            {
                Debug.Write(" " + this.countsOfInternalAgentsThatConflict[j]);
            }
            Debug.WriteLine("");
            for (int j = 0; j < this.conflictCountsPerAgent.Length; j++)
            {
                //if (this.conflictCountsPerAgent[j].Count != 0)
                {
                    Debug.Write("Agent " + problem.m_vAgents[j].agent.agentNum + " conflict counts: ");
                    foreach (var pair in this.conflictCountsPerAgent[j])
	                {
                        Debug.Write(pair.Key.ToString() + ":" + pair.Value.ToString() + " ");
	                }
                    Debug.WriteLine("");

                }
            }
            for (int j = 0; j < this.conflictTimesPerAgent.Length; j++)
            {
                //if (this.conflictCountsPerAgent[j].Count != 0)
                {
                    Debug.Write("Agent " + problem.m_vAgents[j].agent.agentNum + " conflict times: ");
                    foreach (var pair in this.conflictTimesPerAgent[j])
                    {
                        Debug.Write(pair.Key.ToString() + ":[" + String.Join(",", pair.Value) + "], ");
                    }
                    Debug.WriteLine("");

                }
            }
           
            var plan = this.CalculateJointPlan();
            if (plan.GetSize() < 200)
                plan.PrintPlan();
            else
                Debug.WriteLine("Plan is too long to print");
            Debug.WriteLine("");
            Debug.WriteLine("");
        }

        /// <summary>
        /// Update conflict counts according to what happens after the plan finishes -
        /// needed if the plan is shorter than one of the previous plans and collides
        /// with it while at the goal.
        /// It's cheaper to do it this way than to force the solver the go more deeply.
        /// The conflict counts are saved at the group's representative.
        /// </summary>
        protected void UpdateAtGoalConflictCounts(int agentIndex, int maxPlanSize, IReadOnlyDictionary<TimedMove, List<int>> CAT)
        {
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var afterGoal = new TimedMove(
                problem.m_vAgents[agentIndex].agent.Goal.x, problem.m_vAgents[agentIndex].agent.Goal.y, Move.Direction.Wait, 0);
            for (int time = allSingleAgentPlans[agentIndex].GetSize(); time < maxPlanSize; time++)
            {
                afterGoal.time = time;
                afterGoal.UpdateConflictCounts(CAT,
                                                this.conflictCountsPerAgent[this.agentsGroupAssignment[agentIndex]],
                                                this.conflictTimesPerAgent[this.agentsGroupAssignment[agentIndex]]);
            }
        }



        /// <summary>
        /// Calculates the minimum number of replans to solve, and from it the minimum number of replans or merges to solve.
        /// 
        /// A replan can resolve all of the agent's conflicts by luck, even if it was only targeting a single conflict.
        ///
        /// To calculate the minimum number of replans to solve, 
        /// what we want is the size of the minimum vertex cover of the conflict graph.
        /// Sadly, it's an NP-hard problem. Its decision variant is NP-complete.
        /// Happily, it has a 2-approximation: Just choose both endpoints of each uncovered edge repeatedly until no uncovered edges are lef.
        /// 
        /// So we can just take half the count from that approximation.
        /// 
        /// Notice a merge is like two replans in one, so we might need to take ceil(num_replans/2).
        /// Luckily, in Cbs_LocalConflicts, a merge is only possible once every B+1 depth steps,
        /// because we only count selected conflicts (they're guaranteed to be unequal),
        /// so we can cap the number of possible merges and substract less.
        /// 
        /// In Cbs_GlobalConflicts, we could use the global table to discount some merges.
        /// </summary>
        protected void CalcMinOpsToSolve()
        {
            var vertexCover = new HashSet<int>();

            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
            {
                if (vertexCover.Contains(i)) // This node is already in the cover - all its edges are already covered.
                    continue;

                foreach (KeyValuePair<int, int> otherEndAgentNumAndCount in this.conflictCountsPerAgent[i])
                {
                    if (this.agentNumToIndex.ContainsKey(otherEndAgentNumAndCount.Key)) // It's an internal conflict
                    {
                        int otherEndIndex = this.agentNumToIndex[otherEndAgentNumAndCount.Key];
                        if (vertexCover.Contains(otherEndAgentNumAndCount.Key) == false) // The vertex isn't covered from its other end yet
                        {
                            vertexCover.Add(i);
                            vertexCover.Add(otherEndIndex);
                            break; // All of this node's edges are now covered.
                        }
                    }
                }
            }

            int minReplansToSolve = (int)Math.Ceiling(((double)vertexCover.Count) / 2); // We have a 2-approximation of the cover -
           
            this.minOpsToSolve = (int)minReplansToSolve;
        }

        protected void CountConflicts()
        {
            var externalConflictingAgentNums = new HashSet<int>();
            this.totalInternalAgentsThatConflict = 0;
            this.totalConflictsBetweenInternalAgents = 0;
            this.totalConflictsWithExternalAgents = 0;

            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
            {
                this.countsOfInternalAgentsThatConflict[i] = 0;

                if (conflictCountsPerAgent[i].Count != 0)
                    totalInternalAgentsThatConflict++;

                foreach (KeyValuePair<int, int> conflictingAgentNumAndCount in conflictCountsPerAgent[i])
                {
                    if (this.agentNumToIndex.ContainsKey(conflictingAgentNumAndCount.Key)/* &&
                        listContainsZeros(conflictTimesBiasPerAgent[i])*/) // It's an internal conflict
                    {
                        this.countsOfInternalAgentsThatConflict[i]++; // Counts one conflict for each agent the i'th agent conflicts with
                        this.totalConflictsBetweenInternalAgents += conflictingAgentNumAndCount.Value;
                    }
                    else
                    {
                        externalConflictingAgentNums.Add(conflictingAgentNumAndCount.Key);
                        this.totalConflictsWithExternalAgents += conflictingAgentNumAndCount.Value;
                        this.conflictTimesPerAgent[i].Remove(conflictingAgentNumAndCount.Key); // Not needed
                    }
                }
            }

            this.totalExternalAgentsThatConflict = externalConflictingAgentNums.Count;

            this.totalConflictsBetweenInternalAgents /= 2; // Each conflict was counted twice
            this.totalConflictsWithExternalAgents /= 2; // Each conflict was counted twice
        }

        private bool listContainsZeros(Dictionary<int, List<int>> list)
        {
            foreach (KeyValuePair<int, List<int>> item in list)
                foreach(int singleItem in item.Value)
                    if (singleItem == 0)
                        return true;
            return false;
        }

        /// <summary>
        /// Used to preserve state of conflict iteration.
        /// </summary>
        private IEnumerator<CbsConflict> nextConflicts;

        /// <summary>
        /// The iterator holds the state of the generator, with all the different queues etc - a lot of memory.
        /// We also clear the MDDs that were built - if no child uses them, they'll be garbage-collected.
        /// </summary>
        public void ClearConflictChoiceData()
        {
            this.nextConflicts = null;
        }

        /// Returns whether another conflict was found
        public bool ChooseNextConflict()
        {
            bool hasNext = this.nextConflicts.MoveNext();
            if (hasNext)
                this.conflict = this.nextConflicts.Current;
            return hasNext;
        }

        /// <summary>
        /// Chooses an internal conflict to work on.
        /// Resets conflicts iteration if it's used.
        /// </summary>
        public void ChooseConflict()
        {
            if (this.allSingleAgentPlans.Length == 1) // A single internal agent can't conflict with anything internally
                return;

            if (this.isGoal) // Goal nodes don't have conflicts
                return;

            if (this.conflict != null) // Conflict already chosen before
                return;

            int groupRepA = -1;
            int groupRepB = -1;
            int time = int.MaxValue;
            int time2;

            ChooseFirstConflict(out groupRepA, out groupRepB, out time, out time2);
            this.conflict = FindConflict(groupRepA, groupRepB, time, null, time2);
        }

       


        /// <summary>
        /// Assuming the groups conflict, return their conflict.
        /// </summary>
        /// <param name="aConflictingGroupMemberIndex"></param>
        /// <param name="bConflictingGroupMemberIndex"></param>
        /// <param name="time"></param>
        /// <returns></returns>
        private CbsConflict FindConflict(int aConflictingGroupMemberIndex, int bConflictingGroupMemberIndex, int time,
                                         ISet<int>[] groups = null, int time2 = -1)
        {
            if (time2 == -1)
                time2 = time;
            int specificConflictingAgentA, specificConflictingAgentB;
            this.FindConflicting(aConflictingGroupMemberIndex, bConflictingGroupMemberIndex, time,
                                 out specificConflictingAgentA, out specificConflictingAgentB,
                                 groups);
            ProblemInstance problem = this.cbs.GetProblemInstance();
            int initialTimeStep = problem.m_vAgents[0].lastMove.time; // To account for solving partially solved problems.
            // This assumes the makespan of all the agents is the same.
            Move first = allSingleAgentPlans[specificConflictingAgentA].GetLocationAt(time);
            Move second = allSingleAgentPlans[specificConflictingAgentB].GetLocationAt(time2);
            return new CbsConflict(specificConflictingAgentA, specificConflictingAgentB, first, second, time + initialTimeStep, time, time2);
        }

        /// <summary>
        /// Assuming the groups conflict, find the specific agents that conflict.
        /// Also sets largerConflictingGroupSize.
        /// </summary>
        /// <param name="aConflictingGroupMemberIndex"></param>
        /// <param name="bConflictingGroupMemberIndex"></param>
        /// <param name="time"></param>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private void FindConflicting(int aConflictingGroupMemberIndex, int bConflictingGroupMemberIndex, int time, out int a, out int b,
                                     ISet<int>[] groups = null)
        {
            a = aConflictingGroupMemberIndex;
            b = bConflictingGroupMemberIndex;
        }

        
        private void ChooseFirstConflict(out int groupRepA, out int groupRepB, out int time, out int time2)
        {

            groupRepA = -1; // To quiet the compiler
            groupRepB = -1; // To quiet the compiler
            time  = int.MaxValue;
            time2 = int.MaxValue;
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                foreach (var otherAgentNumAndConflictTimes in this.conflictTimesPerAgent[i])
                {
                    for (int conflictTimeIndex = 0; conflictTimeIndex < otherAgentNumAndConflictTimes.Value.Count; conflictTimeIndex++)
                    {
                        int conflictTime = otherAgentNumAndConflictTimes.Value[conflictTimeIndex];
                        if (conflictTime < time)   
                        {
                            time = conflictTime;    
                            time2 = time;
                            groupRepA = i;
                            groupRepB = this.agentNumToIndex[otherAgentNumAndConflictTimes.Key];
                        }
                    }
                }
            }
        }


        public CbsConflict GetConflict()
        {
            return this.conflict;
        }

        
        /// <summary>
        /// Uses the group assignments and the constraints.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                for (int i = 0; i < agentsGroupAssignment.Length; i++)
                {
                    ans += Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length] * agentsGroupAssignment[i];
                }

                HashSet<CbsConstraint> constraints = this.GetConstraints();

                foreach (CbsConstraint constraint in constraints)
                {
                    ans += constraint.GetHashCode();
                }

                return ans;
            }
        }

        /// <summary>
        /// Checks the group assignment and the constraints
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj) 
        {
            CbsNode other = (CbsNode)obj;

            if (this.agentsGroupAssignment.SequenceEqual<ushort>(other.agentsGroupAssignment) == false)
                return false;

            CbsNode current = this;
            HashSet<CbsConstraint> other_constraints = other.GetConstraints();
            HashSet<CbsConstraint> constraints = this.GetConstraints();

            foreach (CbsConstraint constraint in constraints)
            {
                if (other_constraints.Contains(constraint) == false)
                    return false;
                //current = current.prev;    dor comment
            }
            return constraints.Count == other_constraints.Count;
        }

        /// <summary>
        /// Worth doing because the node may always be in the closed list
        /// </summary>
        public void Clear()
        {
            this.allSingleAgentPlans = null;
            this.allSingleAgentCosts = null;
        }

        

        public int CompareTo(IBinaryHeapItem item)
        {
            CbsNode other = (CbsNode)item;


            int thisTotalCostPlusH = this.totalCost + this.h;
            int otherTotalCostPlusH = other.totalCost + other.h;

            if (thisTotalCostPlusH < otherTotalCostPlusH)
                return -1;
            if (thisTotalCostPlusH > otherTotalCostPlusH)
                return 1;

            return this.CompareToIgnoreH(other);
        }


        public int CompareToIgnoreH(CbsNode other, bool ignorePartialExpansion = false)
        {
            // Tie breaking:

            // Prefer larger cost - higher h usually means more work needs to be done
            if (this.totalCost > other.totalCost)
                return -1;
            if (this.totalCost < other.totalCost)
                return 1;

            // Prefer less external conflicts, even over goal nodes, as goal nodes with less external conflicts are better.
            // External conflicts are also taken into account by the low level solver to prefer less conflicts between fewer agents.
            // This only helps when this CBS is used as a low level solver, of course.
            if (this.totalConflictsWithExternalAgents < other.totalConflictsWithExternalAgents)
                return -1;
            if (this.totalConflictsWithExternalAgents > other.totalConflictsWithExternalAgents)
                return 1;

            if (this.totalExternalAgentsThatConflict < other.totalExternalAgentsThatConflict)
                return -1;
            if (this.totalExternalAgentsThatConflict > other.totalExternalAgentsThatConflict)
                return 1;
            
            // Prefer goal nodes. The elaborate form is to keep the comparison consistent. Without it goalA<goalB and also goalB<goalA.
            if (this.GoalTest() == true && other.GoalTest() == false)
                return -1;
            if (other.GoalTest() == true && this.GoalTest() == false)
                return 1;

            if (this.minOpsToSolve < other.minOpsToSolve)
                return -1;
            if (this.minOpsToSolve > other.minOpsToSolve)
                return 1;
            
            return 0;
        }

        /// <summary>
        /// Not used.
        /// </summary>
        /// <returns></returns>
        public CbsConstraint GetLastConstraint()
        {
            return this.constraint;
        }

        public HashSet<CbsConstraint> GetConstraints()
        {
            var constraints = new HashSet<CbsConstraint>();
            CbsNode current = this;
            CbsConstraint currentConstraint = null;

            while (current.depth > 0) // The root has no constraints
            {

                if (current.constraint != null && // Next check not enough if "surprise merges" happen (merges taken from adopted child)
                    current.prev.conflict != null && // Can only happen for temporary lookahead nodes the were created and then later the parent adopted a goal node
                    this.agentsGroupAssignment[current.prev.conflict.agentAIndex] !=
                    this.agentsGroupAssignment[current.prev.conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                    // agents that were later merged. They're irrelevant
                    // since merging fixes all conflicts between merged agents.
                    // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                    // Dereferencing current.prev is safe because current isn't the root.
                    // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                    
                    currentConstraint = current.constraint;
                    TimedMove     currentMove       = current.constraint.move;
                    CbsConstraint newConstraint = new CbsConstraint(currentConstraint.agentNum, currentMove.x, currentMove.y, currentMove.direction, currentMove.time);
                    constraints.Add(newConstraint);
                    
                current = current.prev;
            }
            return constraints;
        }

        private bool isLeftNode(CbsNode node)
        {
            if (node.prev == null || node.agentToReplan == node.prev.conflict.agentAIndex)
                return true;
            return false;
        }

        /// <summary>
        /// For printing
        /// </summary>
        /// <returns></returns>
        public List<CbsConstraint> GetConstraintsOrdered()
        {
            var constraints = new List<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0) // The root has no constraints
            {
                if (current.constraint != null && // Next check not enough if "surprise merges" happen (merges taken from adopted child)
                    current.prev.conflict != null && // Can only happen for temporary lookahead nodes the were created and then later the parent adopted a goal node
                    this.agentsGroupAssignment[current.prev.conflict.agentAIndex] !=
                    this.agentsGroupAssignment[current.prev.conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                    // agents that were later merged. They're irrelevant
                    // since merging fixes all conflicts between merged agents.
                    // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                    // Dereferencing current.prev is safe because current isn't the root.
                    // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                    constraints.Add(current.constraint);
                current = current.prev;
            }
            return constraints;
        }


        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        public Plan CalculateJointPlan()
        {
            return new Plan(allSingleAgentPlans);
        }

       

        /// <summary>
        /// Returns a list of indices of agents in the group
        /// </summary>
        /// <param name="agentIndex"></param>
        /// <returns></returns>
        public ISet<int> GetGroup(int agentIndex)
        {
            int groupNumber = this.agentsGroupAssignment[agentIndex];
            ISet<int> group = new SortedSet<int>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    group.Add(i);
            }
            return group;
        }

        /// <summary>
        /// Currently unused.
        /// </summary>
        /// <param name="groupNumber"></param>
        /// <returns></returns>
        public int GetGroupCost(int groupNumber)
        {
            int cost = 0;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    cost += this.allSingleAgentCosts[i];
            }
            return cost;
        }

        /// <summary>
        /// A bit cheaper than GetGroup(n).Count. Still O(n).
        /// </summary>
        /// <param name="groupNumber"></param>
        /// <returns></returns>
        public int GetGroupSize(int agentIndex)
        {
            int groupNumber = this.agentsGroupAssignment[agentIndex];
            int count = 0;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    count += 1;
            }
            return count;
        }

        /// <summary>
        /// In O(n)
        /// </summary>
        /// <returns></returns>
        public int[] GetGroupSizes()
        {
            int[] counts = new int[this.agentsGroupAssignment.Length];

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
                counts[this.agentsGroupAssignment[i]]++;

            int[] groupSizes = new int[this.agentsGroupAssignment.Length];

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
                groupSizes[i] = counts[this.agentsGroupAssignment[i]];
            
            return groupSizes;
        }

        /// <summary>
        /// In O(n)
        /// </summary>
        /// <returns></returns>
        public ISet<int>[] GetGroups()
        {
            Dictionary<int, ISet<int>> repsToGroups = new Dictionary<int, ISet<int>>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                int groupRep = this.agentsGroupAssignment[i];
                if (repsToGroups.ContainsKey(groupRep))
                    repsToGroups[groupRep].Add(i);
                else
                {
                    var newGroup = new HashSet<int>();
                    newGroup.Add(i);
                    repsToGroups[groupRep] = newGroup;

                }
            }

            ISet<int>[] res = new HashSet<int>[this.agentsGroupAssignment.Length];
            for (int i = 0; i < res.Length; i++)
			    res[i] = repsToGroups[this.agentsGroupAssignment[i]];

            return res;
        }

      

        public void PrintConflict()
        {
            if (conflict != null)
            {
                Debug.WriteLine("Conflict:");
                Debug.WriteLine("Agents:({0},{1})", conflict.agentAIndex, conflict.agentBIndex);
                Debug.WriteLine("Location:({0},{1})", conflict.agentAmove.x, conflict.agentAmove.y);
                Debug.WriteLine("Time:{0}", conflict.timeStep);
            }
            Debug.WriteLine("");
        }

       

        private bool isGoal = false;

        public bool GoalTest() {
            return isGoal;
        }

       
    }

    /// <summary>
    /// Because the default tuple comparison compares the first element only :(.
    /// </summary>
    public class AgentToCheckForCardinalConflicts : IBinaryHeapItem
    {
        //public bool hasMDD;
        //int conflictingAgentsWithMDD;
        int groupSize;
        int degree;
        int planCost;
        public int index;
 

        public int CompareTo(IBinaryHeapItem item)
        {
            AgentToCheckForCardinalConflicts other = (AgentToCheckForCardinalConflicts)item;


            if (this.groupSize < other.groupSize)
                return -1;
            else if (this.groupSize > other.groupSize)
                return 1;

            if (this.degree < other.degree)
                return -1;
            else if (this.degree > other.degree)
                return 1;

            if (this.planCost < other.planCost)
                return -1;
            else if (this.planCost > other.planCost)
                return 1;

            if (this.index < other.index)
                return -1;
            else if (this.index > other.index)
                return 1;
            else
                return 0;
        }

        int binaryHeapIndex;

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }
    }
}
