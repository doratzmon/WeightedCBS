using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Accord.Statistics;

namespace CPF_experiment
{

    /// <summary>
    /// This class is responsible for running the experiments.
    /// </summary>
    public class Run : IDisposable
    {

        /// <summary>
        /// Delimiter character used when writing the results of the runs to the output file.
        /// </summary>
        /// 
        public static bool toPrint = false;

        public static readonly string RESULTS_DELIMITER = ",";

        public static readonly int SUCCESS_CODE = 1;

        public static readonly int FAILURE_CODE = 0;

        public static readonly bool WRITE_LOG = false;
        /// <summary>
        /// Number of random steps performed when generating a new problem instance for choosing a start-goal pair.
        /// </summary>
        public static int RANDOM_WALK_STEPS = 100000;

        /// <summary>
        /// Indicates the starting time in ms for timing the different algorithms.
        /// </summary>
        private double startTime;

        public Plan plan;

        public  int solutionCost = -1;


        /// <summary>
        /// This hold an open stream to the results file.
        /// </summary>
        private TextWriter resultsWriter;

        /// <summary>
        /// EH: I introduced this variable so that debugging and experiments
        /// can have deterministic results.
        /// </summary>
        public static Random rand = new Random();

        /// <summary>
        /// Calls resultsWriter.Dispose()
        /// </summary>
        protected virtual void Dispose(bool dispose_managed)
        {
            if (dispose_managed)
            {
                if (this.resultsWriter != null)
                {
                    this.resultsWriter.Dispose();
                    this.resultsWriter = null;
                }
            }
        }

        public void Dispose() {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Open the results file for output. Currently the file is opened in append mode.
        /// </summary>
        /// <param name="fileName">The name of the results file</param>
        public void OpenResultsFile(string fileName)
        {
            this.resultsWriter = new StreamWriter(fileName, true); // 2nd argument indicates the "append" mode
        }

        /// <summary>
        /// Closes the results file.
        /// </summary>
        public void CloseResultsFile()
        {
            this.resultsWriter.Close();
        }

        /// <summary>
        /// all types of algorithms to be run
        /// </summary>
        List<ISolver> solvers;

        /// <summary>
        /// all types of heuristics used
        /// </summary>
        public List<HeuristicCalculator> heuristics; // FIXME: Make unpublic again later

        /// <summary>
        /// Counts the number of times each algorithm went out of time consecutively
        /// </summary>
        public int[] outOfTimeCounters;

        /// <summary>
        /// Construct with chosen algorithms.
        /// </summary>
        public Run()
        {
            this.watch = Stopwatch.StartNew();

            // Preparing the heuristics:
            heuristics = new List<HeuristicCalculator>();
            var sic = new SumIndividualCosts();
            heuristics.Add(sic);

            var astar = new ClassicAStar(sic);      
            ISolver cbs = new CBS(astar, astar);   // CBS

            // Preparing the solvers:
            solvers = new List<ISolver>();

            solvers.Add(cbs);

            outOfTimeCounters = new int[solvers.Count];
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        /// <summary>
        /// Generates a problem instance, including a board, start and goal locations of desired number of agents
        /// and desired precentage of obstacles
        /// TODO: Refactor to use operators.
        /// </summary>
        /// <param name="gridSize"></param>
        /// <param name="agentsNum"></param>
        /// <param name="obstaclesNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateProblemInstance(int gridSize, int agentsNum, int obstaclesNum)
        {
            m_mapFileName = "GRID";
            m_agentNum = agentsNum;
            /**
             * Randomization based on timer is disabled for purposes of getting
             * reproducible experiments.
             */
            //Random rand = new Random();

            if (agentsNum + obstaclesNum + 1 > gridSize * gridSize)
                throw new Exception("Not enough room for " + agentsNum + ", " + obstaclesNum + " and one empty space in a " + gridSize + "x" + gridSize + "map.");

            int x;
            int y;
            Agent[] aGoals = new Agent[agentsNum];
            AgentState[] aStart = new AgentState[agentsNum];
            bool[][] grid = new bool[gridSize][];
            bool[][] goals = new bool[gridSize][];

            // Generate a random grid
            for (int i = 0; i < gridSize; i++)
            {
                grid[i] = new bool[gridSize];
                goals[i] = new bool[gridSize];
            }
            for (int i = 0; i < obstaclesNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (grid[x][y]) // Already an obstacle
                    i--;
                grid[x][y] = true;
            }

            // Choose random goal locations
            for (int i = 0; i < agentsNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (goals[x][y] || grid[x][y])
                    i--;
                else
                {
                    goals[x][y] = true;
                    aGoals[i] = new Agent(x, y, i);
                }
            }

            // Select random start/goal locations for every agent by performing a random walk
            for (int i = 0; i < agentsNum; i++)
            {
                aStart[i] = new AgentState(aGoals[i].Goal.x, aGoals[i].Goal.y, aGoals[i]);
            }

            // Initialized here only for the IsValid() call. TODO: Think how this can be sidestepped elegantly.
            ProblemInstance problem = new ProblemInstance();
            problem.Init(aStart, grid);
            
            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = false; // We're going to move the goal somewhere else
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        aStart[i].lastMove.Update(op);
                        if (problem.IsValid(aStart[i].lastMove) &&
                            !goals[aStart[i].lastMove.x][aStart[i].lastMove.y]) // this spot isn't another agent's goal
                            break;
                        else
                            aStart[i].lastMove.setOppositeMove(); // Rollback
                    }
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in aStart) 
            {
                agentStart.lastMove.time = 0;
            }

            // TODO: There is some repetition here of previous instantiation of ProblemInstance. Think how to elegantly bypass this.
            problem = new ProblemInstance();
            problem.Init(aStart, grid);
            return problem;            
        }

        public string m_mapFileName = "";
        public int    m_agentNum    = 0;

        /// <summary>
        /// Generates a problem instance based on a DAO map file.
        /// TODO: Fix code dup with GenerateProblemInstance and Import later.
        /// </summary>
        /// <param name="agentsNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateDragonAgeProblemInstance(string mapFileName, int agentsNum)
        {
            /**
             * Randomization based on timer is disabled for purposes of getting
             * reproducible experiments.
             */
            //Random rand = new Random();

            m_mapFileName       = mapFileName;
            m_agentNum          = agentsNum; 
            TextReader input    = new StreamReader(mapFileName);
            string[] lineParts;
            string line;

            line = input.ReadLine();
            Debug.Assert(line.StartsWith("type octile"));

            // Read grid dimensions
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Debug.Assert(lineParts[0].StartsWith("height"));
            int maxX = int.Parse(lineParts[1]);
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Debug.Assert(lineParts[0].StartsWith("width"));
            int maxY = int.Parse(lineParts[1]);
            line = input.ReadLine();
            Debug.Assert(line.StartsWith("map"));
            bool[][] grid = new bool[maxX][];
            char cell;
            for (int i = 0; i < maxX; i++)
            {
                grid[i] = new bool[maxY];
                line = input.ReadLine();
                for (int j = 0; j < maxY; j++)
                {
                    cell = line.ElementAt(j);
                    if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                        grid[i][j] = true;
                    else
                        grid[i][j] = false;
                }
            }

            int x;
            int y;
            Agent[] agentGoals = new Agent[agentsNum];
            AgentState[] agentStates = new AgentState[agentsNum];
            bool[][] goals = new bool[maxX][];

            for (int i = 0; i < maxX; i++)
                goals[i] = new bool[maxY];

            // Choose random valid unclaimed goal locations
            for (int i = 0; i < agentsNum; i++)
            {
                x = rand.Next(maxX);
                y = rand.Next(maxY);
                if (goals[x][y] || grid[x][y])
                    i--;
                else
                {
                    goals[x][y] = true;
                    agentGoals[i] = new Agent(x, y, i);
                }
            }

            // Select random start/goal locations for every agent by performing a random walk
            for (int i = 0; i < agentsNum; i++)
            {
                agentStates[i] = new AgentState(agentGoals[i].Goal.x, agentGoals[i].Goal.y, agentGoals[i]);
            }

            ProblemInstance problem = new ProblemInstance();
            problem.parameters[ProblemInstance.GRID_NAME_KEY] = Path.GetFileNameWithoutExtension(mapFileName);
            problem.Init(agentStates, grid);

            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = false; // We're going to move the goal somewhere else
                    // Move in a random legal direction:
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        agentStates[i].lastMove.Update(op);
                        if (problem.IsValid(agentStates[i].lastMove) &&
                            !goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y]) // This spot isn't another agent's goal
                            break;
                        else
                            agentStates[i].lastMove.setOppositeMove(); // Rollback
                    }
                    goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in agentStates)
                agentStart.lastMove.time = 0;

            return problem;
        }

        
        public static double    planningTime;
        public static int       instanceId = 0; 

        /// <summary>
        /// Solve given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to solve</param>
        public void SolveGivenProblem(ProblemInstance instance)
        {
            // add for generator
            // Preparing a list of agent indices (not agent nums) for the heuristics' Init() method
            List<uint> agentList = Enumerable.Range(0, instance.m_vAgents.Length).Select<int, uint>(x=> (uint)x).ToList<uint>(); // FIXME: Must the heuristics really receive a list of uints?
            
            // Solve using the different algorithms
            Debug.WriteLine("Solving " + instance);

            // Initializing all heuristics, whereever they're used
            for (int i = 0; i < heuristics.Count; i++)
                heuristics[i].init(instance, agentList);

            solutionCost = -1;
            int firstSolverToSolveIndex = -1;
            for (int i = 0; i < solvers.Count; i++)
            {
                if (outOfTimeCounters[i] < Constants.MAX_FAIL_COUNT) // After "MAX_FAIL_COUNT" consecutive failures of a given algorithm we stop running it.
                                                                    // Assuming problem difficulties are non-decreasing, if it consistently failed on several problems it won't suddenly succeed in solving the next problem.
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();


                    int solverSolutionCost;
                    Stopwatch planningStopwatch = new Stopwatch(); ;
      
                    planningStopwatch.Start();
                    this.run(solvers[i], instance);
                    solverSolutionCost = solvers[i].GetSolutionCost();
         
                    if (solverSolutionCost < 0)
                    {
                        Console.WriteLine("TIMEOUT!");
                        solverSolutionCost = -1;
                        solutionCost = -1;
                        planningStopwatch.Stop();
                        continue;
                    }

                    if (solverSolutionCost >= 0) // Solved successfully
                    {
                        outOfTimeCounters[i] = 0;
                        firstSolverToSolveIndex = i;
                        solutionCost = solverSolutionCost;
                        Console.WriteLine("+SUCCESS+ (:");

                        solvers[i].GetPlan().PrintPlan();

                    }
                    else
                    {
                        outOfTimeCounters[i]++;
                        Console.WriteLine("-FAILURE- ):");
                    }

                    planningStopwatch.Stop();
                    TimedMove[] lastMove = new TimedMove[instance.m_vAgents.Length];
                    for (int ass = 0; ass < instance.m_vAgents.Length; ass++)
                    {
                        lastMove[ass] = new TimedMove(instance.m_vAgents[ass].lastMove, 0);

                    }
                       
                }
                else if(toPrint)
                    PrintNullStatistics(solvers[i]);
                
                Console.WriteLine();
            }
        }

       

        public double elapsedTime;

        /// <summary>
        /// Solve a given instance with the given solver
        /// </summary>
        /// <param name="solver">The solver</param>
        /// <param name="instance">The problem instance that will be solved</param>
        private void run(ISolver solver, ProblemInstance instance)
        {
            // Run the algorithm
            bool solved;
            Console.WriteLine("-----------------" + solver + "-----------------");
            this.startTime = this.ElapsedMillisecondsTotal();
            solver.Setup(instance, this);
            solved = solver.Solve();
            elapsedTime = this.ElapsedMilliseconds();
            if (solved)
            {
                Console.WriteLine("Total cost: {0}", solver.GetSolutionCost());
                Console.WriteLine("Solution depth: {0}", solver.GetSolutionDepth());
            }
            else
            {
                Console.WriteLine("Failed to solve");
                Console.WriteLine("Solution depth lower bound: {0}", solver.GetSolutionDepth());
            }
            Console.WriteLine();
            Console.WriteLine("Time In milliseconds: {0}", elapsedTime);
           // Console.WriteLine("Total Unique/Full Expanded Nodes: {0}", solver.GetNodesPassedPruningCounter());
            if(toPrint)
                this.PrintStatistics(instance, solver, elapsedTime);
            // Solver clears itself when it finishes the search.
            solver.ClearStatistics();
        }

        /// <summary>
        /// Print the header of the results file
        /// </summary>
        public void PrintResultsFileHeader()
        {
            this.resultsWriter.Write("Grid Name");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Rows");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Columns");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Agents");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Obstacles");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Instance Id");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);

            for (int i = 0; i < solvers.Count; i++)
            {
                var solver = solvers[i];
                this.resultsWriter.Write(solver + " Success");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Runtime");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Solution Cost");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                solver.OutputStatisticsHeader(this.resultsWriter);
                this.resultsWriter.Write(solver + " Max Group");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Solution Depth");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);

            }
            this.ContinueToNextLine();
        }

        /// <summary>
        /// Print the solver statistics to the results file.
        /// </summary>
        /// <param name="instance">The problem instance that was solved. Not used!</param>
        /// <param name="solver">The solver that solved the problem instance</param>
        /// <param name="runtimeInMillis">The time it took the given solver to solve the given instance</param>
        private void PrintStatistics(ProblemInstance instance, ISolver solver, double runtimeInMillis)
        {
            // Success col:
            if (solver.GetSolutionCost() < 0)
                this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(Run.SUCCESS_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(runtimeInMillis + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write(solver.GetSolutionCost() + RESULTS_DELIMITER);
            // Algorithm specific cols:
            solver.OutputStatistics(this.resultsWriter);
            // Max Group col:
            this.resultsWriter.Write(solver.GetMaxGroupSize() + RESULTS_DELIMITER);
            // Solution Depth col:
            this.resultsWriter.Write(solver.GetSolutionDepth() + RESULTS_DELIMITER);
            //this.resultsWriter.Flush();
        }

        private void PrintProblemStatistics(ProblemInstance instance)
        {
            // Grid Name col:
            if (instance.parameters.ContainsKey(ProblemInstance.GRID_NAME_KEY))
                this.resultsWriter.Write(instance.parameters[ProblemInstance.GRID_NAME_KEY] + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(RESULTS_DELIMITER);
            // Grid Rows col:
            this.resultsWriter.Write(instance.m_vGrid.Length + RESULTS_DELIMITER);
            // Grid Columns col:
            this.resultsWriter.Write(instance.m_vGrid[0].Length + RESULTS_DELIMITER);
            // Num Of Agents col:
            this.resultsWriter.Write(instance.m_vAgents.Length + RESULTS_DELIMITER);
            // Num Of Obstacles col:
            this.resultsWriter.Write(instance.m_nObstacles + RESULTS_DELIMITER);
            // Instance Id col:
            this.resultsWriter.Write(instance.instanceId + RESULTS_DELIMITER);
        }

        private void ContinueToNextLine()
        {
            this.resultsWriter.WriteLine();
            this.resultsWriter.Flush();
        }

        private void PrintNullStatistics(ISolver solver)
        {
            // Success col:
            this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(Constants.MAX_TIME + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Algorithm specific cols:
            for (int i = 0; i < solver.NumStatsColumns; ++i)
                this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Max Group col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Solution Depth col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
        }
        
        public void ResetOutOfTimeCounters()
        {
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        private Stopwatch watch;
        private double ElapsedMillisecondsTotal()
        {
            return this.watch.Elapsed.TotalMilliseconds;
        }

        public double ElapsedMilliseconds()
        {
            return ElapsedMillisecondsTotal() - this.startTime;
        }

        public void StartOracle()
        {
            this.watch.Stop();
            // NOTE: This allows the algorithm with the oracle solve harder problems without timing out, getting
            // a higher average timeout than running without the oracle, which isn't what we want.
            // We need to start another counter when the oracle runs and when the run successfully finishes
            // substract its count
        }

        public void StopOracle()
        {
            this.watch.Start();
        }

        public static double TIMEOUT    = 300000;
        //public static double TIMEOUT = double.MaxValue;
        public static double replanTime = 0;
        public static Stopwatch sw      = new Stopwatch();
        public static int replanCounter = 0;
        

    }
}
