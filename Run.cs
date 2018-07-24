﻿using System;
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
        ////////debug
        // public static TextWriter resultsWriterdd;
        /////////////

        /// <summary>
        /// Delimiter character used when writing the results of the runs to the output file.
        /// </summary>
        /// 
        public static bool toPrint = false;

        public static int upperB = 2;

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
        //Reasonable 
        public enum ExecutePolicy { Lazy, Stressful, Reasonable, Stressful_Repair , Reasonable_Repair, MCP , pRobustCheck};
        public enum ConstraintPolicy { Single, Range, DoubleRange };

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
            var astar = new ClassicAStar(sic, false, false, 0);             //withNoBias
            var astarWithBias1 = new ClassicAStar(sic, false, false, 1);    //withBias
            var astarWithBias2 = new ClassicAStar(sic, false, false, 2);    //withBias2
            var astarWithBias3 = new ClassicAStar(sic, false, false, 3);    //withBias3
            var astarWithBias4 = new ClassicAStar(sic, false, false, 4);    //withBias 4
            var astarWithBias5 = new ClassicAStar(sic, false, false, 5);    //withBias 5
            var astarWithBias6 = new ClassicAStar(sic, false, false, 6);    //withBias 6
            var astarWithBias7 = new ClassicAStar(sic, false, false, 7);    //withBias 7
            var astarWithDynamicBias = new ClassicAStar(sic, false, false, -1);   //withBias -1
            var cbs = new CBS_LocalConflicts(astar, astar, -1);
            var astar_with_od = new AStarWithOD(sic);
            var epea = new AStarWithPartialExpansion(sic);
            var macbsLocal5Epea = new CBS_LocalConflicts(astar, epea, 5);
            var macbsLocal50Epea = new CBS_LocalConflicts(astar, epea, 50);
            //var cbsHeuristicNoSolve1 = new CbsHeuristic(cbs, this, false, 1);
            //var cbsHeuristicNoSolve2 = new CbsHeuristic(cbs, this, false, 2);
            //var cbsHeuristicNoSolve3 = new CbsHeuristic(cbs, this, false, 3);
            //var cbsHeuristicNoSolve4 = new CbsHeuristic(cbs, this, false, 4);
            //var cbsHeuristicNoSolve5 = new CbsHeuristic(cbs, this, false, 5);
            //var cbsHeuristicNoSolve6 = new CbsHeuristic(cbs, this, false, 6);
            //var cbsHeuristicSolve1 = new CbsHeuristic(cbs, this, true, 1);
            //var cbsHeuristicSolve2 = new CbsHeuristic(cbs, this, true, 2);
            //var cbsHeuristicSolve3 = new CbsHeuristic(cbs, this, true, 3);
            //var cbsHeuristicSolve4 = new CbsHeuristic(cbs, this, true, 4);
            //var cbsHeuristicSolve5 = new CbsHeuristic(cbs, this, true, 5);
            //var cbsHeuristicSolve6 = new CbsHeuristic(cbs, this, true, 6);
            //var sicOrCbsh6 = new RandomChoiceOfHeuristic(cbsHeuristicSolve6, sic, 1.0 / 5);
            
            //var dynamicLazyCbsh = new DyanamicLazyCbsh(cbs, this, true);
            //heuristics.Add(dynamicLazyCbsh);

            //var dynamicLazyMacbsLocal5EpeaH = new DyanamicLazyCbsh(macbsLocal5Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal5EpeaH);

            //var dynamicLazyMacbsLocal50EpeaH = new DyanamicLazyCbsh(macbsLocal50Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal50EpeaH);

            //var dynamicLazyMacbsLocal5EpeaHForOracleMustBeLast = new DyanamicLazyCbsh(macbsLocal5Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal5EpeaHForOracleMustBeLast);

            // Preparing the solvers:
            solvers = new List<ISolver>();
            /*
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, false, false, true, false, false, false)); // CBS/EPEA*
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, true, false, true, false, false, true, 1)); // CBS/EPEA* with first-fit adoption 1 expansions max
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, true, false, true, false, false, true, 256)); // CBS/EPEA* with first-fit adoption 256 expansions max
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, true, false, true, false, false, true, int.MaxValue)); // CBS/EPEA* with first-fit adoption infinity expansions max
            */
            // B is actually set according to the map in a hack elsewhere

            //IJCAI:

           //  solvers.Add(new CBS_LocalConflicts(astar, epea, 0, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //              CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // CBS/EPEA*

           // solvers.Add(new IndependenceDetection(astar, new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
           //             CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD, false, false), sic)); // CBS/EPEA*
           // solvers.Add(new IndependenceDetection(astar, new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
           //             CBS_LocalConflicts.ConflictChoice._CONFLICTING, false, false), sic)); // CBS/EPEA*
        //  solvers.Add(new IndependenceDetection(astar, new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
        //              CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false), sic)); // CBS/EPEA*
           // solvers.Add(new IndependenceDetection(astar, new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //            CBS_LocalConflicts.ConflictChoice.FIRST, false, false), sic)); // CBS/EPEA*

            //soldier: solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //soldier:     CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // CBS/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD, false, false)); // CBS/EPEA* Cardinal not using MDDs

            //soldier: 
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 2, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(2)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(4)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 8, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(8)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 16, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(16)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 32, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(32)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 64, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(32)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 50, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(50)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 128, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(128)/EPEA*
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 256, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false)); // MA-CBS(256)/EPEA*


            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(5)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 10, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(10)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 25, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(25)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 100, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(100)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 150, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(100)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 200, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(100)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 300, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(100)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //    astar, epea, 500, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, false), sic)); // MA-CBS(100)/EPEA* + ID.

            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(
            //        astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //        CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, true),
            //    sic)); // MA-CBS(B)/EPEA* + ID + restart

            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.MOST_CONFLICTING, false, false, int.MaxValue, false)); // MA-CBS(5)/EPEA*
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
             //   CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
           // solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
             //   CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
             //   CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
             //   CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
             //   CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
           // solvers.Add(new CBS_LocalConflicts(astar, epea, 2, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
        //    solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD, false,
      //          CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
     //       solvers.Add(epea); // EPEA*
            //        solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //            CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false));
            //        solvers.Add(new CBS_LocalConflicts(astar, epea, 25, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //             CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false));
            //         solvers.Add(new CBS_LocalConflicts(astar, epea, 100, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //             CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false));
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD, false, false)); // MA-CBS(5)/EPEA* Cardinal not using MDDs

            //soldier: solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //    CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, true)); // MA-CBS(5)/EPEA* + restart
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 64, false, CBS_LocalConflicts.BypassStrategy.NONE, false,
            //            CBS_LocalConflicts.ConflictChoice.FIRST, false, false, int.MaxValue, true)); // MA-CBS(64)/EPEA* + restart

            //soldier: solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //soldier:     false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1)); // CBS/EPEA* + BP1

            /*
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0)); // CBS/EPEA* + CARDINAL + BP1
            
            solvers.Add(new CBS_GlobalConflicts(astarWithBias1, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 1)); // CBS/EPEA* + CARDINAL + BP1
            */
            /*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, false)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, true)); // CBS/EPEA* + CARDINAL + BP1
             */

            /*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, true)); // CBS/EPEA* + CARDINAL + BP1
            
            solvers.Add(new CBS_GlobalConflicts(astarWithBias4, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 4, true)); // CBS/EPEA* + CARDINAL + BP1
            */

            //solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //            false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

           /*
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.Single)); // CBS/EPEA* + CARDINAL + BP1
            
           solvers.Add(new CBS_GlobalConflicts(astarWithBias1, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                       false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 1, ConstraintPolicy.Single)); // CBS/EPEA* + CARDINAL + BP1

           solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                       false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, ConstraintPolicy.Single)); // CBS/EPEA* + CARDINAL + BP1
           *//*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, ConstraintPolicy.Single)); // CBS/EPEA* + CARDINAL + BP1
            */
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.MOST_CONFLICTING, false, false, 1)); // MA-CBS(B)/EPEA* + BP1

             //k-robust 0,1,2
             
             
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.NONE,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
         /*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias1, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
            
            solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias4, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 4, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias5, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 5, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias6, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 6, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias7, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 7, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
*/
            //p-robust

            // solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //           false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            /*
               solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                         false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
                         */

            /*
            solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                         false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                         false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                         false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
              solvers.Add(new CBS_GlobalConflicts(astarWithDynamicBias, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, -1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
          */


            //solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //          false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
            /*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
            */
            /*
           solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                       false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.DoubleRange)); // CBS/EPEA* + CARDINAL + BP1

         solvers.Add(new CBS_GlobalConflicts(astarWithBias1, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                     false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 1, ConstraintPolicy.DoubleRange)); // CBS/EPEA* + CARDINAL + BP1

         solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                     false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, ConstraintPolicy.DoubleRange)); // CBS/EPEA* + CARDINAL + BP1
        *//*
             solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                         false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, ConstraintPolicy.DoubleRange)); // CBS/EPEA* + CARDINAL + BP1
             */
          // (/
          //solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
          //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, false)); // CBS + CARDINAL (lookahead) + BP1


            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, -1, false,
            //    CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, false), sic)); // CBS + CARDINAL + BP1 + ID

            //solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD, false, false, 1)); // CBS/EPEA* Cardinal not using MDDs + BP1

            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.MOST_CONFLICTING, false, false, 1)); // MA-CBS(B)/EPEA* + BP1
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1)); // MA-CBS(B)/EPEA* Cardinal using MDDs + BP1
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD, false, false, 1)); // MA-CBS(B)/EPEA* Cardinal not using MDDs + BP1

            //soldier: 
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 2, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(5)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 4, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(10)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(5)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 8, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(25)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 16, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 32, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 50, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(50)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 64, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 128, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 256, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 512, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 1024, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart


            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(5)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 10, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(10)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 25, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(25)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 100, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 150, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 200, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 300, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 500, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true), sic)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID


            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    false, CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD, false, false, 1, true)); // MA-CBS(B)/EPEA* Cardinal using MDDs + BP1 + restart

            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 5, false, true, false, CBS_LocalConflicts.ConflictChoice.MOST_CONFLICTING, false, false, true, int.MaxValue)); // MA-CBS/EPEA* with first-fit adoption infinity expansions max
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 1), sic)); // MA-CBS/EPEA* with first-fit adoption 1 expansions max + ID
            //solvers.Add(new CBS_GlobalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 256)); // MA-CBS/EPEA* with first-fit adoption 256 expansions max
            //solvers.Add(new IndependenceDetection(astar, new CBS_GlobalConflicts(astar, epea, 5, false, true, false, true, false, false, true, int.MaxValue), sic)); // MA-CBS/EPEA* with first-fit adoption infinity expansions max + ID

            /* soldier:*/ //solvers.Add(epea); // EPEA*
                          //soldier: solvers.Add(new CostTreeSearchSolverOldMatching(3)); // ICTS
                          //        solvers.Add(new IndependenceDetection(astar, epea, sic)); // EPEA* + ID
                          //solvers.Add(new IndependenceDetection(astar, new CostTreeSearchSolverOldMatching(3), sic)); // ICTS + ID

            /*
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, false, false, true, false, false, false)); // MACBS(5)/EPEA* - Works and is very fast so is a good choice for cost validation
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 1)); // MACBS(5)/EPEA* + adoption immediately
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, int.MaxValue)); // MACBS(5)/EPEA* + adoption immediately infinite expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 2)); // MACBS(5)/EPEA* + adoption immediately 2 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 4)); // MACBS(5)/EPEA* + adoption immediately 4 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 8)); // MACBS(5)/EPEA* + adoption immediately 8 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 16)); // MACBS(5)/EPEA* + adoption immediately 16 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 32)); // MACBS(5)/EPEA* + adoption immediately 32 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 64)); // MACBS(5)/EPEA* + adoption immediately 64 expansions
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 128)); // MACBS(5)/EPEA* + adoption immediately
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, 256)); // MACBS(5)/EPEA* + adoption immediately
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, true, int.MaxValue)); // MACBS(5)/EPEA* + adoption immediately infinite expansions
            */

            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false)); // MACBS(5)/EPEA* + infinite lookahead adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false, 4)); // MACBS(5)/EPEA* + 4 lookahead adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false, 8)); // MACBS(5)/EPEA* + 8 lookahead adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false, 16)); // MACBS(5)/EPEA* + 16 lookahead adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false, 32)); // MACBS(5)/EPEA* + 32 lookahead adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, true, false, true, false, false, false, 64)); // MACBS(5)/EPEA* + 64 lookahead adoption

            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, false, false, truesllslss, false, false, false)); // CBS/EPEA*
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, true, false, true, false, false)); // CBS+adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, true, false, false, true, false, false)); // CBS+shuffle
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, true, false, true, false, true)); // CBS+MDD+adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, true, false, false, true, false, true)); // CBS+MDD+shuffle
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, false, false, true, true)); // Simply tie break for more conflicts
            //solvers.Add(new CBS_LocalConflicts(astar, epea, 5, false, false, false, false)); // MACBS(5) Find first conflict
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, false, false, false, false, false)); // CBS Find first conflict
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1)); // Works and is very fast so is a good choice for cost validation
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, true, false, false)); // shuffle
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, true, false)); // Adoption
            //solvers.Add(new CBS_LocalConflicts(astar, epea, -1, false, false, true)); // Malte
            /*
            //solvers.Add(new CBS_LocalConflicts(epea, epea, -1));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, -1)); // Should be identical since no merging is done.
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 0));
            solvers.Add(new CBS_LocalConflicts(astar, epea, 0));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 0));
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 1));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 1));
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 5));
            solvers.Add(new CBS_LocalConflicts(astar, epea, 5));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 5));
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 10));
            solvers.Add(new CBS_LocalConflicts(astar, epea, 10));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 10));
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 100));
            solvers.Add(new CBS_LocalConflicts(astar, epea, 100));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 100));
            //solvers.Add(new CBS_LocalConflicts(epea, epea, 500));
            //solvers.Add(new CBS_GlobalConflicts(epea, epea, 500));
            
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, -1));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, -1)); // Should be identical since no merging is done.
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 0));
            solvers.Add(new CBS_LocalConflicts(astar, astar_with_od, 0));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 0));
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 1));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 1));
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 5));
            solvers.Add(new CBS_LocalConflicts(astar, astar_with_od, 5));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 5));
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 10));
            solvers.Add(new CBS_LocalConflicts(astar, astar_with_od, 10));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 10));
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 100));
            solvers.Add(new CBS_LocalConflicts(astar, astar_with_od, 100));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 100));
            //solvers.Add(new CBS_LocalConflicts(astar_with_od, astar_with_od, 500));
            //solvers.Add(new CBS_GlobalConflicts(astar_with_od, astar_with_od, 500));
            */
            //solvers.Add(new ClassicAStar(sic, true)); // Works
            //solvers.Add(new ClassicAStar(sic, true, false)); // rM*! Works
            //solvers.Add(new ClassicAStar(sic, true, true)); // rM* shuffle! Works
            //solvers.Add(new ClassicAStar(cbsHeuristic)); // Works
            //solvers.Add(new AStarWithOD(sic));  // Works
            //solvers.Add(new AStarWithOD(sic, true, false)); // rM*+OD!
            //solvers.Add(new AStarWithOD(sic, true, true)); // rM*+OD shuffle!
            //solvers.Add(new AStarWithPartialExpansionBasic(sic)); // Works
            //solvers.Add(new AStarWithPartialExpansionBasic(cbsHeuristic));
            //soldier: solvers.Add(new AStarWithPartialExpansion(sic)); // Works.
            //solvers.Add(new AStarWithPartialExpansion(sic, true, false)); // EPErM*
            //solvers.Add(new AStarWithPartialExpansion(sic, true, true)); // EPErM* shuffle
            //soldier: solvers.Add(new CBS_LocalConflicts(astar, epea, 0)); // EPEA*+(S)ID

            //solvers.Add(new ClassicAStar(cbsHeuristicSolve1));
            //solvers.Add(new ClassicAStar(cbsHeuristicSolve2));
            //solvers.Add(new ClassicAStar(cbsHeuristicSolve3));
            //solvers.Add(new ClassicAStar(cbsHeuristicSolve4));
            //solvers.Add(new ClassicAStar(cbsHeuristicSolve5));
            //solvers.Add(new ClassicAStar(cbsHeuristicSolve6));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve1));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve2));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve3));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve4));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve5));
            //solvers.Add(new ClassicAStar(cbsHeuristicNoSolve6));
            //solvers.Add(new ClassicAStar(sicOrCbsh6));

            //solvers.Add(new AStarWithOD(cbsHeuristicSolve1));
            //solvers.Add(new AStarWithOD(cbsHeuristicSolve2));
            //solvers.Add(new AStarWithOD(cbsHeuristicSolve3));
            //solvers.Add(new AStarWithOD(cbsHeuristicSolve4));
            //solvers.Add(new AStarWithOD(cbsHeuristicSolve5));
            //solvers.Add(new AStarWithOD(cbsHeuristicSolve6));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve1));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve2));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve3));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve4));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve5));
            //solvers.Add(new AStarWithOD(cbsHeuristicNoSolve6));
            //solvers.Add(new AStarWithOD(sicOrCbsh6));

            ClassicAStar solver;
            // dynamic not rational lazy A*+OD/CBS/A*/SIC:
            //solver = new AStarWithOD(sic);
            //var dynamicLazyOpenList1 = new DynamicLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicLazyOpenList1;
            //solvers.Add(solver);

            // dynamic rational lazy A*+OD/CBS/A*/SIC:
            //solver = new AStarWithOD(sic);
            //var dynamicRationalLazyOpenList1 = new DynamicRationalLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicRationalLazyOpenList1;
            //solvers.Add(solver);

            // dynamic rational lazy MA-CBS-local-5/A*+OD/MA-CBS-local-5/EPEA*/SIC:
            //solver = new AStarWithOD(sic);
            //var dynamicRationalLazyOpenList3 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            //solver.openList = dynamicRationalLazyOpenList3;
            //solvers.Add(new CBS_LocalConflicts(astar, solver, 5));

            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve1));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve2));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve3));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve4));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve5));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicSolve6));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve1));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve2));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve3));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve4));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve5));
            //solvers.Add(new AStarWithPartialExpansion(cbsHeuristicNoSolve6));
            //solvers.Add(new AStarWithPartialExpansion(sicOrCbsh6));

            // dynamic not rational lazy EPEA*/CBS/A*/SIC:
            //solver = new AStarWithPartialExpansion(sic);
            //var dynamicLazyOpenList2 = new DynamicLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicLazyOpenList2;
            //solvers.Add(solver);

            // dynamic rational lazy EPEA*/CBS/A*/SIC:
            //solver = new AStarWithPartialExpansion(sic);
            //var dynamicRationalLazyOpenList2 = new DynamicRationalLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicRationalLazyOpenList2;
            //solvers.Add(solver);

            /*
             * soldiers:
            // MA-CBS-local-5 / dynamic rational lazy EPEA* / MA-CBS-local-50 / EPEA* / SIC:
            solver = new AStarWithPartialExpansion(sic);
            var dynamicRationalLazyOpenList4 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal50EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList4;
            solvers.Add(new CBS_LocalConflicts(astar, solver, 5));

            // dynamic rational lazy EPEA* / MA-CBS-local-5 / EPEA* / SIC + (S)ID:
            solver = new AStarWithPartialExpansion(sic);
            var dynamicRationalLazyOpenList6 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList6;
            solvers.Add(new CBS_LocalConflicts(astar, solver, 0));
             */
            
            /*
            //soldier: but can't handle 50 agents
            // dynamic rational lazy EPEA* / MA-CBS-local-5 / EPEA* / SIC:
            solver = new AStarWithPartialExpansion(sic);
            var dynamicRationalLazyOpenList8 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList8;
            solvers.Add(solver);
             */

            //solvers.Add(new CostTreeSearchSolverNoPruning());
            //solvers.Add(new CostTreeSearchSolverKMatch(2));
            //solvers.Add(new CostTreeSearchSolverOldMatching(2));
            //solvers.Add(new CostTreeSearchSolverRepeatedMatch(2));
            //solvers.Add(new CostTreeSearchSolverKMatch(3));
            //!@# USE ME solvers.Add(new CostTreeSearchSolverOldMatching(3)); // Use this parameter. Best according to paper. 3RE
            //solvers.Add(new CostTreeSearchSolverRepeatedMatch(3));

            //solvers.Add(new CostTreeSearchNoPruning());
            //solvers.Add(new CostTreeSearchKMatch(2));
            //solvers.Add(new CostTreeSearchOldMatching(2));
            //solvers.Add(new CostTreeSearchRepatedMatch(2));
            //solvers.Add(new CostTreeSearchKMatch(3));
            //solvers.Add(new CostTreeSearchOldMatching(3));
            //solvers.Add(new CostTreeSearchRepatedMatch(3));

            //solvers.Add(new IndependenceDetection(new AStarWithPartialExpansion()));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 1, 1)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 5, 5)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 10, 10)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 100, 100)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 500, 500)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar())));

            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar(), 1, 1)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar(), 5, 5)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar(), 10, 10)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar(), 100, 100)));
            //solvers.Add(new IndependenceDetection(new CBS_GlobalConflicts(new ClassicAStar(), 500, 500)));

            //solvers.Add(new IndependenceDetection(new AStarWithPartialExpansionBasic()));
            //solvers.Add(new IndependenceDetection(new AStarWithPartialExpansion()));
            //solvers.Add(new IndependenceDetection(new ClassicAStar()));
            //solvers.Add(new IndependenceDetection());

            //solvers.Add(new CBS_IDA(new ClassicAStar())); // Don't run! Uses must conds

            //solvers.Add(new CBS_GlobalConflicts(new ClassicAStar())); // Works

            //solvers.Add(new CBS_NoDD(new ClassicAStar()));
            //solvers.Add(new CBS_NoDDb3(new ClassicAStar()));
            //solvers.Add(new CBS_GlobalConflicts(new ClassicAStar(), 1, 1)); // Run this!

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
        public void SolveGivenProblem(ProblemInstance instance, bool toExecute = false)
        {
            if (toExecute)
            {
                instanceId += 1;
            }
            
            
            //return; // add for generator
            // Preparing a list of agent indices (not agent nums) for the heuristics' Init() method
            List<uint> agentList = Enumerable.Range(0, instance.m_vAgents.Length).Select<int, uint>(x=> (uint)x).ToList<uint>(); // FIXME: Must the heuristics really receive a list of uints?
            
            // Solve using the different algorithms
            Debug.WriteLine("Solving " + instance);

            //this.PrintProblemStatistics(instance);

            //double cr0 = instance.getConflictRation(0);
            //double cr1 = instance.getConflictRation(1);

            //Debug.WriteLine("Conflict ratio (first order): " + cr0);
            //Debug.WriteLine("Conflict ratio (second order): " + cr1);
            //this.resultsWriter.Write(cr0 + RESULTS_DELIMITER);
            //this.resultsWriter.Write(cr1 + RESULTS_DELIMITER);

            // Initializing all heuristics, whereever they're used
            for (int i = 0; i < heuristics.Count; i++)
                heuristics[i].init(instance, agentList);

            solutionCost = -1;
            int firstSolverToSolveIndex = -1;
            double[] PRobustBound = { /*0.4,*/ /*0.45,*/ /*0.5,*//* 0.55,*/ /*0.6,*//* 0.65,*/ /*0.7, *//*0.75,*/ /*0.8,*//* 0.85,*/ 0.9/*, 0.95*/};
            for (int i = 0; i < solvers.Count; i++)
            {
                //CbsNode.PRobustBound = 0.4;
                //CbsNode.PRobustBound = PRobustBound[i];
                if (outOfTimeCounters[i] < Constants.MAX_FAIL_COUNT) // After "MAX_FAIL_COUNT" consecutive failures of a given algorithm we stop running it.
                                                                    // Assuming problem difficulties are non-decreasing, if it consistently failed on several problems it won't suddenly succeed in solving the next problem.
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();

                    if (solvers[i].GetType() == typeof(CBS_LocalConflicts) || solvers[i].GetType() == typeof(CBS_GlobalConflicts))
                    {
                        if (((CBS_LocalConflicts)solvers[i]).mergeThreshold == 314159) // MAGIC NUMBER WHICH MAKES US ADJUST B according to map
                        {
                            ((CBS_LocalConflicts)solvers[i]).mergeThreshold = 0;
                        }
                    }


                    if (
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(CBS_LocalConflicts)) ||
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(CBS_GlobalConflicts))
                       )
                    {
                        if (((CBS_LocalConflicts)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold == 314159) // MAGIC NUMBER SEE ABOVE
                        {
                            ((CBS_LocalConflicts)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold = 0;
                        }
                    }

                    
                    int solverSolutionCost;
                    Stopwatch planningStopwatch = new Stopwatch(); ;
      
                        planningStopwatch.Start();
                        this.run(solvers[i], instance);
                        solverSolutionCost = solvers[i].GetSolutionCost();
         
                        if (solverSolutionCost < 0)
                        {
                            Console.WriteLine("TIMEOUT!!!!!!!!!!!!2");
                            solverSolutionCost = -1;
                            solutionCost = -1;
                            planningStopwatch.Stop();
                            continue;
                        }
                        if (toPrint)
                        {
                            Console.WriteLine();
                            printLinkedList(solvers[i].GetPlan().GetLocations());
                            Console.WriteLine();
                        }
                        

                        bool success = true;

                        if (solverSolutionCost >= 0) // Solved successfully
                        {
                            plan = solvers[i].GetPlan();
                            int planSize = plan.GetSize();
                            int j = 0;
                            for (LinkedListNode<List<Move>> node = plan.GetLocations().First; node != null; node = node.Next)
                            {
                                foreach (Move mv in node.Value)
                                {
                                    ((TimedMove)mv).time = j;
                                }
                                j++;
                            }

                            outOfTimeCounters[i] = 0;

                            // Validate solution:
                            if (solutionCost == -1) // Record solution cost
                            {
                                solutionCost = solverSolutionCost;
                                firstSolverToSolveIndex = i;
                            }
                            solutionCost = solverSolutionCost;
 

                            Console.WriteLine("+SUCCESS+ (:");
                        }
                        else
                        {
                            outOfTimeCounters[i]++;
                            Console.WriteLine("-FAILURE- ):");
                            success = false;
                        }

                        int originalCost = solutionCost;
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
            //this.ContinueToNextLine();
        }

        public bool isGoalEmpirically(ProblemInstance instance, ISolver solver, double pRobustProb, LinkedList<List<Move>> currentPlan, List<TimedMove> tm = null)   //for p-robust
        {
            int originalCost = solutionCost;
            TimedMove[] lastMove = new TimedMove[instance.m_vAgents.Length];
            for (int ass = 0; ass < instance.m_vAgents.Length; ass++)
            {
                lastMove[ass] = new TimedMove(instance.m_vAgents[ass].lastMove, 0); 
            }

            double c = TimedMove.updateCollisionProbability();
            int succeses = 0;
            int fails    = 0;
            int significant = 0;
            while (significant == 0)
            {
                for (int index = 0; index < 50; index += 1)
                {
                    LinkedList<List<Move>> originalLinkedList = copyLinkedList(currentPlan);
                    Array values = new ExecutePolicy[1] { ExecutePolicy.MCP };
                    foreach (ExecutePolicy policy in values)
                    {
                        AgentState[] originalAgents = new AgentState[instance.GetNumOfAgents()];
                        for (int ass = 0; ass < instance.m_vAgents.Length; ass++)
                        {
                            instance.m_vAgents[ass].lastMove = new TimedMove(lastMove[ass], 0);

                        }
                        ExecuteGivenProblem(instance, solver, c, policy, false, originalLinkedList);
                        //currentPlan = copyLinkedList(originalLinkedList);
                        solutionCost = originalCost;
                    }
                    if (replanCounter == 0)
                        succeses++;
                    else
                        fails++;
                }
                significant = checkSignificant(pRobustProb, succeses, fails);
            }
            if (significant == 1)
                return true;
            return false;
        }

        private int checkSignificant(double c, int succeses, int fails)
        {
            int x = succeses;
            int n = succeses + fails;
            //double mone;
            double p = ((double)x) / ((double)(n));
            double z_0_95 = 1.96;
            //n = 10;
            //p = 0.82;
            //c = 0.8;

            // accept the plan
            //mone = p - z_0_95 * p * (1 - p);
            double stat = p - z_0_95 * Math.Sqrt(p * (1 - p) / n);
            if (stat > c)
                return 1;

            // reject the plan
            //mone = p + z_0_95 * p * (1 - p);
            stat = p + z_0_95 * Math.Sqrt(p * (1 - p) / n);
            if (stat < c)
                return -1;

            // not significant
            return 0;

            /*
            int n = succeses + fails;
            double p_hat = ((double)succeses) / ((double)(n));
            p_hat = 0.82;
            n = 10;
            double sigma = ((p_zero) * (1 - p_zero)) / (n);
            double zed  = (p_hat - p_zero) / sigma;
            //Accord.Statistics.Testing.ZTest zTest = new Accord.Statistics.Testing.ZTest(zed, 2);
            Accord.Statistics.Testing.ZTest zTest = new Accord.Statistics.Testing.ZTest(p_hat, 2);
            double p_value = zTest.StatisticToPValue(zed);
            Console.WriteLine("n = " + n);
            Console.WriteLine("fails = " + fails);
            Console.WriteLine("succeses = " + succeses);
            Console.WriteLine("zed = " + zed);
            Console.WriteLine("p_hat = " + p_hat);
            Console.WriteLine("sigma = " + sigma);
            Console.WriteLine("p_zero = " + p_zero);
            Console.WriteLine("p_val = " + p_value);
            */

            //return true;
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

                //this.resultsWriter.Write(name + "Min Group / G&D");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Max depth");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Memory Used");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
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
        

        /// <summary>
        /// Execute given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to execute</param>
        public void ExecuteGivenProblem(ProblemInstance instance, ISolver solver, double chance, ExecutePolicy policy, bool writeFile = false, LinkedList<List<Move>> currentPlan = null)
        {
            int solutionSumOfCost;

            if (currentPlan == null)
            {
                currentPlan = copyLinkedList(plan.GetLocations());
                solutionSumOfCost = solutionCost;
                
            }
            else
                solutionSumOfCost = computeSumOfCost(currentPlan);
            double planTime = planningTime;
            int newCost     = 0;
            
            LinkedList<List<Move>> originalLinkedList = copyLinkedList(currentPlan);
            //Console.WriteLine("***** EXECUTE    Policy = "+ policy.ToString() +"  *****");
            replanTime                      = 0;
            sw                              = new Stopwatch();
            replanCounter                   = 0;
            LinkedList<List<Move>> newPlan  = new LinkedList<List<Move>>();

            if(policy == ExecutePolicy.MCP)
            {
                buildMcpGraph(currentPlan);
            }
  
            GC.Collect();
            GC.WaitForPendingFinalizers();
            string solverName = "";
            if (solver.GetType() == typeof(CBS_LocalConflicts) || solver.GetType() == typeof(CBS_GlobalConflicts))
                solverName = "ICBS + " + ((CBS_LocalConflicts)solver).conflictRange;
            else
                solverName = "EPEA*";
            sw.Start();
          //  try
         //   {
                if (solutionSumOfCost > 0)
                {
                    if(WRITE_LOG)
                    {
                        writeHeaderToLog(
                            instanceId.ToString(),
                            policy,
                            m_agentNum.ToString(),
                            solverName,
                            chance,
                            m_mapFileName
                         );
                    }
                    writeLineToLog("Original Plan:");
                    LinkedList<List<Move>> lLocations = currentPlan;
                    //Dictionary<Move, List<Tuple<int, int>>> ggg = getJunctions(lLocations);
                    //Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> fff = getAgentsProbability(ggg, chance);
                    //Dictionary<Move, double> hhh = getLocationsProbability(fff);
                    double executionCost = executePlan(this, instance, newPlan, policy, chance, solver, currentPlan /*, hhh, fff*/);
                    newCost                 = computeSumOfCost(newPlan);
                    writeLineToLog("Execution Plan:");
                    if(toPrint)
                        printLinkedList(newPlan, WRITE_LOG);
                    checkValidExecution(originalLinkedList, newPlan);
                    //Console.WriteLine(fff);
                    //Console.WriteLine(ggg);
                    //Console.WriteLine(hhh);
            }
                else
                    newCost = -2;
        //    }
           /* catch(Exception e)
            {
                newCost = -3;
            }*/
            sw.Stop();
          //  Console.WriteLine("Execution:");
          //  printLinkedList(newPlan);  //print plan
          //  Console.WriteLine("");
          //  Console.Read();

            if(writeFile)
                writeToFile(chance,                         // chance for exec. mistake
                    false,                                  // header
                    solverName,                             // solver name
                    planTime.ToString(),                    // planning time             
                    sw.Elapsed.TotalMilliseconds.ToString(),// total exec. time
                    solutionSumOfCost.ToString(),           // solution cost
                    newCost.ToString(),                     // exec. cost
                    instanceId.ToString(),                  // instanceId                         
                    instance.m_vAgents.Length.ToString(),   // #Agents
                    this,                                   // runner
                    policy,                                 // Execution Policy (Lazy, Eager ..)
                    m_mapFileName,                          // Map name
                    instance.m_nObstacles,                  // Obstacles
                    ((CBS_GlobalConflicts)solver).constraintPolicy,  // Constraint Policy
                    solver);                                // Solver
           
        }

        private Dictionary<Move, List<Tuple<int, int>>> getJunctions(LinkedList<List<Move>> linkedList)
        {
            LinkedListNode<List<Move>> node = linkedList.First;
            LinkedListNode<List<Move>> prev = null;
            Dictionary<Move, List<Tuple<int, int>>> locationToAgentTimeDic = new Dictionary<Move, List<Tuple<int, int>>>();
            int[] times = new int[node.Value.Count];
            bool first = true;
            while(node != linkedList.Last)
            {
                if (!first)
                    node = node.Next;
                else
                    first = false;
                for (int agent = 0; agent < node.Value.Count; agent++)
                {
                    if (prev != null)
                        if (prev.Value[agent].x != node.Value[agent].x || prev.Value[agent].y != node.Value[agent].y)
                            times[agent]++;
                    Move currentMove = new Move(node.Value[agent].x, node.Value[agent].y, Move.Direction.NO_DIRECTION);
                    Tuple<int, int> currentTuple = new Tuple<int, int>(agent, times[agent]);
                    List<Tuple<int, int>> currentLocationList;
                    if (!locationToAgentTimeDic.Keys.Contains(currentMove))
                    {
                        currentLocationList = new List<Tuple<int, int>>();
                    }
                    else
                        currentLocationList = locationToAgentTimeDic[currentMove];
                    currentLocationList.Add(currentTuple);
                    locationToAgentTimeDic[currentMove] = currentLocationList;
                }
                prev = node;
            }
            return locationToAgentTimeDic;
        }

        private Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> getAgentsProbability(Dictionary<Move, List<Tuple<int, int>>> locationToAgentTimeDic, double prob)
        {
            Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationToAgentsProbability = new Dictionary<Move, List<Tuple<Tuple<int, int>, double>>>();
            foreach(Move currentMove in locationToAgentTimeDic.Keys)
            {
                List<Tuple<int, int>> currentList = locationToAgentTimeDic[currentMove];
                foreach(Tuple<int,int> currentTupleA in currentList)
                {
                    foreach (Tuple<int, int> currentTupleB in currentList)
                    {
                        int agentA = currentTupleA.Item1;
                        int agentB = currentTupleB.Item1;
                        if(agentA < agentB)
                        {
                            int timeA = currentTupleA.Item2;
                            int timeB = currentTupleB.Item2;
                            double collisionProb;
                            if (timeA > timeB)
                                collisionProb = TimedMove.calculateCollisionProbability(prob, timeA - timeB, timeB);
                            else
                                collisionProb = TimedMove.calculateCollisionProbability(prob, timeB - timeA, timeA);
                            Tuple<int, int> twoAgent = new Tuple<int, int>(agentA, agentB);
                            Tuple<Tuple<int, int>, double> twoAgentAndProb = new Tuple<Tuple<int, int>, double>(twoAgent, collisionProb);
                            List<Tuple<Tuple<int, int>, double>> currentOrNewList;
                            if(!locationToAgentsProbability.Keys.Contains(currentMove))
                                currentOrNewList = new List<Tuple<Tuple<int,int>,double>>();
                            else
                                currentOrNewList = locationToAgentsProbability[currentMove];
                            currentOrNewList.Add(twoAgentAndProb);
                            locationToAgentsProbability[currentMove] = currentOrNewList;
                        }
                    }
                }
            }
            return locationToAgentsProbability;
        }

        private Dictionary<Move, double> getLocationsProbability(Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationToAgentsProbability)
        {
            Dictionary<Move, double> moveToProb = new Dictionary<Move, double>();
            foreach(Move currentMove in locationToAgentsProbability.Keys)
            {
                double sumProb = 1;
                List<Tuple<Tuple<int, int>, double>> currentList = locationToAgentsProbability[currentMove];
                foreach(Tuple<Tuple<int, int>, double> currentTuple in currentList)
                {
                    double currentProb = currentTuple.Item2;
                    sumProb = sumProb * (1 - currentProb);
                }
                moveToProb.Add(currentMove, 1 - sumProb);
            }
            return moveToProb;
        }

        private int[] createFirstMonitorsLocations(int monitorsCount)
        {
            int[] locations = new int[monitorsCount];
            for (int i = 0; i < locations.Count(); i++)
                locations[i] = i;
            return locations;
        }

        private int[] getNextChoose(int[] arr, int chooseSize)
        {
            int index = arr.Count() - 1;
            bool done = false;
            while(!done)
            {
                if(index == -1)
                {
                    return null;
                }
                else if(arr[index] + arr.Count() - index == chooseSize)
                {
                    index --;
                }
                else
                {
                    bool first = true;
                    while(index != arr.Count())
                    {
                        if(first)
                        {
                            arr[index] ++;
                            first = false;
                        }
                        else
                            arr[index] = arr[index - 1] + 1;
                        index ++;
                    }
                    done = true;
                }
            }
            return arr;
        }

        private Dictionary<KeyValuePair<int, int>, LinkedList<KeyValuePair<int, int>>> McpGraph;

        /// <summary>
        /// build the mcp graph
        /// </summary>
        private void buildMcpGraph(LinkedList<List<Move>> currentPlan)
        {
            LinkedList<List<Move>> lLocations           = currentPlan;
            LinkedListNode<List<Move>> lLocationsNode   = lLocations.First;
            List<Move> previousTime                     = lLocationsNode.Value;
            lLocationsNode                              = lLocationsNode.Next;
            List<Move> currentTime                      = lLocationsNode.Value;
            McpGraph = new Dictionary<KeyValuePair<int, int>, LinkedList<KeyValuePair<int, int>>>();
            for (int i = 1; i < lLocations.Count; i++)
            {
                for (int agent = 0; agent < currentTime.Count; agent++)
                {
                    Move currentAgentMove  = currentTime[agent];
                    Move previousAgentMove = previousTime[agent];
                    if (previousAgentMove.x != currentAgentMove.x || previousAgentMove.y != currentAgentMove.y)
                    {
                        KeyValuePair<int, int> agentLocation = new KeyValuePair<int, int>(previousAgentMove.x, previousAgentMove.y);
                        KeyValuePair<int, int> timeAgent     = new KeyValuePair<int, int>(i, agent);
                        if (!McpGraph.Keys.Contains(agentLocation))
                        {
                            LinkedList<KeyValuePair<int, int>> queue = new LinkedList<KeyValuePair<int, int>>();
                            queue.AddLast(timeAgent);
                            McpGraph.Add(agentLocation, queue);
                        }
                        else
                        {
                            McpGraph[agentLocation].AddLast(timeAgent);
                        }
                    }
                }
                previousTime = currentTime;
                if (i != lLocations.Count - 1)
                {
                    lLocationsNode = lLocationsNode.Next;
                    currentTime    = lLocationsNode.Value;
                }  
            }
        }

        /// <summary>
        /// create new log file
        /// </summary>
        public void creatLogFile()
        {
            if (!WRITE_LOG)
                return;
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustLog.txt";
            using (StreamWriter sw = File.CreateText(filePath))
            {
            }
        }

        /// <summary>
        /// write header to log
        /// </summary>
        public void writeHeaderToLog
        (
            string instanceId, 
            ExecutePolicy policy, 
            string agentsCount, 
            string solverName, 
            double chanceForExecutionMistake, 
            string mapName
        )
        {
            if (!WRITE_LOG)
                return;
            string pathDesktop  = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath     = pathDesktop + "\\RobustLog.txt";
            if (!File.Exists(filePath))
                creatLogFile();
            string[] lines = {
                "---------------------------------------------------------------------------------------------------",
                "instance: "    + instanceId                  + " | " +
                "policy: "      + policyToString(policy)      + " | " +
                "#agents: "     + agentsCount                 + " | " +
                "solver: "      + solverName                  + " | " +
                "chance: "      + chanceForExecutionMistake   + " | " +
                "map: "         + mapName,
                "---------------------------------------------------------------------------------------------------" };
            using (StreamWriter file = File.AppendText(filePath))
            //using (System.IO.StreamWriter file = new System.IO.StreamWriter(filePath))
            {
                foreach (string line in lines)
                {
                    file.WriteLine(line);
                }
            }
        }

        /// <summary>
        /// write line to log
        /// </summary>
        public void writeLineToLog
        (
            string line
        )
        {
            if (!WRITE_LOG)
                return;
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustLog.txt";
            using (StreamWriter file = File.AppendText(filePath))
            {
                file.WriteLine(line);
            }
        }

        /// <summary>
        /// write execution info to file
        /// </summary>
        public void writeToFile(double chanceForExecutionMistake, bool header, string solver, string planTime, string elapsedTime, string planCost, string executionCost, string instanceId, string agentsCount, Run runner, ExecutePolicy policy, string mapFileName, uint obstaclesPercents, ConstraintPolicy constraintPolicy, ISolver isolver)
        {
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustCSV.csv";

            string delimter = ",";
            List<string[]> output = new List<string[]>();
            string[] temps = new string[17];

            if (header && File.Exists(filePath))
                return;

            if (!File.Exists(filePath))
            {
                File.Create(filePath).Close();
            }
            if (header)
            {
                temps[0] = "IncstanceId";
                temps[1] = "#Agents";
                temps[2] = "Error Prob.";
                temps[3] = "Solver";
                temps[4] = "Policy";
                temps[5] = "Map";
                temps[6] = "Obstacles";
                temps[7] = "Plan Cost";
                temps[8] = "Exec. Cost";
                temps[9] = "Success";
                temps[10] = "#Replans";
                temps[11] = "Plan time";
                temps[12] = "Replan time";
                temps[13] = "Constraint policy";
            }
            if (!header)
            {
                temps[0] = instanceId;
                temps[1] = agentsCount;
                temps[2] = chanceForExecutionMistake.ToString();
                temps[3] = solver;
                temps[4] = policyToString(policy);
                temps[5] = Path.GetFileNameWithoutExtension(mapFileName);
                temps[6] = obstaclesPercents.ToString(); ;
                temps[7] = planCost;
                temps[8] = executionCost;
                if (temps[8].Equals("-1") || temps[8].Equals("-2") || temps[8].Equals("-3"))
                {
                    //temps[7] = "-1";
                    temps[9] = "0";
                }
                else
                    temps[9] = "1";
                temps[10] =  replanCounter.ToString();
                temps[11] = planTime;
                temps[12] = replanTime.ToString();
                temps[13] = constraintPolicyToString(constraintPolicy);
                temps[14] = ((CBS_GlobalConflicts)isolver).GetMemoryUsed().ToString();
                temps[15] = ((CBS_GlobalConflicts)isolver).GetSolutionDepth().ToString();
                temps[16] = ((CBS_GlobalConflicts)isolver).GetGenerated().ToString();
            }
            output.Add(temps);

            int length = output.Count;
            using (System.IO.TextWriter writer = File.AppendText(filePath))
            {
                for (int index = 0; index < length; index++)
                {
                    writer.WriteLine(string.Join(delimter, output[index]));
                }
            }
        }

        /// <summary>
        /// convert policy to string
        /// </summary>
        private string policyToString(ExecutePolicy policy)
        {
            if (policy == ExecutePolicy.Lazy)
                return "Lazy";
            else if (policy == ExecutePolicy.Stressful)
                return "Stressful";
            else if (policy == ExecutePolicy.Reasonable)
                return "Reasonable";
            else if (policy == ExecutePolicy.Stressful_Repair)
                return "Stressful_Repair";
            else if (policy == ExecutePolicy.Reasonable_Repair)
                return "Reasonable_Repair";
            else if (policy == ExecutePolicy.MCP)
                return "MCP";
            else
                return "NO_POLICY";
        }

        /// <summary>
        /// convert constraint policy to string
        /// </summary>
        private string constraintPolicyToString(ConstraintPolicy policy)
        {
            if (policy == ConstraintPolicy.Single)
                return "Single";
            else if (policy == ConstraintPolicy.Range)
                return "Range";
            else if (policy == ConstraintPolicy.DoubleRange)
                return "Double Range";
            else
                return "NO_POLICY";
        }

        /// <summary>
        /// compute sum of cost for a given plan
        /// </summary>
        public int computeSumOfCost(LinkedList<List<Move>> newPlan)// Run runner, ProblemInstance instance)
        {
            int agentNumber = newPlan.ElementAt<List<Move>>(0).Count;//runner.plan.GetLocations().ElementAt<List<Move>>(0).Count;
            int totalCost = 0;
            int currentCounter = 0;
            int currentCost = 0;
            Move previousMove = null;
            for (int i = 0; i < agentNumber; i++)
            {
                currentCounter = 0;
                currentCost = 0;
                foreach (List<Move> locationsAtTime in newPlan)//runner.plan.GetLocations())
                {
                    Move currentMove = locationsAtTime[i];
                    if (currentCounter != 0 && (currentMove.x != previousMove.x || currentMove.y != previousMove.y))
                        currentCost = currentCounter;
                    previousMove = locationsAtTime[i];
                    currentCounter++;
                }
                totalCost += currentCost;
            }
            return totalCost;
        }

        private void printMcpGraph()
        {
            foreach (KeyValuePair<int, int> loc in McpGraph.Keys)
            {
                Console.WriteLine("Location: " + loc.Key + "," + loc.Value);
                for (LinkedListNode<KeyValuePair<int, int>> value = McpGraph[loc].First; value != null; value = value.Next)
                {
                    Console.WriteLine("                     Time: " + value.Value.Key + "    Agent: " + value.Value.Value);
                }
            }
        }

        public static Stopwatch replanStopwath;

        /// <summary>
        /// execute single plan
        /// </summary>
        public int executePlan(Run runner, ProblemInstance instance, LinkedList<List<Move>> newPaths, ExecutePolicy policy, double chance, ISolver solver, LinkedList<List<Move>> currentPlan = null, Dictionary<Move, double> locationProb = null, Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationCountProb = null)
        {
            bool robustExecution = true;
            

            int kValue = ((CBS_LocalConflicts)solver).conflictRange;

            if (currentPlan == null)
                currentPlan = runner.plan.GetLocations();
            //print graph - test
            //Dictionary < KeyValuePair<int, int>, LinkedList >
            //printMcpGraph();
            //end test
            if (toPrint)
                printLinkedList(plan.GetLocations(), WRITE_LOG);
           /* if (sw.ElapsedMilliseconds > Run.TIMEOUT)
                throw new Exception("1");*/
            int cost                            = 0;
            List<Move> previousTime             = null;
            List<Move> delayMoves               = new List<Move>();
            List<int> delayAgents               = new List<int>();
            LinkedList<List<Move>> lLocations   = currentPlan;
            int dynamicSize                     = lLocations.Count;
            LinkedListNode<List<Move>> node     = lLocations.First;
            LinkedListNode<List<Move>> kNode    = lLocations.First;
            List<Move> locationsAtTime          = node.Value;
            List<List<Move>> kLocationsAtTime   = new List<List<Move>>();
            Dictionary<int,int> kDelayAgents        = new Dictionary<int,int>();
            kDelayAgents = new Dictionary<int, int>();
            for (int agent = 0; agent < locationsAtTime.Count; agent++)
                kDelayAgents[agent] = 0;
            // LinkedList<List<Move>> LinkedList = copyLinkedList(lLocations);
            //  printLinkedList(runner.plan.GetLocations());  //print plan
            //  Console.WriteLine("");
            for (int i = 0; i < dynamicSize; i++)
            {
                

                if (robustExecution && i !=0 && (kValue == 0 || i % kValue == 0) && i != 0 && (policy == ExecutePolicy.Lazy || policy == ExecutePolicy.Reasonable || policy == ExecutePolicy.Reasonable_Repair))
                {
                    kLocationsAtTime = new List<List<Move>>();      // for robust exectuion policies
                    kNode = lLocations.First;
                    int p;
                    for (p = 0; p <= i; p++)
                        kNode = kNode.Next;
                    if (policy == ExecutePolicy.Reasonable || policy == ExecutePolicy.Reasonable_Repair)
                        for (; p < dynamicSize; p++)
                        {
                            kLocationsAtTime.Add(kNode.Value);
                            kNode = kNode.Next;
                        }
                    if (policy == ExecutePolicy.Lazy)
                        for (int k = 0; k <= kValue && k + p < dynamicSize; k++)
                        {
                            kLocationsAtTime.Add(kNode.Value);
                            kNode = kNode.Next;
                        }
                }

                cost++;
                if (i != dynamicSize - 1)
                {
                    //foreach (Move aMove in locationsAtTime)
                    for(int iMove = 0; iMove < locationsAtTime.Count; iMove++)
                    {
                        Move aMove = locationsAtTime[iMove];
                        double randomChance = rand.NextDouble();
                        int agentIndex = locationsAtTime.IndexOf(aMove);
                        Move agentNextMove = node.Next.Value[agentIndex];
                        if ((randomChance < chance) && (agentNextMove.x != aMove.x || agentNextMove.y != aMove.y))
                        {
                            delayMoves.Add((Move)aMove);
                            if(delayAgents.Contains(iMove))
                                Console.WriteLine("Problem!!");
                            delayAgents.Add(iMove);
                            kDelayAgents[iMove]++;

                            if (toPrint)
                                Console.WriteLine("Agent: " + iMove + ", At time: " + i);
                            writeLineToLog("Delay agent: " + iMove + ", At time: " + i);
                        }
                    }
                }
                //if (robustExecution && i % kValue == 0 && i != 0)
                //    agentDelay(delayAgents, delayMoves, runner, locationsAtTime, i, lLocations);
 

                if (policy == ExecutePolicy.MCP)
                {
                    if (node.Next != null)
                    {
                        List<Move> nextMove = node.Next.Value;
                        bool newInformation;
                        do
                        {
                            newInformation = false;
                            for(int aMoveIndex = 0; aMoveIndex < nextMove.Count; aMoveIndex++)
                            {
                                Move aMove = nextMove[aMoveIndex];
                                int x = aMove.x;
                                int y = aMove.y;
                                KeyValuePair<int, int> agentLocation = new KeyValuePair<int, int>(x, y);                
                                if ((x != locationsAtTime[aMoveIndex].x || y != locationsAtTime[aMoveIndex].y) &&       //the agent want to move
                                    McpGraph.ContainsKey(agentLocation)                                        &&       //Mcp graph contains the next location
                                    McpGraph[agentLocation].First != null)                                              //location queue not empty
                                {
                                    int nextAgentAtLocation     = (McpGraph[agentLocation].First).Value.Value;          //first in queue
                                    int nextNextAgentAtLocation = -1;
                                    if (McpGraph[agentLocation].First.Next != null)
                                        nextNextAgentAtLocation = (McpGraph[agentLocation].First).Next.Value.Value;     //next in queue
                                    if (!delayAgents.Contains(aMoveIndex))                                              //not already delayed
                                    {
                                        if (nextAgentAtLocation != aMoveIndex)                                          //not the next agent to leave the location
                                        {
                                            if (locationsAtTime[nextAgentAtLocation].x != aMove.x ||
                                                locationsAtTime[nextAgentAtLocation].y != aMove.y ||
                                                delayAgents.Contains(nextAgentAtLocation)         ||
                                                (nextMove[nextAgentAtLocation].x == locationsAtTime[nextAgentAtLocation].x && nextMove[nextAgentAtLocation].y == locationsAtTime[nextAgentAtLocation].y)
                                                /*    ||
                                                (nextNextAgentAtLocation != -1 && nextNextAgentAtLocation != aMoveIndex)*/)   //the next agent at location not there right now or there and delayed or not the after next at location or his next step is to stay
                                            {
                                                if (toPrint)
                                                    Console.WriteLine("Next Agent at location: " + locationsAtTime[nextAgentAtLocation].x + "," + locationsAtTime[nextAgentAtLocation].y + " should be " +nextAgentAtLocation);
                                                delayMoves.Add((Move)aMove);
                                                delayAgents.Add(aMoveIndex);
                                                newInformation = true;
                                                if (toPrint)
                                                    Console.WriteLine("***** MCP REPAIR ***** Add delay to agent " + aMoveIndex + " at time " + i);
                                                replanCounter++;
                                                writeLineToLog("***** MCP REPAIR *****");
                                            }
                                            else if (nextNextAgentAtLocation != -1 && nextNextAgentAtLocation != aMoveIndex)   //the next agent at location not there right now or there and delayed or not the after next at location
                                            {
                                                if (toPrint)
                                                    Console.WriteLine("Next Agent at location: " + locationsAtTime[nextAgentAtLocation].x + "," + locationsAtTime[nextAgentAtLocation].y + " should be " + nextAgentAtLocation);
                                                delayMoves.Add((Move)aMove);
                                                delayAgents.Add(aMoveIndex);
                                                newInformation = true;
                                                if (toPrint)
                                                    Console.WriteLine("***** MCP REPAIR ***** Add delay to agent " + aMoveIndex + " at time " + i);
                                                replanCounter++;
                                                writeLineToLog("***** MCP REPAIR *****");
                                            }
                                        }
                                    }
                                }
                            }
                        } while (newInformation);

                        agentDelay(delayAgents, delayMoves, runner, locationsAtTime, i, lLocations);

                        for (int aMoveIndex = 0; aMoveIndex < nextMove.Count; aMoveIndex++)
                        {
                            int x = locationsAtTime[aMoveIndex].x;
                            int y = locationsAtTime[aMoveIndex].y;
                            KeyValuePair<int, int> agentLocation = new KeyValuePair<int, int>(x, y);
                            if (!delayAgents.Contains(aMoveIndex))                              //if the agent didnt delay and did a move
                                if(x != nextMove[aMoveIndex].x || y != nextMove[aMoveIndex].y )
                                {
                                    McpGraph[agentLocation].RemoveFirst();
                                    if (toPrint)
                                        Console.WriteLine("Remove: Agent "+ aMoveIndex + "  at time +" + i + "  from loc "+ x +","+ y);
                                }
                        }
                    }
                }
                else if (delayMoves.Count > 0)
                {
                    //printLinkedList(runner.plan.GetLocations());  //print plan
                    //Console.Read();   //print plan

                    //   Console.Read();
                    //bool aa = planCheck(runner);

                    //   Console.WriteLine("compare 1:");
                    //   compareLinkedList(LinkedList, lLocations);
                    //   Console.WriteLine("done.");
                    if (policy == ExecutePolicy.Stressful_Repair)
                    {
                        List<int> additionalDelayAgents = new List<int>();
                        foreach (Move aMove in locationsAtTime)
                        {
                            //if(!delayAgents.Contains(locationsAtTime.IndexOf((Move)aMove)))
                            additionalDelayAgents.Add(locationsAtTime.IndexOf((Move)aMove));
                        }
                        agentDelay(additionalDelayAgents, delayMoves, runner, locationsAtTime, i);
                        if (toPrint)
                            Console.WriteLine("***** REPAIR *****");
                        writeLineToLog("***** REPAIR *****");
                        replanCounter++;
                        writeLineToLog("Delay all agents at time: " + i);
                    }
                    //LinkedList<List<Move>> LinkedListCopy = copyLinkedList(runner.plan.locationsAtTimes);
                    //agentDelay(delayAgents, delayMoves, runner, previousTime, i);
                    else if (policy == ExecutePolicy.Reasonable_Repair/* || policy == ExecutePolicy.Reasonable*/)
                    {
                        LinkedList<List<Move>> LinkedListCopy = copyLinkedList(runner.plan.locationsAtTimes);
                        agentDelayTest(delayAgents, delayMoves, LinkedListCopy, locationsAtTime, i);
                        //agentDelay(delayAgents, delayMoves, runner, locationsAtTime, i);
                        if (checkIfWillCollide(LinkedListCopy, i))
                        {
                            writeLineToLog("Check colliding plan:");
                            if(toPrint)
                                printLinkedList(LinkedListCopy, WRITE_LOG);
                            //runner.plan.locationsAtTimes = copyLinkedList(LinkedListCopy);
                            // Console.WriteLine("***** REPAIR *****");
                            //replanCounter++;
                            //    if (policy == ExecutePolicy.Reasonable_Repair)
                            //   {
                            List<int> additionalDelayAgents = new List<int>();
                            foreach (Move aMove in locationsAtTime)
                            {
                                //if (!delayAgents.Contains(locationsAtTime.IndexOf((Move)aMove)))
                                additionalDelayAgents.Add(locationsAtTime.IndexOf((Move)aMove));
                            }
                            agentDelay(additionalDelayAgents, delayMoves, runner, locationsAtTime, i);
                            if (toPrint)
                                Console.WriteLine("***** REPAIR *****");
                            replanCounter++;
                            writeLineToLog("***** REPAIR *****");
                            writeLineToLog("Delay all agents at time: " + i);
                        }
                        else
                        {
                            agentDelay(delayAgents, delayMoves, runner, locationsAtTime, i);
                        }
                    }
                    else
                        agentDelay(delayAgents, delayMoves, runner, locationsAtTime, i);  //changed to this time(locationsAtTime) and not pre
                                                                                          //   Console.WriteLine("compare 2:");
                                                                                          //   compareLinkedList(LinkedList, lLocations);
                                                                                          //   Console.WriteLine("done.");
                                                                                          //   Console.Read();
                                                                                          //bool bb = planCheck(runner);
                                                                                          /*if(!aa || !bb)
                                                                                          {
                                                                                              if(!aa)
                                                                                                  Console.WriteLine("PROBLEM AT AA");
                                                                                              if (!bb)
                                                                                                  Console.WriteLine("PROBLEM AT BB");
                                                                                              Console.Read();
                                                                                          }*/
                                                                                          //   printLinkedList(runner.plan.GetLocations());  //print plan
                                                                                          //   Console.WriteLine("");
                }

                bool toReplan = false;
                if (robustExecution && (kValue == 0 || i % kValue == 0) && i != 0 && (policy == ExecutePolicy.Lazy || policy == ExecutePolicy.Reasonable))
                    toReplan = checkIfKColide(kLocationsAtTime);

                if (robustExecution && (kValue == 0 || i % kValue == 0) && i != 0 && policy == ExecutePolicy.Stressful)
                {
                    for (int agent = 0; agent < locationsAtTime.Count; agent++)
                    {
                        if (kDelayAgents[agent] > 0)
                            toReplan = true;
                    }
                }

                    toReplan = checkIfKColide(kLocationsAtTime);
                //stressfull k

                if ((policy == ExecutePolicy.Lazy           && toReplan/*checkIfColide(locationsAtTime, locationProb, locationCountProb)*/)      ||
                    (policy == ExecutePolicy.Stressful      && toReplan/*delayMoves.Count > 0*/)                ||  
                    (policy == ExecutePolicy.Reasonable     && toReplan/*checkIfWillCollide(runner.plan.locationsAtTimes, i)*/)) //i != 0 &&
                {
                    if(toPrint)
                        Console.WriteLine("***** REPLAN *****");
                    writeLineToLog("***** REPLAN *****");
                    //printLinkedList(newPaths);
                    replanCounter++;
                    
                   /* if (previousTime == null)
                    {
                        Console.WriteLine("PROBLEM!!");
                        Console.WriteLine("i = " + i);
                        Console.Read();
                    }*/
                    replanStopwath = new Stopwatch();
                    replanStopwath.Start();
                    try
                    {
                        if (policy == ExecutePolicy.Lazy || /*toReplan*/checkIfColide(locationsAtTime, locationProb, locationCountProb))
                        {
                            //newPaths.Remove(newPaths.Last);
                            List<int> addAgentsToDelay = new List<int>();
                            correctAgentsCollision(previousTime, ref locationsAtTime/*, ref addAgentsToDelay*/);
                            //agentDelay(addAgentsToDelay, null, runner, previousTime, i - 1);
                            replan(locationsAtTime, instance, runner, solver);

                            //LinkedList<List<Move>> mLocations = this.plan.GetLocations();
                            //Dictionary<Move, List<Tuple<int, int>>> ggg = getJunctions(mLocations);
                            //Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> fff = getAgentsProbability(ggg, chance);
                            //locationProb = getLocationsProbability(fff);
                            //locationCountProb = fff;

                            replan(previousTime, instance, runner, solver);
                            writeLineToLog("Replan at time: " + i);
                        }
                        else if (policy == ExecutePolicy.Reasonable)
                        {
                            /* if (!checkIfColide(node.Next.Value))
                             {
                                 newPaths.AddLast(locationsAtTime);
                                 replan(node.Next.Value, instance, runner, solver);
                             }
                             else
                             {
                                 replan(locationsAtTime, instance, runner, solver);
                             }*/
                            replan(locationsAtTime, instance, runner, solver);
                        }
                        else if (policy == ExecutePolicy.Stressful)
                        {
                        //       printLinkedList(newPaths);
                        //       Console.WriteLine("");
                           /* if (!checkIfColide(node.Next.Value))
                            {
                                newPaths.AddLast(locationsAtTime);
                                replan(node.Next.Value, instance, runner, solver);
                            }
                            else
                            { */
                                replan(locationsAtTime, instance, runner, solver);
                            //}
                        }
                    }
                    catch(Exception e)
                    {
                        replanStopwath.Stop();
                        replanTime += replanStopwath.Elapsed.TotalMilliseconds;
                        throw e;
                    }
                    replanStopwath.Stop();
                    replanTime  += replanStopwath.Elapsed.TotalMilliseconds;
                    cost        += executePlan(runner, instance, newPaths, policy, chance, solver);
                    return cost;
                }
                delayAgents = new List<int>();
                delayMoves  = new List<Move>();
                newPaths.AddLast(locationsAtTime);
                previousTime = locationsAtTime;
                lLocations = runner.plan.locationsAtTimes;
                if (node != lLocations.Last)
                {
                    node = lLocations.First;
                    for (int p = 0; p <= i; p++)
                        node = node.Next;
                    locationsAtTime = node.Value;
                }
                dynamicSize = lLocations.Count;

                if (i != 0 && (kValue == 0 || i % kValue == 0) && (policy == ExecutePolicy.Stressful || policy == ExecutePolicy.Stressful_Repair || policy == ExecutePolicy.Reasonable_Repair))
                {
                    kDelayAgents = new Dictionary<int, int>();
                    for (int agent = 0; agent < locationsAtTime.Count; agent++)
                        kDelayAgents[agent] = 0;
                }
            }
            return cost;
        }

        private bool checkValidBiasPlan(LinkedList<List<Move>> plan, int bias)
        {
            LinkedListNode<List<Move>> node = plan.First;
            LinkedListNode<List<Move>> biasnode;
            List<Move> biasMove;
            List<Move> cur = node.Value;
            bool valid = true;
            for (int i = 0; i < plan.Count; i++)
            {

                for (int j = 0; j < bias + 1; j++)
                {
                    biasnode = node;
                    for (int p = 0; p < j && biasnode.Next != null; p++)
                    {
                        biasnode = biasnode.Next;
                    }
                    biasMove = biasnode.Value;
                    for (int aMove = 0; aMove < plan.First.Value.Count; aMove++)
                    {
                        for (int bMove = 0; bMove < plan.First.Value.Count; bMove++)
                        {
                            if (aMove == bMove)
                                continue;
                            Move agent1 = cur[aMove];
                            Move agent2 = biasMove[bMove];
                            if (agent1.IsColliding(agent2))
                            {
                                if (toPrint)
                                    Console.WriteLine("Agents " + aMove + " and " + bMove + " collides at time " + i + " bias " + j);
                                valid = false;
                            }
                        }
                    }
                }
                node = node.Next;
                if (node != null)
                    cur  = node.Value;
            }
            if (valid)
            {
                if (toPrint)
                    Console.WriteLine("A valid bias plan!");
            }
            else
            {
                //if (toPrint)
                    Console.WriteLine("Not a valid bias plan!");
            }
            return valid;
        }

        private bool checkValidExecution(LinkedList<List<Move>> originalLinkedList, LinkedList<List<Move>> exePlan)
        {
            LinkedListNode<List<Move>> node = exePlan.First;
            List<Move> pre;
            List<Move> cur;
            bool valid = true;
            for (int i = 1; i < exePlan.Count; i++)
            {
                pre  = node.Value;
                node = node.Next;
                cur  = node.Value;
                for(int aMove = 0; aMove < cur.Count; aMove++)
                {
                    if (!isFollowingMove(pre[aMove], cur[aMove]))
                    {
                        if(toPrint)
                            Console.WriteLine("Agent: " + aMove + ", at time " + (i-1) + " at " + pre[aMove] + "and at time " + i + " at " + cur[aMove]);
                        valid = false;
                    }
                    for(int bMove = aMove + 1; bMove < cur.Count; bMove++)
                    {
                        if (pre[aMove].IsColliding(pre[bMove]))
                        {
                            if (toPrint)
                                Console.WriteLine("Agents: " + aMove + " and " + bMove + " collides at time " + i);
                            valid = false;
                        }
                    }
                }
                for(int sMove = 0; sMove < originalLinkedList.First.Value.Count; sMove++)
                {
                    Move sOriginal = originalLinkedList.First.Value[sMove];
                    Move sNew      = exePlan.First.Value[sMove];
                    Move gOriginal = originalLinkedList.Last.Value[sMove];
                    Move gNew      = exePlan.Last.Value[sMove];
                    if (sOriginal.x != sNew.x || sOriginal.y != sNew.y ||
                        gOriginal.x != gNew.x || gOriginal.y != gNew.y)
                    {
                        if (toPrint)
                            Console.WriteLine("wrong start or goal");
                        valid = false;
                    }
                }
            }
            if (toPrint)
            {
                if (valid)
                    Console.WriteLine("A valid execution!");
                else
                    Console.WriteLine("Not a valid execution!");
            }
            return valid;
        }

        private bool isFollowingMove(Move a, Move b)
        {
            if (Math.Abs(a.x - b.x) > 0 &&
                Math.Abs(a.y - b.y) > 0)
                return false;
            return true;
        }

        private void correctAgentsCollision(List<Move> preList, ref List<Move> colList/*, ref List<int> addAgentsToDelay*/)
        {
            //List<TimedMove> newList = new List<TimedMove>();
            bool hasCollision = false;
            for (int i = 0; i < colList.Count; i++)
            {
                for (int j = i + 1; j < colList.Count; j++)
                {
                    Move aMove = colList[i];
                    Move bMove = colList[j];
                    if (aMove.IsColliding(bMove)) //aMove != bMove && 
                    {

                        if (aMove.x == preList[i].x && aMove.y == preList[i].y)
                        {
                            //addAgentsToDelay.Add(j);
                            colList[j] = new Move(preList[j]);
                            //Console.WriteLine("****         correct Agent: " + j);
                        }
                        else if (bMove.x == preList[j].x && bMove.y == preList[j].y)
                        //addAgentsToDelay.Add(i);
                        { 
                        colList[i] = new Move(preList[i]);
                         //   Console.WriteLine("****         correct Agent: " + i);
                        }
                    else
                    {
                        //addAgentsToDelay.Add(i);
                        //addAgentsToDelay.Add(j);
                       // Console.WriteLine("****         correct Agent: " + i +"   and Agent: " + j);
                        colList[j] = new Move(preList[j]);
                        colList[i] = new Move(preList[i]);
                    }
                        hasCollision = true;
                    }
                }
            }
            if (hasCollision)
            {
                correctAgentsCollision(preList, ref colList/*, ref addAgentsToDelay*/);
                return ;
            }
        }

        private void printLinkedList(LinkedList<List<Move>> toPrint, bool writeToFile = false)
        {
            if (toPrint.Count == 0)
                return;
            PrintLine(writeToFile);
            LinkedListNode<List<Move>> node = toPrint.First;
            string[] columns = new string[node.Value.Count + 1];
            columns[0] = "";
            for(int agentNumber = 1; agentNumber < node.Value.Count + 1; agentNumber++)
            {
                columns[agentNumber] = (agentNumber-1).ToString();
                //agentNumber++;
                //node = node.Next;
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
                    //Console.Write("(" + currentMove.x + "," + currentMove.y + ")");
                    columns[i + 1] = currentMove.x + "," + currentMove.y;
                    //Move newMove = new TimedMove((TimedMove)mv);
                    //newList.Add(newMove);
                }
                PrintRow(writeToFile, columns);
                node = node.Next;
            }
            PrintLine(writeToFile);
        }

        static int tableWidth = 200;

        static void PrintLine(bool writeToFile)
        {
            if(!writeToFile)
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
            if(!writeToFile)
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

        private LinkedList<List<Move>> copyLinkedList(LinkedList<List<Move>> toCopy)
        {
            LinkedList<List<Move>> newLinkedList = new LinkedList<List<Move>>();
            LinkedListNode<List<Move>> node = toCopy.First;
            while(node != null)
            {
                List<Move> newList = new List<Move>();
                foreach(Move mv in node.Value)
                {
                    Move newMove = new TimedMove((TimedMove)mv);
                    newList.Add(newMove);
                }
                newLinkedList.AddLast(newList);
                node = node.Next;
            }
            return newLinkedList;
        }

        private bool checkIfWillCollide(LinkedList<List<Move>> lLocations, int current)
        {
            LinkedListNode<List<Move>> node = lLocations.First;
            Move currentMove = new Move();
            for(int i = 0; i < lLocations.Count; i++)
            {
                if (checkIfColide(node.Value))
                {
                    if (toPrint)
                    {
                        Console.WriteLine("***** NOW AT TIME: " + current + "*****");
                        printLinkedList(lLocations, WRITE_LOG);
                    }
                    writeLineToLog("***** NOW AT TIME: " + current + "*****");
                    return true;
                }
                node = node.Next;
            }
            return false;
        }

        private void compareLinkedList(LinkedList<List<Move>> lA, LinkedList<List<Move>> lB)
        {
            LinkedListNode<List<Move>> nodeA = lA.First;
            LinkedListNode<List<Move>> nodeB = lB.First;
            List<Move> lAcurrentList;
            List<Move> lBcurrentList;
            int aSize = nodeA.Value.Count;
            int bSize = nodeB.Value.Count;
            do
            {
                if (nodeA != null)
                    lAcurrentList = nodeA.Value;
                else
                    lAcurrentList = null;
                if (nodeB != null)
                    lBcurrentList = nodeB.Value;
                else
                    lBcurrentList = null;
                for (int i = 0; i < aSize && i < bSize; i++)
                {
                    if(lAcurrentList != null && lBcurrentList != null)
                        printTimedMove((TimedMove)lAcurrentList[i], (TimedMove)lBcurrentList[i], i);
                    else if(lAcurrentList == null)
                        printTimedMove(null, (TimedMove)lBcurrentList[i], i);
                    else
                        printTimedMove((TimedMove)lAcurrentList[i], null, i);
                }
                if (nodeA != lA.Last)
                    nodeA = nodeA.Next;
                else
                    nodeA = null;
                if (nodeB != lB.Last)
                    nodeB = nodeB.Next;
                else
                    nodeB = null;
            } while (nodeA != lA.Last && nodeB != lB.Last);
        }

        private void printTimedMove(TimedMove mv1, TimedMove mv2, int Agent)
        {
            if(mv1 == null)
                Console.WriteLine("Agent: " + Agent + ", At A: null, At B: " + timedMoveString(mv2));
            else if(mv2 == null)
                Console.WriteLine("Agent: " + Agent + ", At A: " + timedMoveString(mv1) + ", At B: null");
            else if(!compareTimedMoves(mv1, mv2))
                Console.WriteLine("Agent: " + Agent + ", At A: " + timedMoveString(mv1) + ", At B: " + timedMoveString(mv2));
        }

        private string timedMoveString(TimedMove mv)
        {
            return "[ (" + mv.x + "," + mv.y + ") - " + mv.time + " ]";
        }

        private bool compareTimedMoves(TimedMove A, TimedMove B)
        {
            if (A.x == B.x && A.y == B.y && A.time == B.time)
                return true;
            return false;
        }

        /// <summary>
        /// calculate new plan
        /// </summary>
        public void replan(List<Move> previousTime, ProblemInstance instance, Run runner, ISolver solver)
        {
            //if(instance == null || instance.m_vAgents == null)
            //    Console.WriteLine("lol");
            // AgentState[] aOldStart = new AgentState[previousTime.Count];
            //int counti = previousTime.Count;
            AgentState[] aOldStart = instance.m_vAgents;
            AgentState[] aStart     = new AgentState[previousTime.Count]; //instance.m_vAgents;
            for (int i = 0; i < previousTime.Count; i++)
            {
                aStart[i] = new AgentState(previousTime[i].x, previousTime[i].y, instance.m_vAgents[i].agent);
            }
            instance.Init(aStart, instance.m_vGrid);
            try
            {
                //runner.SolveGivenProblem(instance);
                run(solver, instance);

                int solverSolutionCost = solver.GetSolutionCost();

                if (solverSolutionCost >= 0) // Solved successfully
                {
                    plan = solver.GetPlan();
                    int planSize = plan.GetSize();
                    int j = 0;
                    for (LinkedListNode<List<Move>> node = plan.GetLocations().First; node != null; node = node.Next)
                    {
                        foreach (Move mv in node.Value)
                        {
                            ((TimedMove)mv).time = j;
                        }
                        j++;
                    }

                    // Validate solution:
                    if (solutionCost == -1) // Record solution cost
                    {
                        solutionCost = solverSolutionCost;
                    }

                    Console.WriteLine("+SUCCESS+ (:");
                }
                else
                {
                    Console.WriteLine("-FAILURE- ):");
                }
            }
            catch(Exception e)
            {
                throw e;
            }
            finally
            {
                instance.Init(aOldStart, instance.m_vGrid);
            }
        }

        private bool planCheck(Run runner)
        {
            LinkedList<List<Move>> locationsAtTimes = runner.plan.GetLocations();
            LinkedListNode<List<Move>> node = locationsAtTimes.First;
            List<Move> locations = node.Value;
            for (int i = 0; i < locationsAtTimes.Count; i++)
            {
                if (((TimedMove)locations[0]).time != i)
                {
                    Console.WriteLine("problem");
                    Console.Read();
                    return false;
                }
                    
                node = node.Next;
                locations = node.Value;
            }
            return true;
        }

        /// <summary>
        /// test adding delay to some agents
        /// </summary>
        public void agentDelayTest(List<int> delayAgents, List<Move> delayMoves, LinkedList<List<Move>> locationsAtTimes, List<Move> currentLocations, int currentTime)
        {
            Dictionary<int, Move> agentToMove = new Dictionary<int, Move>();
            Dictionary<int, Move> previousToMove = new Dictionary<int, Move>();
            //LinkedList<List<Move>> locationsAtTimes = runner.plan.GetLocations();
            List<Move> newTime = null;
            bool first = true;
            int i;
            bool change = false;
            LinkedListNode<List<Move>> node = locationsAtTimes.First;
            List<Move> locations = node.Value;
            while (((TimedMove)locations[0]).time != ((TimedMove)currentLocations[0]).time) //currentLocations != null && 
            {
                node = node.Next;
                locations = node.Value;
            }
            //if (currentLocations != null)
            //    node = node.Next;
            for (i = currentTime; i < locationsAtTimes.Count; i++)
            {
                locations = node.Value;
                if (!first)
                {
                    foreach (int agentIndex in delayAgents)
                    {
                        if (previousToMove[agentIndex].x != locations[agentIndex].x || previousToMove[agentIndex].y != locations[agentIndex].y)
                            change = true;
                        agentToMove[agentIndex] = new TimedMove((TimedMove)locations.ElementAt<Move>(agentIndex));
                        locations[agentIndex] = new TimedMove((TimedMove)previousToMove[agentIndex]);
                        previousToMove[agentIndex] = new TimedMove((TimedMove)agentToMove[agentIndex]);
                        if (i == locationsAtTimes.Count - 1 && change)
                        {
                            newTime = new List<Move>();
                            foreach (Move mv in locations)
                            {
                                newTime.Add(new TimedMove((TimedMove)mv));
                            }
                            newTime[agentIndex] = new TimedMove((TimedMove)agentToMove[agentIndex]);
                            foreach (Move mv in newTime)
                            {
                                ((TimedMove)mv).time = i + 1;
                            }
                        }
                    }
                    change = false;
                    foreach (Move mv in locations)
                    {
                        ((TimedMove)mv).time = i;
                    }
                }
                else
                {
                    foreach (int agentIndex in delayAgents)
                    {
                        previousToMove[agentIndex] = new TimedMove((TimedMove)locations.ElementAt<Move>(agentIndex));
                    }
                }
                first = false;
                node = node.Next;
            }
            if (newTime != null)
            {
                locationsAtTimes.AddLast(newTime);
            }
           // runner.plan.locationsAtTimes = locationsAtTimes;
        }

        /// <summary>
        /// add delay to some agents
        /// </summary>
        public void agentDelay(List<int> delayAgents, List<Move> delayMoves, Run runner, List<Move> currentLocations, int currentTime, LinkedList<List<Move>> currentPlan = null)
        {
            if (currentPlan == null)
                currentPlan = runner.plan.GetLocations();
            Dictionary<int, Move> agentToMove       = new Dictionary<int, Move>();
            Dictionary<int, Move> previousToMove    = new Dictionary<int, Move>();
            LinkedList<List<Move>> locationsAtTimes = currentPlan;


            //test//
            List<Move> nodeLast = locationsAtTimes.Last.Value;
            List<KeyValuePair<int, int>> goalStates = new List<KeyValuePair<int, int>>();
            for(int p = 0; p < nodeLast.Count; p++)
            {
                goalStates.Add(new KeyValuePair<int, int>(nodeLast[p].x, nodeLast[p].y));
            }
            //end-test//

            List<Move> newTime  = null;
            bool first          = true;
            int i;
            bool change         = false;
            LinkedListNode<List<Move>> node = locationsAtTimes.First;
            List<Move> locations            = node.Value;
            while (((TimedMove)locations[0]).time != ((TimedMove)currentLocations[0]).time) //currentLocations != null && 
            {
                node        = node.Next;
                locations   = node.Value;
            }
            //if (currentLocations != null)
            //    node = node.Next;
            List<int> lastMoveChangeAgents = new List<int>();
            for (i = currentTime; i < locationsAtTimes.Count; i++)
            {
                locations = node.Value;
                if (!first)
                {
                    foreach (int agentIndex in delayAgents)
                    {
                        if (previousToMove[agentIndex].x != locations[agentIndex].x || previousToMove[agentIndex].y != locations[agentIndex].y)
                        {
                            change = true;
                            lastMoveChangeAgents.Add(agentIndex);
                        }
                        agentToMove[agentIndex]     = new TimedMove((TimedMove)locations[agentIndex]);
                        locations[agentIndex]       = new TimedMove((TimedMove)previousToMove[agentIndex]);
                        previousToMove[agentIndex]  = new TimedMove((TimedMove)agentToMove[agentIndex]);
                    }
                    if (i == locationsAtTimes.Count - 1 && change)
                    {
                        //Console.WriteLine("new time agent: " + agentIndex);
                        newTime = new List<Move>();
                        foreach (Move mv in locations)
                        {
                            newTime.Add(new TimedMove((TimedMove)mv));
                        }
                        foreach(int agentIndex in lastMoveChangeAgents)
                            newTime[agentIndex] = new TimedMove((TimedMove)agentToMove[agentIndex]);
                        foreach (Move mv in newTime)
                        {
                            ((TimedMove)mv).time = i + 1;
                        }
                    }
                    change = false;
                    foreach (Move mv in locations)
                    {
                        ((TimedMove)mv).time = i;
                    }
                }
                else
                {
                    foreach (int agentIndex in delayAgents)
                    {
                        previousToMove[agentIndex] = new TimedMove((TimedMove)locations[agentIndex]);
                        //previousToMove[agentIndex] = new TimedMove((TimedMove)locations.ElementAt<Move>(agentIndex));
                    }
                }
                first = false;
                node = node.Next;
            }
            if (newTime != null)
            {
                locationsAtTimes.AddLast(newTime);
            }
            //runner.plan.locationsAtTimes = locationsAtTimes;

            //test//
            nodeLast = locationsAtTimes.Last.Value;
            List<KeyValuePair<int, int>> goalStates2 = new List<KeyValuePair<int, int>>();
            for (int p = 0; p < nodeLast.Count; p++)
            {
                goalStates2.Add(new KeyValuePair<int, int>(nodeLast[p].x, nodeLast[p].y));
            }

            for(int p = 0; p < goalStates.Count; p++)
            {
                if(goalStates[p].Key != goalStates2[p].Key || goalStates[p].Value != goalStates2[p].Value)
                {
                    Console.WriteLine("PROBLEM!!!!!!!!!!!!!!!1");
                }
            }
            //end-test//
        }

        /// <summary>
        /// check if paths are colliding
        /// </summary>
        public bool checkIfColide(List<Move> locationsAtTime, Dictionary<Move, double> locationProb = null, Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationCountProb = null)
        {
            Move aMove, bMove;
            for (int i = 0; i < locationsAtTime.Count; i++)
                for (int j = i + 1; j < locationsAtTime.Count; j++)
                {
                    aMove = locationsAtTime[i];
                    bMove = locationsAtTime[j];
                    if (aMove != bMove && ((Move)aMove).IsColliding(bMove) && !checkIfProtectedLocation(aMove, bMove, locationProb, locationCountProb))
                    {
                        Console.WriteLine("***** COLLIDE AT (" + aMove.x + "," + aMove.y + ") Agents: " + i +" , " + j + " AT TIME: " + ((TimedMove)aMove).time + " *****");
                        writeLineToLog("***** COLLIDE AT (" + aMove.x + "," + aMove.y + ") Agents: " + i + " , " + j + " AT TIME: " + ((TimedMove)aMove).time + " *****");
                        return true;
                    }

                }
            return false;
        }


        /// <summary>
        /// check if paths are colliding (k conflicts)
        /// </summary>
        public bool checkIfKColide(List<List<Move>> kLocationsAtTime, int kValue = 0)
        {
            Move aMove, bMove;
            List<Move> locationsAtTime1, locationsAtTime2;
            for (int time1 = 0; time1 < kLocationsAtTime.Count(); time1++)
                for (int time2 = time1; time2 < kLocationsAtTime.Count();  time2++)
                {
                    if (kValue < time2 - time1)
                        continue;
                    locationsAtTime1 = kLocationsAtTime[time1];
                    locationsAtTime2 = kLocationsAtTime[time2];

                    for (int i = 0; i < locationsAtTime1.Count; i++)
                        for (int j = i + 1; j < locationsAtTime2.Count; j++)
                        {
                            aMove = locationsAtTime1[i];
                            bMove = locationsAtTime2[j];
                            if (aMove != bMove && ((Move)aMove).IsColliding(bMove))
                            {
                                Console.WriteLine("***** COLLIDE AT (" + aMove.x + "," + aMove.y + ") Agents: " + i + " , " + j + " AT TIME: " + ((TimedMove)aMove).time + " *****");
                                writeLineToLog("***** COLLIDE AT (" + aMove.x + "," + aMove.y + ") Agents: " + i + " , " + j + " AT TIME: " + ((TimedMove)aMove).time + " *****");
                                return true;
                            }
                        }
                }
            return false;
        }


        public bool checkIfProtectedLocation(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationCountProb = null)
        {
            int numberOfMonitors = 100;
            //return locateAtBestLocations(aMove, bMove, locationProb, numberOfMonitors);

           //return locateAtMostCommonLocations(aMove, bMove, locationProb, numberOfMonitors, locationCountProb);

            //return locateAtLastCommonLocations(aMove, bMove, locationProb, numberOfMonitors, locationCountProb);

            //return locateAtRandomLocations(aMove, bMove, locationProb, numberOfMonitors);

            return locateZeroJunctions(aMove, bMove, locationProb, numberOfMonitors);

        }

        private bool locateZeroJunctions(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, int numberOfMonitors = 5)
        {
            //throw new NotImplementedException();
            return false;
        }

        private bool locateAtBestLocations(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, int numberOfMonitors = 5)
        {
            if (locationProb == null || aMove.x != bMove.x || aMove.y != bMove.y)
                return false;
            Dictionary<Move, double> bestList = new Dictionary<Move, double>();
            for (int i = 0; i < numberOfMonitors && i < locationProb.Count; i++)
            {
                double max = 0;
                Move maxMove = null;
                foreach (Move currentMove in locationProb.Keys)
                {
                    if (!bestList.Keys.Contains(currentMove) && locationProb[currentMove] >= max)
                    {
                        max = locationProb[currentMove];
                        maxMove = currentMove;
                    }
                }
                bestList[maxMove] = max;
            }
            if (bestList.Keys.Contains(new Move(aMove.x, aMove.y, Move.Direction.NO_DIRECTION)))
                return true;
            return false;
        }

        private bool locateAtMostCommonLocations(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, int numberOfMonitors = 5, Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationCountProb = null)
        {
            if (locationProb == null || aMove.x != bMove.x || aMove.y != bMove.y)
                return false;
            Dictionary<Move, double> bestList = new Dictionary<Move, double>();
            for (int i = 0; i < numberOfMonitors && i < locationCountProb.Count; i++)
            {
                double max = 0;
                Move maxMove = null;
                foreach (Move currentMove in locationCountProb.Keys)
                {
                    if (!bestList.Keys.Contains(currentMove) && locationCountProb[currentMove].Count >= max)
                    {
                        max = locationCountProb[currentMove].Count;
                        maxMove = currentMove;
                    }
                }
                bestList[maxMove] = max;
            }
            if (bestList.Keys.Contains(new Move(aMove.x, aMove.y, Move.Direction.NO_DIRECTION)))
                return true;
            return false;
        }

        private bool locateAtLastCommonLocations(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, int numberOfMonitors = 5, Dictionary<Move, List<Tuple<Tuple<int, int>, double>>> locationCountProb = null)
        {
            if (locationProb == null || aMove.x != bMove.x || aMove.y != bMove.y)
                return false;
            Dictionary<Move, double> bestList = new Dictionary<Move, double>();
            for (int i = 0; i < numberOfMonitors && i < locationCountProb.Count; i++)
            {
                double min = Double.MaxValue;
                Move maxMove = null;
                foreach (Move currentMove in locationCountProb.Keys)
                {
                    if (!bestList.Keys.Contains(currentMove) && locationCountProb[currentMove].Count <= min)
                    {
                        min = locationCountProb[currentMove].Count;
                        maxMove = currentMove;
                    }
                }
                bestList[maxMove] = min;
            }
            if (bestList.Keys.Contains(new Move(aMove.x, aMove.y, Move.Direction.NO_DIRECTION)))
                return true;
            return false;
        }

        static Dictionary<Move, double> randomList = null;
        private bool locateAtRandomLocations(Move aMove, Move bMove, Dictionary<Move, double> locationProb = null, int numberOfMonitors = 5)
        {
            if (locationProb == null || aMove.x != bMove.x || aMove.y != bMove.y)
                return false;
            bool first = false;
            if (randomList == null)
            {
                randomList = new Dictionary<Move, double>();
                first = true;
            }
            if (first)
                for (int i = 0; i < numberOfMonitors && i < locationProb.Count; i++)
                {
                    bool done = false;
                    Random rnd = new Random();
                    while(!done)
                    {
                        int randomNumber = rnd.Next(0, locationProb.Count);  
                        Move currentMove = locationProb.ElementAt(randomNumber).Key;
                        if (!randomList.Keys.Contains(currentMove))
                        {
                            randomList.Add(currentMove, locationProb[currentMove]);
                            done = true;
                        }
                    }
                }
            if (randomList.Keys.Contains(new Move(aMove.x, aMove.y, Move.Direction.NO_DIRECTION)))
                return true;
            return false;
        }
    }
}
