/*
* Copyright (c) 2021 by Damien Pellier <Damien.Pellier@imag.fr>.
*
* This file is part of PDDL4J library.
*
* PDDL4J is free software: you can redistribute it and/or modify * it under the terms of the GNU General Public License
* as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
* PDDL4J is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License * along with PDDL4J.  If not,
* see <http://www.gnu.org/licenses/>
*/

package fr.uga.pddl4j.examples.asp;
import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;
import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

@CommandLine.Command(name = "EPP",
    version = "EPP 1.0",
    description = "Solves a specified planning problem using A* search strategy. The heuristic function is a custom.",
    sortOptions = false,
    mixinStandardHelpOptions = true,
    headerHeading = "Usage:%n",
    synopsisHeading = "%n",
    descriptionHeading = "%nDescription:%n%n",
    parameterListHeading = "%nParameters:%n",
    optionListHeading = "%nOptions:%n")

public class EPP extends AbstractPlanner {
    //The class logger
    private static final Logger LOGGER = LogManager.getLogger(EPP.class.getName());

    /**
    * The HEURISTIC property used for planner configuration.
    */
   public static final String HEURISTIC_SETTING = "HEURISTIC";

   /**
    * The default value of the HEURISTIC property used for planner configuration.
    */
   public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;

   /**
    * The WEIGHT_HEURISTIC property used for planner configuration.
    */
   public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";

   /**
    * The default value of the WEIGHT_HEURISTIC property used for planner configuration.
    */
   public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;

   /**
    * The weight of the heuristic.
    */
   private double heuristicWeight;

   /**
    * The name of the heuristic used by the planner.
    */
   private StateHeuristic.Name heuristic;

   /**
    * Creates a new A* search planner with the default configuration.
    */
   public EPP() {
       this(EPP.getDefaultConfiguration());
   }

        /**
      * Creates a new A* search planner with a specified configuration.
      *
      * @param configuration the configuration of the planner.
      */
      public EPP(final PlannerConfiguration configuration) {
        super();
        this.setConfiguration(configuration);
    }

    /**
      * Set the name of heuristic used by the planner to the solve a planning problem.
      *
      * @param heuristic the name of the heuristic.
      */
      @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
      description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
          + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }
    
    /**
      * Returns the name of the heuristic used by the planner to solve a planning problem.
      *
      * @return the name of the heuristic used by the planner to solve a planning problem.
      */
      public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

     /**
      * Checks the planner configuration and returns if the configuration is valid.
      * A configuration is valid if (1) the domain and the problem files exist and
      * can be read, (2) the timeout is greater than 0, (3) the weight of the
      * heuristic is greater than 0 and (4) the heuristic is a not null.
      *
      * @return <code>true</code> if the configuration is valid <code>false</code> otherwise.
      */
      public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
                && this.getHeuristicWeight() > 0.0
                && this.getHeuristic() != null;
    }

    /**
      * Sets the configuration of the planner. If a planner setting is not defined in
      * the specified configuration, the setting is initialized with its default value.
      *
      * @param configuration the configuration to set.
      */
      @Override
      public void setConfiguration(final PlannerConfiguration configuration) {
          super.setConfiguration(configuration);
          if (configuration.getProperty(EPP.WEIGHT_HEURISTIC_SETTING) == null) {
              this.setHeuristicWeight(EPP.DEFAULT_WEIGHT_HEURISTIC);
          } else {
              this.setHeuristicWeight(Double.parseDouble(configuration.getProperty(
                      EPP.WEIGHT_HEURISTIC_SETTING)));
          }
          if (configuration.getProperty(EPP.HEURISTIC_SETTING) == null) {
              this.setHeuristic(EPP.DEFAULT_HEURISTIC);
          } else {
              this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(
                      EPP.HEURISTIC_SETTING)));
          }
      }
 
      /**
       * This method return the default arguments of the planner.
       *
       * @return the default arguments of the planner.
       * @see PlannerConfiguration
       */
      public static PlannerConfiguration getDefaultConfiguration() {
          PlannerConfiguration config = Planner.getDefaultConfiguration();
          config.setProperty(EPP.HEURISTIC_SETTING, EPP.DEFAULT_HEURISTIC.toString());
          config.setProperty(EPP.WEIGHT_HEURISTIC_SETTING,
                  Double.toString(EPP.DEFAULT_WEIGHT_HEURISTIC));
          return config;
      }
 

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
         paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
     public void setHeuristicWeight(final double weight) {
         if (weight <= 0) {
             throw new IllegalArgumentException("Weight <= 0");
         }
         this.heuristicWeight = weight;
    }

    //Instantiates the planning problem from a parsed problem.
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
      * Returns the configuration of the planner.
      *
      * @return the configuration of the planner.
      */
      @Override
      public PlannerConfiguration getConfiguration() {
          final PlannerConfiguration config = super.getConfiguration();
          config.setProperty(EPP.HEURISTIC_SETTING, this.getHeuristic().toString());
          config.setProperty(EPP.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
          return config;
      }


    //Search a solution plan to a specified domain and problem using A*.
    //@param problem the problem to solve.
    //@return the plan found or null if no plan was found.
    @Override
    public Plan solve(final Problem problem) throws ProblemNotSupportedException {
        LOGGER.info("* Starting search \n");
        Plan plan = null;
        
        try {
            final long start = System.currentTimeMillis();
            plan = this.customAstar(problem);
            final long end = System.currentTimeMillis();
            if (plan != null) {
                LOGGER.info("* Search succeeded\n");
                //this.getStatistics().setTimeToSearch(end-start);
            } else {
                LOGGER.info("* Search failed\n");
            }
            this.getStatistics().setTimeToSearch(end-start);
        } catch (ProblemNotSupportedException e) {
            LOGGER.error("Not supported problem");
            e.printStackTrace();
        }
        finally {
            // Return the plan found or null if the search fails.
            return plan;
        }
    }
    
    /**
      * Search a solution plan for a planning problem using an A* search strategy.
      *
      * @param problem the problem to solve.
      * @return a plan solution for the problem or null if there is no solution
      * @throws ProblemNotSupportedException if the problem to solve is not supported by the planner.
      */
    public Plan customAstar( Problem problem) throws ProblemNotSupportedException {
        //Requirements
        if (!this.isSupported(problem)) { throw new ProblemNotSupportedException("Cannot solve the problem"); }
    
        //Euristic
        final EmergencyHeuristic heuristic = new EmergencyHeuristic(problem);
    
        //Initial state
        final State init = new State(problem.getInitialState());
        //Nodes list
        final Set<Node> exploredNodes = new HashSet<>();
        //Priority queue of visited node
        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> toExplore = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double w1 = weight*n1.getHeuristic() + n1.getCost();
                double w2 = weight*n2.getHeuristic() + n2.getCost();
                return Double.compare(w1,w2);
            }
        });
        
        //Root
        final Node root = new Node(init, null, -1, 0, (problem.getGoal().getNegativeFluents().stream().count() + problem.getGoal().getPositiveFluents().stream().count())*2);
        toExplore.add(root);
        Plan plan = null;
        
        final int timeout = 600 * 1000;
        long time = 0;
        final long begin = System.currentTimeMillis();	
        //Search beginning
        while (!toExplore.isEmpty() && plan == null && time < timeout) {

            final Node current = toExplore.poll();
            exploredNodes.add(current);
            
            // Check if goal is reached
            if(current.satisfy(problem.getGoal())) {
                long m1 = Runtime.getRuntime().totalMemory();
                long m2 = Runtime.getRuntime().freeMemory();
                this.getStatistics().setMemoryUsedToSearch(m1 - m2);
                return this.returnPlan(problem, current);
            }
            else {
                //Apply actions which belong to the node
                for(int i=0; i<problem.getActions().size(); i++) {
                    Action a = problem.getActions().get(i);
                    // Check if action can be applied to the current node
                    if(a.isApplicable(current)) {
                        Node next = new Node(current);
                        // Apply effects
                        final List<ConditionalEffect> effects = a.getConditionalEffects();
                        for (ConditionalEffect condEffect : effects) {
                            if(current.satisfy(condEffect.getCondition())) {
                                next.apply(condEffect.getEffect());
                            }
                        }
                        //Setting of informations for the child node
                        final double c = current.getCost() +1;
                        if (!exploredNodes.contains(next)) {
                            next.setCost(c);
                            next.setParent(current);
                            next.setAction(i);
                            next.setHeuristic(heuristic.estimate(next, problem.getGoal(), a, current.getHeuristic()));
                            toExplore.add(next);
                        }
                    }
                }
            }
            // Compute the searching time
            time = System.currentTimeMillis() - begin;
        }
        return plan;
    }

   private Plan returnPlan(final Problem problem, final Node node) {
       Node n = node;
       final Plan plan = new SequentialPlan();
       while (n.getAction() != -1) {
           final Action a = problem.getActions().get(n.getAction());
           plan.add(0,a);
           n = n.getParent();
       }
       return plan;
   } 

    //The main method
    public static void main(String[] args) {
        try {
            final EPP planner = new EPP();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }

    }

    //Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
    @Override
    public boolean isSupported(Problem problem) {
        return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
            || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
            || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
            || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
            || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
            || problem.getRequirements().contains(RequireKey.FLUENTS)
            || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
            || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
            || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
            || problem.getRequirements().contains(RequireKey.PREFERENCES)
            || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
            || problem.getRequirements().contains(RequireKey.HIERARCHY))
            ? false : true;
    }


}