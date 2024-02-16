package fr.uga.pddl4j.examples.asp;

import fr.uga.pddl4j.parser.NamedTypedList;
import fr.uga.pddl4j.parser.Symbol;
import fr.uga.pddl4j.parser.SymbolType;
import fr.uga.pddl4j.parser.TypedSymbol;
import fr.uga.pddl4j.planners.statespace.search.Node;
import fr.uga.pddl4j.problem.*;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.util.BitVector;
import fr.uga.pddl4j.problem.operator.Action;
import java.util.*;
import java.util.stream.Collectors;
import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;
import fr.uga.pddl4j.heuristics.state.StateHeuristic;


public final class EmergencyHeuristic extends RelaxedGraphHeuristic {

    public EmergencyHeuristic(Problem problem) {
        super(problem);
        super.setAdmissible(true);
    }

    public int estimate(State state, Condition goal) {
        System.out.println("Wrong Estimating");
        super.setGoal(goal);
        super.expandRelaxedPlanningGraph(state);
        return super.isGoalReachable() ? 0 : Integer.MAX_VALUE;
    }

    public double estimate(Node node, Condition goal) {
        return this.estimate((State) node, goal);
    }

    public double estimate(State state, Condition goal, Action a, double oldHeuristicValue) {
	
        super.setGoal(goal);
        this.expandRelaxedPlanningGraph(state);
       
        double esteem = oldHeuristicValue;
        if(a.getName().equals("giveContent"))
            esteem--;
        else if (a.getName().equals("fill") || a.getName().equals("charge"))
            esteem-=0.5;
        else if (a.getName().equals("moveVeichle")) 
            esteem += 0.1;
        else if (a.getName().equals("moveAgent"))
            esteem += 0.2;

         return super.isGoalReachable() ? esteem : Integer.MAX_VALUE;

    }
}
