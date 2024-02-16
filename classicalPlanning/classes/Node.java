package fr.uga.pddl4j.examples.asp;
import fr.uga.pddl4j.problem.State;

//This class implements a node of the tree search.

public final class Node extends State {

//The parent node of this node.
    private Node parent;

//The action apply to reach this node.
    private int action;

//The cost to reach this node from the root node.
    private double cost;

//The estimated distance to the goal from this node.
    private double heuristic;

//The depth of the node.
    private int depth;

//Creates a new node from a specified state.
    public Node(State state) {
        super(state);
    }

//Creates a new node with a specified state, parent node, operator,cost and heuristic value.
    public Node(State state, Node parent, int action, double cost, double heuristic) {
        super(state);
        this.parent = parent;
        this.action = action;
        this.cost = cost;
        this.heuristic = heuristic;
        this.depth = -1;
    }

//Creates a new node with a specified state, parent node, operator, cost,depth and heuristic value.
    public Node(State state, Node parent, int action, double cost, int depth, double heuristic) {
        super(state);
        this.parent = parent;
        this.action = action;
        this.cost = cost;
        this.depth = depth;
        this.heuristic = heuristic;
    }

//Returns the action applied to reach the node.
    public final int getAction() {
        return this.action;
    }

//Sets the action applied to reach the node.
    public final void setAction(final int action) {
        this.action = action;
    }

//Returns the parent node of the node.
    public final Node getParent() {
        return parent;
    }

//Sets the parent node of the node.
    public final void setParent(Node parent) {
        this.parent = parent;
    }

//Returns the cost to reach the node from the root node.
    public final double getCost() {
        return cost;
    }

//Sets the cost needed to reach the node from the root node.
    public final void setCost(double cost) {
        this.cost = cost;
    }

//Returns the estimated distance to the goal from the node.
    public final double getHeuristic() {
        return heuristic;
    }

//Sets the estimated distance to the goal from the node.
    public final void setHeuristic(double estimates) {
        this.heuristic = estimates;
    }

//Returns the depth of this node.
    public int getDepth() {
        return this.depth;
    }

//Set the depth of this node.
    public void setDepth(final int depth) {
        this.depth = depth;
    }

//Returns the value of the heuristic function, i.e.,<code>this.node.getCost() + this.node.getHeuristic()</code>.
    public final double getValueF(double weight) {
        return weight * this.heuristic + this.cost;
    }
}