/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	
	//private hashmap for storing nodes
	private HashMap<GeographicPoint, MapNode> nodes; 
	private List<MapEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		
		//initialize nodes hashmap and edges list
		nodes = new HashMap<GeographicPoint, MapNode>();
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		//return the size of the hashmap containing all the nodes
		return nodes.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		//return the keyset of the nodes hashmap
		return nodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		
		//check to see if location is null or if node is already in graph. if so, return false
		if (location == null) {
			return false;
		}
		if (nodes.containsKey(location)) {
			return false;
		}
		//create new mapnode and add it to the hashmap
		MapNode node = new MapNode(location);
		nodes.put(location, node);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		//check to see if any args are null (except for double which cannot be null)
		if (from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException();
		}
		//check to see if length < 0
		if (length < 0) {
			throw new IllegalArgumentException();
		}
		//check to see if graph already contains both geographic points. if not, throw illegal argument exception
		if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
			throw new IllegalArgumentException();
		}
		//create new mapedge and add it to list of edges
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(edge);
		//add edge to edgelist of start node
		nodes.get(from).addEdge(edge);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//initializing queue, hashset, and hashmap 
		Queue<MapNode> queue = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parents = new HashMap<MapNode, MapNode>();
		
		//implementing bfs algorithm
		//adding start node to queue
		queue.add(nodes.get(start));
		MapNode curr;
		//starting while loop
		while (!queue.isEmpty()) {
			//get curr from queue
			curr = queue.remove();
			//if curr equals the goal node, return list of locations of parents
			if (curr.equals(nodes.get(goal))) {
				return reconstructPath(start, goal, parents);
			}
			nodeSearched.accept(curr.getLocation());
			//n will be the neighbor of curr
			MapNode n;
			//for each of currs neighbors, n
			for (MapEdge e : curr.getEdges()) {
				n = nodes.get(e.getTo());
				//check to see if n is in visited set
				if (!visited.contains(n)) {
					//if so, add n to visited, add curr as n's parent, and add n to the queue
					visited.add(n);
					parents.put(n, curr);
					queue.add(n);
				}
			}
		}
		return null;
	}
	
	//helper method to reconstruct the path using the parents hashmap
	private List<GeographicPoint> reconstructPath(GeographicPoint start, GeographicPoint goal, HashMap<MapNode, MapNode> parents) {
	    List<GeographicPoint> path = new ArrayList<GeographicPoint>();
	    MapNode curr = nodes.get(goal);
	    path.add(0, curr.getLocation());
	    while (!curr.equals(nodes.get(start))) {
	        curr = parents.get(curr);
	        path.add(0, curr.getLocation());
	    }
	    return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//Initializing data structures
		Queue<MapNode> priorityQ = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parents = new HashMap<MapNode, MapNode>();
		
		//setting all distances to infinity
		for (GeographicPoint p: nodes.keySet()) {
			nodes.get(p).updateDistance(Double.POSITIVE_INFINITY);
			nodes.get(p).updateGuess(0);
		}
		MapNode st = nodes.get(start);
		st.updateDistance(0);
		priorityQ.add(st);
		MapNode curr;
		int nodesVisited = 0;
		while (!priorityQ.isEmpty()) {
			curr = priorityQ.poll();
			nodesVisited++;
			if (!visited.contains(curr)) {
				visited.add(curr);
				
				if (curr.equals(nodes.get(goal))) {
					System.out.println("Dijkstra: " + nodesVisited);
					return reconstructPath(start, goal, parents);
				}
				
				MapNode n;
				for (MapEdge e : curr.getEdges()) {
					n = nodes.get(e.getTo());
					
					if ((curr.getDistance() + e.getLength()) < n.getDistance()) {
						n.updateDistance(curr.getDistance() + e.getLength());
						parents.put(n, curr);
						priorityQ.add(n);
					}
					
					}
			}
		}
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		//Initializing data structures
				Queue<MapNode> priorityQ = new PriorityQueue<MapNode>();
				HashSet<MapNode> visited = new HashSet<MapNode>();
				HashMap<MapNode, MapNode> parents = new HashMap<MapNode, MapNode>();
				
				//setting all distances to infinity
				for (GeographicPoint p: nodes.keySet()) {
					nodes.get(p).updateDistance(Double.POSITIVE_INFINITY);
					nodes.get(p).updateGuess(Double.POSITIVE_INFINITY);
				}
				MapNode st = nodes.get(start);
				st.updateDistance(0);
				st.updateGuess(0);
				priorityQ.add(st);
				MapNode curr;
				int nodesVisited = 0;
				while (!priorityQ.isEmpty()) {
					curr = priorityQ.poll();
					nodesVisited++;
					if (!visited.contains(curr)) {
						visited.add(curr);
						
						if (curr.equals(nodes.get(goal))) {
							System.out.println("A*: " + nodesVisited);
							return reconstructPath(start, goal, parents);
						}
						
						MapNode n;
						for (MapEdge e : curr.getEdges()) {
							n = nodes.get(e.getTo());
							
							if ((curr.getDistance() + curr.getGuess() + e.getLength()) < (n.getDistance() + n.getGuess())) {
								n.updateDistance(curr.getDistance() + e.getLength());
								n.updateGuess(n.getLocation().distance(goal));
								parents.put(n, curr);
								priorityQ.add(n);
							}
							
							}
					}
				}
		
		return null;
	}
	

	
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		*/
		/*  MapGraph simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
			GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
			
			System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
			List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
			List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
			
			
			MapGraph testMap = new MapGraph();
			GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
			// A very simple test using real data
			testStart = new GeographicPoint(32.869423, -117.220917);
			testEnd = new GeographicPoint(32.869255, -117.216927);
			System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			*/
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
