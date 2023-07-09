package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	 
	private GeographicPoint location;
	private List<MapEdge> edges;
	private double currDistance;
	private double distanceGuess;
	
	//initialize location with constructor
	public MapNode(GeographicPoint _location) {
		location = _location;
		edges = new ArrayList<MapEdge>();
		currDistance = 0;
		distanceGuess = 0;
	}

	//getters for location, list of edges, and current distance
	public GeographicPoint getLocation() {
		
		return location;
	}
	
	public List<MapEdge> getEdges(){
		
		return edges;
	}
	
	public double getDistance() {
		return currDistance;
	}
	public double getGuess() {
		return distanceGuess;
	}
	//setter for adding a map edge to the list of edges
	public void addEdge(MapEdge _edge) {
		edges.add(_edge);
	}
	
	//setter for current distance
	public void updateDistance(double dist) {
		currDistance = dist;
	}
	
	public void updateGuess(double guess) {
		distanceGuess = guess;
	}
	//implementing logic for comparable interface
	@Override
	public int compareTo(MapNode n) {
		
		if (this.getDistance() + this.getGuess() == n.getDistance() + n.getGuess()) {
			
			return 0;
		}
		if (this.getDistance() + this.getGuess() < n.getDistance() + n.getGuess()) {
			
			return -1;
		} else {
			
			return 1;
		}
		
		
	}
}
