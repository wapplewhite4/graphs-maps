package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	//private member variables
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	
	//initialize variables with input from constructor
	public MapEdge(GeographicPoint _from, GeographicPoint _to, String _roadName, 
			String _roadType, double _length) {
		
		from = _from;
		to = _to;
		roadName = _roadName;
		roadType = _roadType;
		length = _length;	
	}
	//getters for member variables
	public GeographicPoint getFrom() {
		
		return from;
	}
	
	public GeographicPoint getTo() {
		
		return to;
	}
	
	public String getRoadName() {
		
		return roadName;
	}
	
	public String getRoadType() {
		
		return roadType;
	}
	
	public double getLength() {
		
		return length;
	}
}
