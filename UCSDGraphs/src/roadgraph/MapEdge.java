package roadgraph;

//importing the required packages
import geography.GeographicPoint;


public class MapEdge {
	
	//member variable for MapEdge
	private GeographicPoint start;
	private GeographicPoint end;
	private String edgeName;
	private String edgeType;
	private double distance;
	
	//Constructor for MapEdge object
/*	public MapEdge(){
		//Setting the instance variable
		start = null;
		end = null;
		edgeName = null;
		edgeType = null;
		distance = 0;
	}
*/	
	public MapEdge(GeographicPoint start_loc, GeographicPoint end_loc, String Str_name, String Str_type, double dist){		
		//Setting the instance variable
		this.start = start_loc;
		this.end = end_loc;
		this.edgeName = Str_name;
		this.distance = dist;
		this.edgeType = Str_type;
	}
	
	//Setters
	public void setStart(GeographicPoint start_loc){
		this.start = start_loc;
	}
	
	public void setEnd(GeographicPoint end_loc){
		this.end = end_loc;
	}

	public void setEdgeName(String Str_name){
		this.edgeName = Str_name;
	}

	public void setEdgeType(String Str_type){
		this.edgeType = Str_type;
	}
	public void setDist(double dist){
		this.distance = dist;
	}
	
	//Getters
	public GeographicPoint getStart(){
		return start;
	}
	
	public GeographicPoint getEnd(){
		return end;
	}

	public String getEdgeName(){
		return edgeName;
	}

	public String getEdgeType(){
		return edgeType;
	}
	public double getDist(){
		return distance;
	}
}
