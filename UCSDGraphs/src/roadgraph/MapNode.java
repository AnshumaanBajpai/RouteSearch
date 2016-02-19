package roadgraph;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.HashSet;
import geography.GeographicPoint;
import java.util.PriorityQueue;

public class MapNode {
	
	//member variable for MapNode
	private GeographicPoint nodeLoc;
	private PriorityQueue<Double> distFstart = new PriorityQueue<Double>();
	private List<MapEdge> edges;
	
	//Constructors for MapNode object
	public MapNode(GeographicPoint node_Loc){
		this.nodeLoc = node_Loc;
		this.edges = new ArrayList<MapEdge>();
	}

/*	public int compare(MapNode node1, MapNode node2){
		return Double.compare(node1.getdistFstart(), node2.getdistFstart());
	}
*/	//Setter
	public void setNodeLoc(GeographicPoint node_Loc){
		this.nodeLoc = node_Loc;
	}
	public void setdistFstart(double dfs){
		
		this.distFstart.add(dfs);
	}
	
	//Getter
	public GeographicPoint getNodeLoc(){
		return nodeLoc;
	}
	
	public List<MapEdge> getedges(){
		return edges;
	}

	public double getdistFstart(){
		return distFstart.peek();
	}

	//add an edge to the node
	public void addEdge(MapEdge mE){
		edges.add(mE);
	}

	public Set<GeographicPoint> getNeighbors(){
		Set<GeographicPoint> neighbors = new HashSet<GeographicPoint>();
		for(MapEdge edge: edges){
			neighbors.add(edge.getEnd());
		}
		//TODO: Implement this method in WEEK 2
		return neighbors;
	}

	
}
