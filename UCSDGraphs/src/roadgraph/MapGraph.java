/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import geography.GeographicPoint;
import util.GraphLoader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Collections;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	
	//defining the member variable
	HashMap<GeographicPoint, MapNode> map_d; 
	//TODO: Add your member variables here in WEEK 2
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph(){
		map_d = new HashMap<GeographicPoint, MapNode>();
		// TODO: Implement in this constructor in WEEK 2
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices(){
		//TODO: Implement this method in WEEK 2
		return map_d.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices(){
		//TODO: Implement this method in WEEK 2
		return map_d.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int NumEdges = 0;
		for(MapNode node: map_d.values()){
			NumEdges = NumEdges + node.getedges().size();			
		}
		//TODO: Implement this method in WEEK 2
		return NumEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location){
		if(map_d.containsKey(location) || location == null){
			return false;
		} else{
			MapNode mnd = new MapNode(location);
			map_d.put(location, mnd);
			return true;
		}
		// TODO: Implement this method in WEEK 2
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
		if(!map_d.containsKey(from)||!map_d.containsKey(to)||roadName.equals(null)||roadType.equals(null)||length < 0){
			throw new IllegalArgumentException();
		} else{
			MapEdge med = new MapEdge(from, to, roadName, roadType, length);
			map_d.get(from).addEdge(med);
		}
		//TODO: Implement this method in WEEK 2
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
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		List<GeographicPoint> queue = new ArrayList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, List<GeographicPoint>> parent = new HashMap<GeographicPoint, List<GeographicPoint>>();
		 
		queue.add(start);
		visited.add(start);
		
		while(queue.size() > 0){
			GeographicPoint curr = queue.get(0);
			nodeSearched.accept(curr);
			queue.remove(0);
			if(curr == goal){
				break;
			}
			MapNode curr_node = map_d.get(curr);
			for(GeographicPoint loc: curr_node.getNeighbors()){
				if(!visited.contains(loc)){
					visited.add(loc);
					if(!parent.containsKey(curr)){
						List<GeographicPoint> tmpList = new ArrayList<GeographicPoint>();
						tmpList.add(null);
						parent.put(curr, tmpList);
					}
					parent.get(curr).add(loc);
					queue.add(loc);
				}
			}
			
		}
		
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(goal);
		
//		System.out.println("line175");
		
		GeographicPoint current = goal;
		while(current != start){

			if(parent.size() == 0){				
				return null;
			}

			for(GeographicPoint gp: parent.keySet()){
				if (parent.get(gp).contains(current)){
					path.add(0, gp);
					parent.remove(gp);
					current = gp;
					break;
				}
			}

		}
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

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
		int count = 0;
		
		//Initialize all the mapNodes' distance from goal to infinity
		for(MapNode mn: map_d.values()){
			mn.setdistFstart(Double.POSITIVE_INFINITY);
		}
		Comparator<MapNode> Nodecomparator = new MNComparator();
		
		//A priority queue, visited Set and childMap
		PriorityQueue<MapNode> Pqueue = new PriorityQueue<MapNode>(10, Nodecomparator);
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, List<gpModified>> child = new HashMap<GeographicPoint, List<gpModified>>();
		
		//Obtain the first starting node, set its distance to 0 and add it to the priority queue
		MapNode begin = map_d.get(start);
		begin.setdistFstart(0);
		Pqueue.add(begin);
		
		//As long as priority queue is not empty
		while(Pqueue.size() > 0){
			//Dequeue first node from the priority queue
			GeographicPoint curr = Pqueue.remove().getNodeLoc();
			count++;
			nodeSearched.accept(curr);

			//If the current node has not been visited
			if(!visited.contains(curr)){
				//Add current node to the visited set
				visited.add(curr);

				//If current node is the goal we have reached the goal and break out of the while loop
				if(curr.equals(goal)){
					break;				
				}

				//Performing search for each neighbor of curr
				MapNode curr_node = map_d.get(curr);
				
				//For each edge of the node
				for(MapEdge edgeL: curr_node.getedges()){
					// By pass if the edge end has been visited. If not go in
					if(!visited.contains(edgeL.getEnd())){
						//Update the distance
						if(curr_node.getdistFstart() + edgeL.getDist() < map_d.get(edgeL.getEnd()).getdistFstart()){							
							map_d.get(edgeL.getEnd()).setdistFstart(curr_node.getdistFstart() + edgeL.getDist());
							
							if(!child.containsKey(edgeL.getEnd())){
								List<gpModified> tmpListgp = new ArrayList<gpModified>();
								child.put(edgeL.getEnd(), tmpListgp);
							}

							//Registering curr as the parent of the neighbor
							gpModified newgp = new gpModified(curr, curr_node.getdistFstart() + edgeL.getDist());
							child.get(edgeL.getEnd()).add(newgp);
							
							//Adding the node to the parent queue
							Pqueue.add(map_d.get(edgeL.getEnd()));
						}	
					}
				} //End for			
			} //End if
		} // End while

		List<GeographicPoint> path_s = new ArrayList<GeographicPoint>();
		path_s.add(goal);
		
		GeographicPoint current_s = goal;

		while(!current_s.equals(start)){
			
			List<gpModified> parentList = child.get(current_s);
			Collections.sort(parentList,new gpMcomparator());
			GeographicPoint parentTemp = parentList.get(0).getgp();
			path_s.add(0, parentTemp);
			current_s = parentTemp;
		} //End while
		
		System.out.println("Dijkstra: " + count);
		return path_s;
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

		int count = 0;
		//Initialize all the mapNodes' distance from goal to infinity
		for(MapNode mn: map_d.values()){
			mn.setdistFstart(Double.POSITIVE_INFINITY);
		}
		Comparator<MapNode> Nodecomparator = new MNComparator();
		
		//A priority queue, visited Set and childMap
		PriorityQueue<MapNode> Pqueue = new PriorityQueue<MapNode>(10, Nodecomparator);
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, List<gpModified>> child = new HashMap<GeographicPoint, List<gpModified>>();
		
		//Obtain the first starting node, set its distance to 0 and add it to the priority queue
		MapNode begin = map_d.get(start);
		begin.setdistFstart(0);
		Pqueue.add(begin);
		
		//As long as priority queue is not empty
		while(Pqueue.size() > 0){
			//Dequeue first node from the priority queue
			GeographicPoint curr = Pqueue.remove().getNodeLoc();
			count++;
			nodeSearched.accept(curr);

			//If the current node has not been visited
			if(!visited.contains(curr)){
				//Add current node to the visited set
				visited.add(curr);

				//If current node is the goal we have reached the goal and break out of the while loop
				if(curr.equals(goal)){
					break;				
				}

				//Performing search for each neighbor of curr
				MapNode curr_node = map_d.get(curr);
				
				//For each edge of the node
				for(MapEdge edgeL: curr_node.getedges()){
					// By pass if the edge end has been visited. If not go in
					if(!visited.contains(edgeL.getEnd())){
						//Update the distance
						if(curr_node.getdistFstart() + edgeL.getDist() + edgeL.getEnd().distance(goal) < map_d.get(edgeL.getEnd()).getdistFstart()){							
							map_d.get(edgeL.getEnd()).setdistFstart(curr_node.getdistFstart() + edgeL.getDist() + edgeL.getEnd().distance(goal));
							
							if(!child.containsKey(edgeL.getEnd())){
								List<gpModified> tmpListgp = new ArrayList<gpModified>();
								child.put(edgeL.getEnd(), tmpListgp);
							}

							//Registering curr as the parent of the neighbor
							gpModified newgp = new gpModified(curr, curr_node.getdistFstart() + edgeL.getDist() + edgeL.getEnd().distance(goal));
							child.get(edgeL.getEnd()).add(newgp);
							
							//Adding the node to the parent queue
							Pqueue.add(map_d.get(edgeL.getEnd()));
						}	
					}
				} //End for			
			} //End if
		} // End while

		List<GeographicPoint> path_s = new ArrayList<GeographicPoint>();
		path_s.add(goal);
		
		GeographicPoint current_s = goal;

		while(!current_s.equals(start)){
			
			List<gpModified> parentList = child.get(current_s);
			
			if(!parentList.equals(null) && parentList.size() > 1){
				Collections.sort(parentList,new gpMcomparator());				
			}
			
			GeographicPoint parentTemp = parentList.get(0).getgp();
			path_s.add(0, parentTemp);
			current_s = parentTemp;
		} //End while
		
		System.out.println("Astar: " + count);
		return path_s;
		
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	
	
	public static void main(String[] args)
	{
/*		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(1, 1);
		GeographicPoint end = new GeographicPoint(8, -1);
		
		List<GeographicPoint> route = theMap.aStarSearch(start,end);

		String ret = "";
        for (GeographicPoint point : route) {
            ret += point + "\n";
        }
	
		System.out.println(ret);
*/		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
	}

    public String printRoute(List<GeographicPoint> route) {
        String ret = "";
        for (GeographicPoint point : route) {
            ret += point + "\n";
        }
        return ret;
    }
    
    public Void printHashMap(HashMap<GeographicPoint, List<GeographicPoint>> hm){
    	
    	for (GeographicPoint keyname: hm.keySet()){
    		
    		List<GeographicPoint> thiskey = hm.get(keyname);
            String hashele = "";
            for (GeographicPoint point : thiskey) {
                hashele += point + "\n";

            }
            System.out.println(hashele);
    	}
    	return null;
    }
	
}
