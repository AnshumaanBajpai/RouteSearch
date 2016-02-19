package roadgraph;

import geography.GeographicPoint;


public class gpModified {
	
	private GeographicPoint gp;
	private double DFS;
	
	public gpModified(GeographicPoint geopoint, double disfrst){
		this.gp = geopoint;
		this.DFS = disfrst;
	}
	
	//getter
	public GeographicPoint getgp(){
		return gp;
	}

	public double getDFS(){
		return DFS;
	}

}
