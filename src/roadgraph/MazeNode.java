package roadgraph;

import java.util.HashSet;

import com.sun.xml.internal.ws.api.pipe.ThrowableContainerPropertySet;

import geography.GeographicPoint;

public class MazeNode implements Comparable{
	private final GeographicPoint point;
	private HashSet<Edege> adjEdges;
	
	/** the predicted distance of this node (used in Week 3 algorithms) */
	private double distance;
	
	/** the actual distance of this node from start (used in Week 3 algorithms) */
	private double actualDistance;
	
	//private int numEdges;
	public MazeNode(GeographicPoint point) {		
		this.point = point;
		adjEdges = new HashSet<>();
	}
	public HashSet<Edege> getEdges() {
		return adjEdges;
	}
	
	public String toString()
	{
		String toReturn = "[NODE at location (" + point + ")";
		//toReturn += " intersects streets: ";
//		for (Edege e: adjEdges) {
//			toReturn += e.getName() + ", ";
//		}
		toReturn += "dist: " + distance + "]";
		return toReturn;
	}
	public boolean addEdge(Edege e) {
		if (e != null) {
			adjEdges.add(e);
			return true;
		}
		return false;
	}
	
	public GeographicPoint getLoc() {
		return point;
	}
	
	public int getNumEdges() {
		return adjEdges.size();
	}

	@Override
	public boolean equals(Object obj) {
		// TODO Auto-generated method stub
		if (this == obj) return true;
		if (this.getClass() != obj.getClass()) return false;
		MazeNode that = (MazeNode) obj;
		return that.point.getX() == this.point.getX() && 
				that.point.getY() == this.point.getY();
	}
	
	// get node distance (predicted)
		public double getDistance() {
			return this.distance;
		}
		
		// set node distance (predicted)
		public void setDistance(double distance) {
		    this.distance = distance;
		}

		// get node distance (actual)
		public double getActualDistance() {
			return this.actualDistance;
		}
		
		// set node distance (actual)	
		public void setActualDistance(double actualDistance) {
		    this.actualDistance = actualDistance;
		}
		
	     //Code to implement Comparable
		public int compareTo(Object o) {
			// convert to map node, may throw exception
			MazeNode m = (MazeNode)o; 
			return ((Double)this.getActualDistance()).compareTo((Double) m.getActualDistance());
			//return ((Double)this.getDistance()).compareTo((Double) m.getDistance());
			//return -1;
		}
		
}
