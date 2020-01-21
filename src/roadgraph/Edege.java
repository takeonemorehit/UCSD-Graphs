package roadgraph;

import geography.GeographicPoint;

public class Edege implements Comparable<Edege> {
	private final MazeNode start;
	private final MazeNode end;
	private final String roadName;
	private final String roadType;
	private final double length;
	public Edege(MazeNode start, MazeNode end, String roadName, 
			String roadType, double length) {
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	public MazeNode getStart() {
		return start;
	}
	public MazeNode getEnd() {
		return end;
	}
	
	public String getName() {
		return roadName;
	}
	public double length() {
		return length;
	}
	@Override
	public int compareTo(Edege o) {
		return (int) (this.length - o.length);
	}
 }
