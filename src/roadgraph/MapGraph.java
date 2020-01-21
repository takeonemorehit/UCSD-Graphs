/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;
import java.util.function.Consumer;

import com.sun.org.apache.bcel.internal.classfile.Visitor;

import geography.GeographicPoint;
import javafx.scene.effect.Light.Distant;
import util.GraphLoader;
import week2example.Maze;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint,MazeNode> cells;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		cells = new HashMap<GeographicPoint,MazeNode>();
		// TODO: Implement in this constructor in WEEK 2
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return cells.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return cells.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int count = 0;
		for(MazeNode node : cells.values()) {
			count += node.getNumEdges();
		}
		return count;
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
		// TODO: Implement this method in WEEK 2
		if (location != null) {
			MazeNode n = cells.get(location);
			if (n == null) {
				cells.put(location, new MazeNode(location));
				return true;
			}
			//return cells.put(location,new MazeNode(location));
		}
		return false;
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

		//TODO: Implement this method in WEEK 2
		if (!isLocAdded(from) || !isLocAdded(to)) {
			throw new IllegalArgumentException("caused by node not added");
		}
		if (from == null || to == null || roadName == null
				|| roadType == null || length<0) {
			throw new IllegalArgumentException("argument error caused by illegal range of value");
		}
		MazeNode nodeFrom = cells.get(from);
		MazeNode nodeTo = cells.get(to);
		Edege e = new Edege(nodeFrom, nodeTo, roadName, roadType, length);
		nodeFrom.addEdge(e);
		nodeTo.addEdge(e);
	}
	private boolean isLocAdded(GeographicPoint point) {
		return cells.get(point) != null;
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
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		HashMap<MazeNode, MazeNode> parentTo = new HashMap<>();
		//visited.add(startNode);
		boolean found = found(start, goal, parentTo);
		return constructPath(start, goal, parentTo);
	}
	private List<GeographicPoint> constructPath(GeographicPoint start, 
		     GeographicPoint goal, Map<MazeNode, MazeNode> parentTo) {
		MazeNode startNode = cells.get(start);
		List<GeographicPoint> path = new LinkedList<>();
		MazeNode parent = null;
		MazeNode child = cells.get(goal);
		path.add(child.getLoc());
		while((parent = parentTo.get(child)) != startNode) {
			path.add(parent.getLoc());
			//nodeSearched.accept(child.getLoc());
			child = parent;
			//System.out.println("sn");
		}
		path.add(parent.getLoc());
		
		return path;
	}
	private boolean found(GeographicPoint start, GeographicPoint goal, 
								Map<MazeNode, MazeNode> parentTo) {
		
		MazeNode startNode = cells.get(start);
		MazeNode endNode = cells.get(goal);
		HashSet<MazeNode> visited = new HashSet<>();
		Queue<MazeNode> queue = new LinkedList<>();
		
		queue.add(startNode);
		while (!queue.isEmpty()) {
			MazeNode from = queue.remove();
			visited.add(from);
			for(Edege e : from.getEdges()) {
				MazeNode to = e.getEnd();
				if (!visited.contains(to)) {
					System.out.println(to.getLoc());
					queue.add(to);
					parentTo.put(to, from);
					if (to.getLoc().equals(goal)) {
						return true;
					}
				}
			}
		}
		return false;
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		//HashMap<MazeNode, Double> distTo = new HashMap<>();//dist from start point to cur point 
		for(MazeNode node : cells.values()) {
			//distTo.put(node, Double.POSITIVE_INFINITY);
			node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		HashMap<MazeNode, MazeNode> parentTo = new HashMap<>();
		PriorityQueue<MazeNode> q = new PriorityQueue<MazeNode>();
		MazeNode beg = cells.get(start);
		beg.setActualDistance(0);
		q.add(beg);
		while (!q.isEmpty()) {
			MazeNode cur = q.remove();
			System.out.println(cur);
			if (cur.getLoc().equals(goal)) {
				break;
			}
			relax(cur, q, parentTo);
		}
		System.out.println("finish");
		return constructPath(start, goal, parentTo);
	}
	private void relax(MazeNode cur, PriorityQueue<MazeNode> q, Map<MazeNode, MazeNode> parentTo) {//add pre node's adj nodes to priority
		for(Edege e : cur.getEdges()) {
			//System.out.println(e.length());
			//MazeNode cur = e.getStart();
			MazeNode end = e.getEnd();
			if ((cur.getActualDistance() + e.length()) < end.getActualDistance()) {//not visit
				//MazeNode next = e.getEnd();
				parentTo.put(end, cur);
				end.setActualDistance(e.length()+cur.getActualDistance());
				q.add(end);
			}
		}
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
											 GeographicPoint goal, 
											 Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		for(MazeNode node : cells.values()) {
			//distTo.put(node, Double.POSITIVE_INFINITY);
			node.setDistance(Double.POSITIVE_INFINITY);
			node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		HashMap<MazeNode, MazeNode> parentTo = new HashMap<>();
		PriorityQueue<MazeNode> q = new PriorityQueue<MazeNode>(cells.size(), new DistComparator());
		MazeNode beg = cells.get(start);
		beg.setDistance(0);
		beg.setActualDistance(0);
		q.add(beg);
		while (!q.isEmpty()) {
			MazeNode cur = q.remove();
			System.out.println("cur is -----" + cur);
			//relax(cur, q, parentTo);
			visit(cur, parentTo, q, goal);
			if (cur.getLoc().equals(goal)) {
				break;
			}
//			for(MazeNode n : q) {
//				System.out.println(q);
//			}
//			System.out.println("-------------------");
		}
		System.out.println("finish");
		return constructPath(start, goal, parentTo);
	}
//	寻路步骤
//
//	1，从起点A开始, 把它作为待处理的方格存入一个”开启列表”, 开启列表就是一个等待检查方格的列表. 
//	2，寻找起点A周围可以到达的方格, 将它们放入”开启列表”, 并设置它们的”父方格”为A. 
//	3，从”开启列表”中删除起点 A, 并将起点 A 加入”关闭列表”, “关闭列表”中存放的都是不需要再次检查的方格. 
//	4， 从 “开启列表” 中选择 F 值最低的方格C. 
//	5，把它从 “开启列表” 中删除, 并放到 “关闭列表” 中. 
//	6，检查它所有相邻并且可以到达 (障碍物和 “关闭列表” 的方格都不考虑) 的方格. 如果这些方格还不在 “开启列表” 里的话, 将
//	       它们加入 “开启列表”, 计算这些方格的 G, H 和 F 值各是多少, 并设置它们的 “父方格”为C . 
//	7， 如果某个相邻方格 D 已经在 “开启列表” 里了, 检查如果用新的路径 (就是经过C 的路径) 到达它的话,
//	   G值是否会更低一些, 如果新的G值更低, 那就把它的 “父方格” 改为目前选中的方格 C, 然后重新计算它的 F 、
//	        值和 G 值 (H 值不需要重新计算, 因为对于每个方块, H 值是不变的). 如果新的 G 值比较高, 就说明经过 C 再到达
//	   D 不是一个明智的选择, 因为它需要更远的路, 这时我们什么也不做. 
//	8，然后继续找F值最小的，如此循环下去…

	private void visit(MazeNode cur, Map<MazeNode, MazeNode> parentTo, 
			PriorityQueue<MazeNode> q, GeographicPoint goal) {
		for(Edege e : cur.getEdges()) {
			MazeNode end = e.getEnd();
			double actualDis = cur.getActualDistance() + e.length();//G值
			if (actualDis < end.getActualDistance()) {
				parentTo.put(end, cur);
				//System.out.println("act " + end + "is" + actualDis);
				//System.out.println("straight " + end + "is" + straightDist(end.getLoc(), goal));
				double distance = straightDist(end.getLoc(), goal) + actualDis;//F值	
				//System.out.println("dist " + end + "is" + distance);
				end.setDistance(distance);
				end.setActualDistance(actualDis);
				q.add(end);
			}
		}
	}
	private double straightDist(GeographicPoint p1, GeographicPoint p2) {
//		double dist = Math.sqrt(Math.abs(p1.getX()*p1.getX()-p2.getX()*p2.getX())+
//				Math.abs(p1.getY()*p1.getY()-p2.getY()*p2.getY()));//wrong 
//		double dist = Math.sqrt((p1.getX() - p2.getX())*(p1.getX() - p2.getX())-
//				(p1.getY()-p2.getY())*(p1.getY()-p2.getY()));
		double dist = p1.distance(p2);
		return dist;
	}

	private class DistComparator implements Comparator<MazeNode> {
		@Override
		public int compare(MazeNode n1, MazeNode n2) {
			//System.out.println(n1.getDistance());
			// TODO Auto-generated method stub
			return ((Double)n1.getDistance()).compareTo((Double)n2.getDistance());
		}
		
	}

	
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");
//		
//		GeographicPoint start = new GeographicPoint(1.0, 1.0);
//		GeographicPoint end = new GeographicPoint(8.0, -1.0);
//		List<GeographicPoint> route = theMap.bfs(start,end);
//		
//		System.out.println(route);
		// You can use this method for testing.  
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		System.out.println(route);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println(route2);

	}
	
}
