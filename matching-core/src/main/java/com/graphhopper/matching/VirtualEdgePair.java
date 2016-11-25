package com.graphhopper.matching;

import java.util.ArrayList;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.VirtualEdgeIteratorState;
import com.graphhopper.util.EdgeIteratorState;

public class VirtualEdgePair {
	
	public ArrayList<VirtualEdgeIteratorState> pair = new ArrayList<VirtualEdgeIteratorState>();
	
	public VirtualEdgePair(EdgeIteratorState iter, QueryGraph queryGraph) {
		pair.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
		pair.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getBaseNode()));
	}
	
	public void setUnfavored(boolean favor) {
		pair.get(0).setUnfavored(favor);
		pair.get(1).setUnfavored(favor);
	}
}
