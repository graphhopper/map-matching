package com.graphhopper.matching;

import java.util.ArrayList;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.VirtualEdgeIteratorState;
import com.graphhopper.util.EdgeIterator;

/*
 * At a virtual node we get four virtual edges:
 * 	
 *        GPX
 *         |
 *         |
 * 	  v1   |   v2
 * 	--->--- --->----
 *         N 
 *  ---<--- ---<----
 *    v3       v4
 *    
 * This is to represent this quadruple. We call it 'directed' in the sense that we allow only
 * only of the edges to be used (by unfavoring the other three). 
 */
public class DirectedVirtualEdgeQuadruple {
	
	public ArrayList<VirtualEdgeIteratorState> quad = new ArrayList<VirtualEdgeIteratorState>();
	public int favoured;
	
	public DirectedVirtualEdgeQuadruple(int virtualNode, QueryGraph queryGraph, int favoured) {
		this.favoured = favoured;
		EdgeIterator iter = queryGraph.createEdgeExplorer().setBaseNode(virtualNode);
		while (iter.next()) {
			quad.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
		}
		assert quad.size() == 2;
		// add reverse
		for (int i = 0; i < 2; i++) {
			VirtualEdgeIteratorState e = quad.get(i);
			// TODO: is this the correct way to reverse? Or the following?
			// quad.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(e.getEdge(), e.getBaseNode()));
			quad.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(e.getEdge(), e.getAdjNode()));
		}
	}
	
	public void setFavouringOfUnfavored(boolean favor) {
		for (int i = 0; i < 4; i++) {
			if (i != favoured) {
				quad.get(i).setUnfavored(favor);		
			}
		}
	}
}
