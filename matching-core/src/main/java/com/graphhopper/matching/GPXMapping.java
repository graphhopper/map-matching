package com.graphhopper.matching;

import com.graphhopper.util.GPXEntry;

public class GPXMapping {
    public final GPXEntry originalGPXEntry;
    public final int matchEdgeIdx;
    public final boolean isNeighbor;
    public final int neighborIdx;
    public final MatchEntry matchEntry;
    public GPXMapping(GPXEntry originalGPXEntry, MatchEntry matchEntry, int matchEdgeIdx, boolean isNeighbor, int neighborIdx) {
        this.originalGPXEntry = originalGPXEntry;
        this.matchEdgeIdx = matchEdgeIdx;
        this.isNeighbor = isNeighbor;
        this.neighborIdx = neighborIdx;
        this.matchEntry = matchEntry;
    }
}
