package com.graphhopper.matching;

import java.util.List;

import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.util.TimeStep;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.shapes.GHPoint3D;

public class MatchEntry {
    public final int sequenceIdx;
    public final GPXEntry gpxEntry;
    public final List<GPXEntry> neighboringGpxEntries;
    public final GHPoint3D point;
    public final EdgeIteratorState directedRealEdge;
    public final int sequenceMatchEdgeIdx;
    public final double distanceAlongRealEdge;
    public MatchEntry(int sequenceIdx, SequenceState<GPXExtension, GPXEntry, Path> matchStep, TimeStep timeStep, EdgeIteratorState directedRealEdge, int sequenceMatchEdgeIdx, double distanceAlongRealEdge) {
        this.sequenceIdx = sequenceIdx;
        this.gpxEntry = matchStep.observation;
        this.point = matchStep.state.getQueryResult().getSnappedPoint();
        this.sequenceMatchEdgeIdx = sequenceMatchEdgeIdx;
        this.directedRealEdge = directedRealEdge;
        this.distanceAlongRealEdge = distanceAlongRealEdge;
        this.neighboringGpxEntries = timeStep.getNeighboringEntries();
    }

}
