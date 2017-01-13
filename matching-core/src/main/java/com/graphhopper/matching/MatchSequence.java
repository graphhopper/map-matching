/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for 
 *  additional information regarding copyright ownership.
 * 
 *  GraphHopper GmbH licenses this file to you under the Apache License, 
 *  Version 2.0 (the "License"); you may not use this file except in 
 *  compliance with the License. You may obtain a copy of the License at
 * 
 *       http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.matching;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.util.TimeStep;
import com.graphhopper.routing.Path;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;

/**
 * Used to represent a continuous map matching sequence.
 * 
 * @author kodonnell
 */
public class MatchSequence {
    /**
     * Describing the reason for the sequence to have ended:
     *      - UNKNOWN: we don't know why it ended
     *      - LAST_GPX_ENTRY: it was the last in the GPX track, so had to end.
     *      - NO_CANDIDATES: there were no candidates for the *next* step
     *      - NO_POSSIBLE_TRANSITIONS: there were no possible transitions to the *next* step.
     */
    public static enum BreakReason { UNKNOWN, LAST_GPX_ENTRY, NO_CANDIDATES, NO_POSSIBLE_TRANSITIONS };
    /**
     * Sequence type descriptor:
     *      - STATIONARY: where there was only a single step in the sequence. 
     *      - SEQUENCE: a 'normal' sequence, i.e. anything not covered by the above.
     */
    public static enum SequenceType { SEQUENCE, STATIONARY, UNKNOWN };
    /**
     * The break reason for this sequence.
     */
    public final BreakReason lastTimeStepBreakReason;
    /**
     * The sequence type.
     */
    public final SequenceType type;
    /**
     * The matched sequence, as returned from viterbi.computeMostLikelySequence().
     */
    private final List<SequenceState<GPXExtension, GPXEntry, Path>> matchedSequence;
    /**
     * A list of timeSteps (corresponding to matchedSequence). We only use this to get the
     * neighbouring (i.e. clustered) GPX entries.
     */
    public final List<TimeStep> timeSteps;
    /**
     * Time (inclusive, in milliseconds) when first began on this sequence. -1 if not set.
     * TODO: is in inclusive?
     */
    private long fromTime = -1;
    /**
     * Time (inclusive, in milliseconds) when last was on this sequence. -1 if not set.
     * TODO: is in inclusive?
     */
    private long toTime = -1;
    /**
     * List of edges that make up this sequence. Null until computeMatchEdges is called.
     */
    public List<MatchEdge> matchEdges;
    /**
     * List of match entries that are in this sequence. Null until computeMatchEdges is called.
     */
    public List<MatchEntry> matchEntries = null;
    /**
     * The length (meters) of the *matched* path that makes up this sequence.
     */
    private double matchDistance;
    /**
     * The time (milliseconds) to travel the *matched* path that makes up this sequence, assuming
     * one travels at the speed limit and there are no turn costs between sequence connections.
     */
    private long matchDuration;
    /**
     * The cumulative sequential great-line distance between all of the GPX entries, in meters.
     */
    private double gpxEntriesDistance;
    /**
     * The time (milliseconds) between the last and first GPX entry.
     */
    private long gpxEntriesDuration;

    public MatchSequence(List<SequenceState<GPXExtension, GPXEntry, Path>> matchedSequence,
            List<TimeStep> timeSteps,
            BreakReason lastTimeStepBreakReason, SequenceType type) {
        assert matchedSequence.size() >= 1;
        this.matchedSequence = matchedSequence;
        this.lastTimeStepBreakReason = lastTimeStepBreakReason;
        this.type = type;
        this.timeSteps = timeSteps;
        // times - which assume sequence steps are ordered by time:
        // TODO: these may overlap other sequences at the moment
        this.fromTime = matchedSequence.get(0).observation.getTime();
        this.toTime = matchedSequence.get(matchedSequence.size() - 1).observation.getTime();
        
        assert timeSteps.size() == matchedSequence.size();
    }
    
    public void computeGPXStats(DistanceCalc distCalc) {
        GPXEntry lastGPX = null;
        GPXEntry gpx;
        gpxEntriesDistance = 0;
        long minTime = Long.MAX_VALUE;
        long maxTime = Long.MIN_VALUE;
        for (TimeStep timeStep: timeSteps) {
            gpx = timeStep.observation;
            long t = gpx.getTime();
            minTime = t < minTime ? t : minTime;
            maxTime = t > maxTime ? t : maxTime;
            if (lastGPX != null) {
                gpxEntriesDistance += distCalc.calcDist(lastGPX.lat, lastGPX.lon, gpx.lat, gpx.lon);
            }
            lastGPX = gpx;
            // neighbors
            for (GPXEntry neighboringGPX: timeStep.getNeighboringEntries()) {
                neighboringGPX = timeStep.observation;
                t = neighboringGPX.getTime();
                minTime = t < minTime ? t : minTime;
                maxTime = t > maxTime ? t : maxTime;
                gpxEntriesDistance += distCalc.calcDist(lastGPX.lat, lastGPX.lon, neighboringGPX.lat, neighboringGPX.lon);
                lastGPX = neighboringGPX;
            }
        }
        assert minTime < Long.MAX_VALUE && maxTime > Long.MIN_VALUE;
        gpxEntriesDuration = maxTime - minTime;
    }
    
    /**
     * Compute the match edges, including associating GPX entries (inc. ignored ones) to match edges.
     * 
     * @param virtualEdgesMap a map to convert virtual edges to real one
     * @param nodeCount number of nodes in routing graph (so we can detect virtual ones)
     */
    public void computeMatchEdges(Map<String, EdgeIteratorState> virtualEdgesMap, int nodeCount) {
        
        // TODO: remove gpx extensions and just add time at start/end of edge.
        matchDistance = 0.0;
        matchDuration = 0;
        matchEntries = new ArrayList<MatchEntry>();
        matchEdges = new ArrayList<MatchEdge>();

        if (type != SequenceType.SEQUENCE)
            return;
            // TODO
        
        // add the rest:
        EdgeIteratorState lastEdgeAdded = null;
        long realFromTime = fromTime;
        long lastEdgeEndTime = fromTime;
        for (int j = 1; j < matchedSequence.size(); j++) {
            SequenceState<GPXExtension, GPXEntry, Path> matchStep = matchedSequence.get(j);
            Path path = matchedSequence.get(j).transitionDescriptor;

            double pathDistance = path.getDistance();
            long realEndTime = matchStep.observation.getTime();
            double realTimePerPathMeter = (double) (realEndTime - realFromTime) / (double) pathDistance;
            
            matchDuration += path.getTime();
            matchDistance += pathDistance;

            // loop through edges for this path, and add them
            List<EdgeIteratorState> edges = path.calcEdges();
            EdgeIteratorState edgeIteratorState = null;
            EdgeIteratorState directedRealEdge = null;
            int nEdges = edges.size();
            for (int edgeIdx = 0; edgeIdx < nEdges; edgeIdx++) {
                edgeIteratorState = edges.get(edgeIdx);
                // get time:
                long edgeEndTime = edgeIdx == (nEdges - 1) ? realEndTime : (long) (realFromTime + edgeIteratorState.getDistance() * realTimePerPathMeter);           
                directedRealEdge = resolveToRealEdge(virtualEdgesMap, edgeIteratorState, nodeCount);
                if (lastEdgeAdded == null || !equalEdges(directedRealEdge, lastEdgeAdded)) {
                    if (lastEdgeAdded != null) {
                        matchEdges.add(new MatchEdge(directedRealEdge, lastEdgeEndTime, edgeEndTime));
                        lastEdgeEndTime = edgeEndTime;
                        lastEdgeAdded = directedRealEdge;
                    }
                }
            }
            
            // add first match entry:
            if (j == 1) {
                EdgeIteratorState firstDirectedRealEdge = resolveToRealEdge(virtualEdgesMap, edges.get(0), nodeCount);
                matchEntries.add(new MatchEntry(0, matchedSequence.get(0), timeSteps.get(0), firstDirectedRealEdge, 0, 0));
            }
            
            // add this matchEntry:
            // TODO: handle edgeIteratorState == null or directedRealEdge == null
            matchEntries.add(new MatchEntry(j, matchStep, timeSteps.get(j), directedRealEdge, matchEdges.size() - 1, edgeIteratorState.getDistance()));
            
            // TODO: figure out overlaps ...
            realFromTime = realEndTime;
        }
        
        // we should have some edge matches:
//        if (edgeMatches.isEmpty())
        
    }

    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }
    
    private EdgeIteratorState resolveToRealEdge(Map<String, EdgeIteratorState> virtualEdgesMap, EdgeIteratorState edgeIteratorState, int nodeCount) {
        EdgeIteratorState directedRealEdge;
        if (isVirtualNode(edgeIteratorState.getBaseNode(), nodeCount) || isVirtualNode(edgeIteratorState.getAdjNode(), nodeCount)) {
            directedRealEdge = virtualEdgesMap.get(virtualEdgesMapKey(edgeIteratorState));
        } else {
            directedRealEdge = edgeIteratorState;
        }
        if (directedRealEdge == null) {
            throw new RuntimeException("Did not find real edge for " + edgeIteratorState.getEdge());
        }
        return directedRealEdge;
    }

    private boolean isVirtualNode(int node, int nodeCount) {
        return node >= nodeCount;
    }

    private String virtualEdgesMapKey(EdgeIteratorState iter) {
        return iter.getBaseNode() + "-" + iter.getEdge() + "-" + iter.getAdjNode();
    }

    public long getFromTime() {
        return fromTime;
    }

    public long getToTime() {
        return toTime;
    }
    
    public double getMatchDistance() {
        return matchDistance;
    }

    public long getMatchDuration() {
        return matchDuration;
    }
    
    public double getGPXEntriesDistance() {
        return gpxEntriesDistance;
    }

    public long getGPXEntriesDuration() {
        return gpxEntriesDuration;
    }
}
