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
import com.graphhopper.routing.Path;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

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
    public static enum ViterbiBreakReason { UNKNOWN, LAST_GPX_ENTRY, NO_CANDIDATES, NO_POSSIBLE_TRANSITIONS };
    /**
     * Sequence type descriptor:
     *      - STATIONARY: where there was only a single step in the sequence. 
     *      - SEQUENCE: a 'normal' sequence, i.e. anything not covered by the above.
     */
    public static enum SequenceType { SEQUENCE, STATIONARY, UNKNOWN };
    /**
     * The break reason for this sequence.
     */
    public final ViterbiBreakReason viterbiBreakReason;
    /**
     * The sequence type.
     */
    public final SequenceType type;
    /**
     * The matched sequence, as returned from viterbi.computeMostLikelySequence().
     * TODO: make this private once it's not needed in MapMatching.java
     */
    final List<SequenceState<Candidate, TimeStep, Path>> matchedSequence;
    /**
     * Time (inclusive, in milliseconds) when first began on this sequence. -1 if not set.
     */
    private final long fromTime;
    /**
     * Time (exclusive, in milliseconds) when last was on this sequence. -1 if not set.
     */
    private final long toTime;
    /**
     * Private variable to track whether or not the match edges have been computed.
     */
    public boolean computedMatchEdges = false;
    /**
     * List of edges that make up this sequence. Null until computeMatchEdges is called.
     */
    public List<MatchedEdge> matchEdges;
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
     * Create an UNKNOWN match sequence
     */
    public MatchSequence(long fromTime, long toTime) {
        this.type = SequenceType.UNKNOWN;
        this.matchedSequence = null;
        this.viterbiBreakReason = null;
        this.fromTime = fromTime;
        this.toTime = toTime;
    }
    /**
     * Create a new MatchSequence.
     * 
     * @param matchedSequence
     * @param hmmTimeSteps
     * @param viterbiBreakReason
     * @param type
     */
    public MatchSequence(List<SequenceState<Candidate, TimeStep, Path>> matchedSequence,
            List<HmmTimeStep> hmmTimeSteps,
            ViterbiBreakReason viterbiBreakReason, SequenceType type) {
        if (matchedSequence.size() < 1)
            throw new IllegalArgumentException(
                    "cannot create a MatchSequence from an empty matchedSequence");
        if (hmmTimeSteps.size() != matchedSequence.size())
            throw new IllegalArgumentException(
                    "matchedSequence and hmmTimeSteps must be the same size");
        this.matchedSequence = matchedSequence;
        this.viterbiBreakReason = viterbiBreakReason;
        this.type = type;
        // times - which assume sequence steps are ordered by time:
        this.fromTime = matchedSequence.get(0).observation.gpxEntry.getTime();
        this.toTime = matchedSequence.get(matchedSequence.size() - 1).observation.gpxEntry.getTime();
        
    }
    
    /**
     * Compute the match edges, including associating GPX entries (inc. ignored ones) to match edges.
     * 
     * @param virtualEdgesMap a map to convert virtual edges to real one
     * @param nodeCount number of nodes in routing graph (so we can detect virtual ones)
     */
    public void computeMatchEdges(Map<String, EdgeIteratorState> virtualEdgesMap, int nodeCount) {
        
        matchDistance = 0.0;
        matchDuration = 0;
        matchEdges = new ArrayList<MatchedEdge>();

        // if it's a stationary/unknown sequence, there are no edges:
        // TODO: should we add the single edge in the case of a stationary sequence?
        if (type != SequenceType.SEQUENCE) {
            computedMatchEdges = true;
            return;
        }
        
        // add the rest:
        EdgeIteratorState lastEdgeAdded = null;
        long realFromTime = fromTime;
        long lastEdgeToTime = fromTime;
        for (int j = 1; j < matchedSequence.size(); j++) {
            SequenceState<Candidate, TimeStep, Path> matchStep = matchedSequence.get(j);
            Path path = matchedSequence.get(j).transitionDescriptor;

            double pathDistance = path.getDistance();
            long realToTime = matchStep.observation.gpxEntry.getTime();
            double realTimePerPathMeter = (double) (realToTime - realFromTime) / (double) pathDistance;
            
            matchDuration += path.getTime();
            matchDistance += pathDistance;

            // loop through edges for this path, and add them
            List<EdgeIteratorState> edges = path.calcEdges();
            EdgeIteratorState edgeIteratorState = null;
            EdgeIteratorState directedRealEdge = null;
            int nEdges = edges.size();
            // it's possible that nEdges = 0 ... e.g. we find a path from (51.45122883155668,
            // 12.316070396818143) to (51.45123598872644, 12.316077738375368) which was empty but
            // found: distance: 0.0, edges: 0, found: true, points: (51.45112037176908,
            // 12.316004834681857).
            // TODO: what's going on here? GH bug? Or something to do with how we've messed with
            // the query graph?
            if (nEdges > 0) {
                for (int edgeIdx = 0; edgeIdx < nEdges; edgeIdx++) {
                    edgeIteratorState = edges.get(edgeIdx);
                    // get time:
                    long edgeToTime = edgeIdx == (nEdges - 1) ? realToTime
                            : (long) (realFromTime
                                    + edgeIteratorState.getDistance() * realTimePerPathMeter);
                    directedRealEdge = resolveToRealEdge(virtualEdgesMap, edgeIteratorState,
                            nodeCount);
                    if (lastEdgeAdded == null || !equalEdges(directedRealEdge, lastEdgeAdded)) {
                        matchEdges.add(new MatchedEdge(directedRealEdge, lastEdgeToTime, edgeToTime));
                        lastEdgeToTime = edgeToTime;
                        lastEdgeAdded = directedRealEdge;
                    }
                }
            }

            // save the matching information of the first match step in the sequence:
            if (j == 1) {
                EdgeIteratorState firstDirectedRealEdge = resolveToRealEdge(virtualEdgesMap, edges.get(0), nodeCount);
                matchedSequence.get(0).observation.saveMatchingState(0, 0, firstDirectedRealEdge, 0,
                        matchedSequence.get(0).observation.getSnappedPoint());
            }
            
            // save the matching information to this match step:
            matchStep.observation.saveMatchingState(j, matchEdges.size() - 1, directedRealEdge,
                    nEdges > 0 ? edgeIteratorState.getDistance() : 0, matchStep.observation.getSnappedPoint());
 
            // update fromTime for next loop to toTime
            realFromTime = realToTime;
        }
        
        // check we do have some edge matches:
        if (matchEdges.isEmpty())
            throw new RuntimeException("match edges shoudn't be empty");
        
        // we're done:
        computedMatchEdges = true;
    }

    /**
     * Check whether two edges are equal
     * @param edge1
     * @param edge2
     * @return true if edge1 'equals' edge2, else false.
     */
    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }
    
    /**
     * Get the real edge containing a given edge (which may be the same thing if it's real already)
     * 
     * @param virtualEdgesMap a map of virtual edges to real ones
     * @param edgeIteratorState the edge to resolve
     * @param nodeCount number of nodes in the base graph (so we can detect virtuality)
     * @return if edgeIteratorState is real, just returns it. Otherwise returns the real edge
     *         containing edgeIteratorState.
     */
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

    /**
     * Detects virtuality of nodes.
     * 
     * @param node node ID
     * @param nodeCount number of nodes in base graph
     * @return true if node is virtual
     */
    private boolean isVirtualNode(int node, int nodeCount) {
        return node >= nodeCount;
    }

    /**
     * Creates a unique key for an edge
     * 
     * @param iter edge to create key for
     * @return a unique key for this edge
     */
    private String virtualEdgesMapKey(EdgeIteratorState iter) {
        return iter.getBaseNode() + "-" + iter.getEdge() + "-" + iter.getAdjNode();
    }
    
    /**
     * Wrapper around Path so we can call some private methods publicly.
     */
    private static class MapMatchedPath extends Path {

        public MapMatchedPath(Graph graph, Weighting weighting) {
            super(graph, weighting);
        }

        @Override
        public Path setFromNode(int from) {
            return super.setFromNode(from);
        }

        @Override
        public void processEdge(int edgeId, int adjNode, int prevEdgeId) {
            super.processEdge(edgeId, adjNode, prevEdgeId);
        }
    }

    /**
     * Utility method to ensure edges are computed before they're accessed.
     */
    private void checkEdgesComputed() {
        if (!computedMatchEdges)
            throw new RuntimeException("must call computeMatchEdges first");
    }
    
    /**
     * Calculate the path for this sequence.
     * 
     * @param graph the base graph
     * @param weighting the weighting (which should be the same as that used in map-matching)
     * @return the path of this sequence
     */
    public Path calcPath(Graph graph, Weighting weighting) {
        checkEdgesComputed();
        MapMatchedPath p = new MapMatchedPath(graph, weighting);
        if (!matchEdges.isEmpty()) {
            int prevEdge = EdgeIterator.NO_EDGE;
            p.setFromNode(matchEdges.get(0).edge.getBaseNode());
            for (MatchedEdge em : matchEdges) {
                p.processEdge(em.edge.getEdge(), em.edge.getAdjNode(), prevEdge);
                prevEdge = em.edge.getEdge();
            }
            p.setFound(true);
            return p;
        } else {
            return p;
        }
    }
    
    public long getFromTime() {
        return fromTime;
    }
    
    public long getToTime() {
        return toTime;
    }
    
    public double getMatchDistance() {
        checkEdgesComputed();
        return matchDistance;
    }
    
    public long getMatchDuration() {
        checkEdgesComputed();
        return matchDuration;
    }
}
