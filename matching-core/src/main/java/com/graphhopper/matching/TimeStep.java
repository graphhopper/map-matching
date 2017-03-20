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

import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.shapes.GHPoint3D;

/**
 * A TimeStep is a thin wrapper use to store a) the original GPX entry, and b) any additional
 * information from the map-matching (e.g. where this GPX entry was actually mapped to, etc.)
 * 
 * @author kodonnell
 */
public class TimeStep {
    /**
     * Describe how the state of this match in the matching process.
     */
    public static enum MatchState {
        MATCHING_STATE_NOT_SET, NOT_USED_FOR_MATCHING, MATCHED
    };

    /**
     * The state of this match in the matching process.
     */
    private MatchState matchState = MatchState.MATCHING_STATE_NOT_SET;
    /**
     * Flag to ensure the matchState is only set once.
     */
    private boolean stateSet = false;
    /**
     * The original GPX entry.
     */
    public final GPXEntry gpxEntry;
    /**
     * The point on the map-match result which this entry was 'snapped' to. E.g. if the original
     * entry was 5m off a road, and that road was in the map-match result, then the snappedPoint
     * will be the point on that road 5m away from the original GPX entry. Note that snappedPoint
     * should be on directedRealEdge.
     */
    private GHPoint3D snappedPoint;
    /**
     * The (real) edge containing the snappedPoint.
     */
    private EdgeIteratorState directedRealEdge;
    /**
     * The distance along the directedRealEdge (starting from the baseNode) to the snappedPoint.
     */
    private double distanceAlongRealEdge;
    /**
     * The index of the sequence in the map-match result containing this entry.
     */
    private int sequenceIdx;
    /**
     * The index of the corresponding TimeStep in this sequence.
     */
    private int sequenceMatchEdgeIdx;

    /**
     * Create a TimeStep from a GPXEntry, to be used in map-matching.
     * 
     * @param gpxEntry
     */
    public TimeStep(GPXEntry gpxEntry) {
        this.gpxEntry = gpxEntry;
    }

    /**
     * Flag this entry as not to be used for map-matching which can e.g. happen if there is more
     * points in a given area than the resolution needed by the Viterbi algorithm.
     */
    protected void markAsNotUsedForMatching() {
        assert !stateSet;
        this.matchState = MatchState.NOT_USED_FOR_MATCHING;
        stateSet = true;
    }

    /**
     * Update the matching information.
     * 
     * @param sequenceIdx
     * @param sequenceMatchEdgeIdx
     * @param directedRealEdge
     * @param distanceAlongRealEdge
     * @param snappedPoint
     */
    protected void saveMatchingState(int sequenceIdx, int sequenceMatchEdgeIdx,
            EdgeIteratorState directedRealEdge, double distanceAlongRealEdge,
            GHPoint3D snappedPoint) {
        assert !stateSet;
        this.matchState = MatchState.MATCHED;
        this.sequenceIdx = sequenceIdx;
        this.sequenceMatchEdgeIdx = sequenceMatchEdgeIdx;
        this.directedRealEdge = directedRealEdge;
        this.distanceAlongRealEdge = distanceAlongRealEdge;
        this.snappedPoint = snappedPoint;
        stateSet = true;
    }

    public MatchState getMatchState() {
        return matchState;
    }

    public int getSequenceIdx() {
        return sequenceIdx;
    }

    public GHPoint3D getSnappedPoint() {
        return snappedPoint;
    }

    public EdgeIteratorState getDirectedRealEdge() {
        return directedRealEdge;
    }

    public int getSequenceMatchEdgeIdx() {
        return sequenceMatchEdgeIdx;
    }

    public double getDistanceAlongRealEdge() {
        return distanceAlongRealEdge;
    }
}
