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

import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;

/**
 *
 * @author Peter Karich
 * @author kodonnell
 */
public class MatchResult {

    /**
     * The original GPX entries (wrapped in MatchEntry's to include matching information).
     */
    public final List<MatchEntry> matchEntries;
    /**
     * The sequences that make up the match result.
     */
    public final List<MatchSequence> sequences;
    /**
     * The length (meters) of the total *matched* path, excluding sequence breaks.
     */
    private double matchDistance;
    /**
     * The time (milliseconds) to travel the *matched* path that makes up this sequence, assuming
     * one travels at the speed limit and there are no turn costs between sequence connections.
     */
    private long matchDuration;
    /**
     * The cumulative sequential great-line distance between all of the GPX entries, in meters,
     * optionally skipping the distances between sequence breaks.
     */
    private double gpxEntriesDistance;
    /**
     * The time (milliseconds) between the last and first GPX entry, optionally skipping the
     * time between sequence breaks.
     */
    private long gpxEntriesDuration;
    /**
     * A list of all of the match edges (just a union of those for each sequence).
     */
    private List<MatchEdge> matchEdges = null;
    
    /**
     * Create a match result.
     * 
     * @param matchEntries
     * @param sequences
     */
    public MatchResult(List<MatchEntry> matchEntries, List<MatchSequence> sequences) {
        this.matchEntries = matchEntries;
        this.sequences = sequences;
    }
    
    /**
     * Compute the (real) edges that make up this MatchResult, and some summary information.
     * 
     * @param virtualEdgesMap map to convert virtual edges to real ones
     * @param nodeCount number of nodes in the base graph (so we can detect virtual nodes)
     */
    public void computeMatcheEdges(Map<String, EdgeIteratorState> virtualEdgesMap, int nodeCount) {
        matchEdges = new ArrayList<MatchEdge>();
        matchDistance = 0;
        matchDuration = 0;
        for (MatchSequence sequence: sequences) {
            sequence.computeMatchEdges(virtualEdgesMap, nodeCount);
            matchDistance += sequence.getMatchDistance();
            matchDuration += sequence.getMatchDuration();
            matchEdges.addAll(sequence.matchEdges);
        }
    }

    /**
     * Compute statistics about the original GPX entries e.g. the cumulative point-to-point
     * straight line distance. This is generally so we can compare with the corresponding
     * match statistics.
     * 
     * @param distCalc DistanceCalc to use for calculating distances between GPX entries.
     */
    public void computeGPXStats(DistanceCalc distCalc) {
        gpxEntriesDistance = 0;
        GPXEntry lastGPXEntry = null;
        boolean first = true;
        for (MatchEntry matchEntry: matchEntries) {
            if (first) {
                first = false;
            } else {
                // NOTE: could allow user to calculate GPX stats using only those GPX points
                // used for matching, i.e. matchEntry.getMatchState() == MatchState.MATCHED
                gpxEntriesDistance += distCalc.calcDist(lastGPXEntry.lat, lastGPXEntry.lon,
                        matchEntry.gpxEntry.lat, matchEntry.gpxEntry.lon);
            }
            lastGPXEntry = matchEntry.gpxEntry;
        }
        // NOTE: assumes events temporally ordered!
        gpxEntriesDuration = matchEntries.get(matchEntries.size() - 1).gpxEntry.getTime()
                - matchEntries.get(0).gpxEntry.getTime();
    }

    public List<MatchEdge> getEdgeMatches() {
        return matchEdges;
    }

    public double getGpxEntriesLength() {
        return gpxEntriesDistance;
    }
    
    public long getGpxEntriesMillis() {
        return gpxEntriesDuration;
    }

    public double getMatchLength() {
        return matchDistance;
    }

    public long getMatchMillis() {
        return matchDuration;
    }
}
