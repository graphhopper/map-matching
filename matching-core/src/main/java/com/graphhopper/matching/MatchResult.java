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

import com.graphhopper.matching.MatchEntry.MatchState;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;

/**
 *
 * @author Peter Karich
 * @author kodonnell
 */
public class MatchResult {

    private final List<MatchEntry> matchEntries;
    private final List<MatchSequence> sequences;
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
    
    public MatchResult(List<MatchEntry> matchEntries, List<MatchSequence> sequences) {
        this.matchEntries = matchEntries;
        this.sequences = sequences;
    }
    
    public void computeEdgeMatches(Map<String, EdgeIteratorState> virtualEdgesMap, int nodeCount) {
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
    
    public void computeGPXStats(DistanceCalc distCalc) {
        gpxEntriesDistance = 0;
        GPXEntry lastGPXEntry = null;
        for (MatchEntry matchEntry: matchEntries) {
            if (matchEntry.getMatchState() != MatchState.NOT_USED_FOR_MATCHING) {
                gpxEntriesDistance += distCalc.calcDist(lastGPXEntry.lat, lastGPXEntry.lon,
                        matchEntry.gpxEntry.lat, matchEntry.gpxEntry.lon);
                lastGPXEntry = matchEntry.gpxEntry;
            }
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
