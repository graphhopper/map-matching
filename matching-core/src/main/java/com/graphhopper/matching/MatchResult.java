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

import com.graphhopper.matching.util.TimeStep;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;

/**
 *
 * @author Peter Karich
 * @author kodonnell
 */
public class MatchResult {

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
     * A mapping of all the original GPX entries to their final matched position (i.e. which 
     * sequence, etc.)
     */
    private List<GPXMapping> originalGPXMapping = null;
    /**
     * A list of all of the match edges (just a union of those for each sequence).
     */
    private List<MatchEdge> matchEdges = null;
    
    public MatchResult(List<MatchSequence> sequences) {
        this.sequences = sequences;
    }
    
    public void computeEdgeMatches(Map<String, EdgeIteratorState> virtualEdgesMap, int nodeCount, List<GPXEntry> originalGPXEntries) {
        originalGPXMapping = new ArrayList<GPXMapping>();
        matchEdges = new ArrayList<MatchEdge>();
        matchDistance = 0;
        matchDuration = 0;
        for (MatchSequence sequence: sequences) {
            sequence.computeMatchEdges(virtualEdgesMap, nodeCount);
            matchDistance += sequence.getMatchDistance();
            matchDuration += sequence.getMatchDuration();
            int nMatchEdgesThusFar = matchEdges.size();
            for (MatchEntry me: sequence.matchEntries) {
                int matchEdgesIdx = nMatchEdgesThusFar + me.sequenceMatchEdgeIdx;
                originalGPXMapping.add(new GPXMapping(me.gpxEntry, me, matchEdgesIdx, false, -1));
                int neighborIdx = 0;
                for (GPXEntry gpx: me.neighboringGpxEntries) {
                    originalGPXMapping.add(new GPXMapping(gpx, me, matchEdgesIdx, true, neighborIdx++));
                }
            }
            matchEdges.addAll(sequence.matchEdges);
        }
        
        // check 'em
        int n = originalGPXEntries.size();
        assert originalGPXEntries.size() == n;
        for (int gpxIdx = 0; gpxIdx < n; gpxIdx++) {
            assert originalGPXEntries.get(gpxIdx).equals(originalGPXMapping.get(gpxIdx).originalGPXEntry);
        }
    }
    
    public void computeGPXStats(DistanceCalc distCalc, boolean skipSequenceBreaks) {
        gpxEntriesDistance = 0;
        gpxEntriesDuration = 0;
        GPXEntry lastGPXEntry = null;
        for (MatchSequence sequence: sequences) {
            sequence.computeGPXStats(distCalc);
            gpxEntriesDistance += sequence.getMatchDistance();
            gpxEntriesDuration += sequence.getMatchDuration();
            if (!skipSequenceBreaks) {
                if(lastGPXEntry != null) {
                    GPXEntry firstGPXEntry = sequence.timeSteps.get(0).observation;
                    gpxEntriesDistance += distCalc.calcDist(lastGPXEntry.lat, lastGPXEntry.lon, firstGPXEntry.lat, firstGPXEntry.lon);
                    gpxEntriesDuration += (firstGPXEntry.getTime() - lastGPXEntry.getTime());
                }
                TimeStep lastTimeStep = sequence.timeSteps.get(sequence.timeSteps.size() - 1);
                List<GPXEntry> neighbors = lastTimeStep.getNeighboringEntries();
                lastGPXEntry = neighbors.isEmpty() ? lastTimeStep.observation : neighbors.get(neighbors.size() - 1);
            }
        }
    }

    public List<MatchEdge> getEdgeMatches() {
        return matchEdges;
    }
    
    public List<GPXMapping> getOriginalGPXMapping() {
        return originalGPXMapping;
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
