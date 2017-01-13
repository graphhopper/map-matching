/**
 * Copyright (C) 2015-2016, BMW Car IT GmbH and BMW AG
 * Author: Stefan Holder (stefan.holder@bmw.de)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.graphhopper.matching.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.bmw.hmm.Transition;
import com.carrotsearch.hppc.procedures.IntProcedure;
import com.graphhopper.coll.GHBitSet;
import com.graphhopper.coll.GHIntHashSet;
import com.graphhopper.coll.GHTBitSet;
import com.graphhopper.matching.GPXExtension;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;

/**
 * Contains everything the hmm-lib needs to process a new time step including emisson and
 * observation probabilities.
 * 
 * @author Stefan Holder
 * @author kodonnell
 */
public class TimeStep {

    private static DistanceCalc distCalc = new DistanceCalcEarth();
    public final GPXEntry observation;
    public Collection<GPXExtension> candidates = null;
    public final Map<GPXExtension, Double> emissionLogProbabilities = new HashMap<>();
    public final Map<Transition<GPXExtension>, Double> transitionLogProbabilities = new HashMap<>();
    public final Map<Transition<GPXExtension>, Path> roadPaths = new HashMap<>();
    private List<GPXEntry> gpxEntryNeighbors = new ArrayList<GPXEntry>();

    private static final Comparator<QueryResult> QR_COMPARATOR = new Comparator<QueryResult>() {
        @Override
        public int compare(QueryResult o1, QueryResult o2) {
            return Double.compare(o1.getQueryDistance(), o2.getQueryDistance());
        }
    };

    public TimeStep(GPXEntry observation) {
        if (observation == null) {
            throw new NullPointerException("observation must not be null.");
        }
        this.observation = observation;
    }


    /*
     * Find all results which are within the GPS signal accuracy.
     * TODO: shouldn't we find those within e.g. 5 * accuracy? ...
     */
    public List<QueryResult> findCandidateLocations(final Graph graph,
            final LocationIndexTree index, final EdgeFilter edgeFilter,
            double gpxAccuracyInMetern) {
        
        final double returnAllResultsWithin = distCalc.calcNormalizedDist(gpxAccuracyInMetern);

        // implement a cheap priority queue via List, sublist and Collections.sort
        // TODO: check hasn't already been run ...
        final List<QueryResult> candidateLocations = new ArrayList<QueryResult>();
        GHIntHashSet set = new GHIntHashSet();

        for (int iteration = 0; iteration < 2; iteration++) {
            // should we use the return value of earlyFinish?
            index.findNetworkEntries(observation.lat, observation.lon, set, iteration);

            final GHBitSet exploredNodes = new GHTBitSet(new GHIntHashSet(set));
            final EdgeExplorer explorer = graph.createEdgeExplorer(edgeFilter);

            set.forEach(new IntProcedure() {

                @Override
                public void apply(int node) {
                    index.new XFirstSearchCheck(observation.lat, observation.lon, exploredNodes, edgeFilter) {
                        @Override
                        protected double getQueryDistance() {
                            // do not skip search if distance is 0 or near zero (equalNormedDelta)
                            return Double.MAX_VALUE;
                        }

                        @Override
                        protected boolean check(int node, double normedDist, int wayIndex, EdgeIteratorState edge, QueryResult.Position pos) {
                            if (normedDist < returnAllResultsWithin
                                    || candidateLocations.isEmpty()
                                    || candidateLocations.get(0).getQueryDistance() > normedDist) {

                                int index = -1;
                                for (int qrIndex = 0; qrIndex < candidateLocations.size(); qrIndex++) {
                                    QueryResult qr = candidateLocations.get(qrIndex);
                                    // overwrite older queryResults which are potentially more far away than returnAllResultsWithin
                                    if (qr.getQueryDistance() > returnAllResultsWithin) {
                                        index = qrIndex;
                                        break;
                                    }

                                    // avoid duplicate edges
                                    if (qr.getClosestEdge().getEdge() == edge.getEdge()) {
                                        if (qr.getQueryDistance() < normedDist) {
                                            // do not add current edge
                                            return true;
                                        } else {
                                            // overwrite old edge with current
                                            index = qrIndex;
                                            break;
                                        }
                                    }
                                }

                                QueryResult qr = new QueryResult(observation.lat, observation.lon);
                                qr.setQueryDistance(normedDist);
                                qr.setClosestNode(node);
                                qr.setClosestEdge(edge.detach(false));
                                qr.setWayIndex(wayIndex);
                                qr.setSnappedPosition(pos);

                                if (index < 0) {
                                    candidateLocations.add(qr);
                                } else {
                                    candidateLocations.set(index, qr);
                                }
                            }
                            return true;
                        }
                    }.start(explorer, node);                    
                }
            });
        }

        Collections.sort(candidateLocations, QR_COMPARATOR);

        for (QueryResult qr : candidateLocations) {
            if (qr.isValid()) {
                // denormalize distance
                qr.setQueryDistance(distCalc.calcDenormalizedDist(qr.getQueryDistance()));
                qr.calcSnappedPoint(distCalc);
            } else {
                throw new IllegalStateException("Invalid QueryResult should not happen here: " + qr);
            }
        }
        
        return candidateLocations;
    }
    
    /**
     * Create the (directed) candidates based on the provided candidate locations. 
     */
    public void createCandidates(List<QueryResult> candidateLocations, QueryGraph queryGraph) {
        candidates = new ArrayList<GPXExtension>();
        for (QueryResult qr: candidateLocations) {
            int closestNode = qr.getClosestNode();
            if (queryGraph.isVirtualNode(closestNode)) {
                // get virtual edges:
                List<VirtualEdgeIteratorState> virtualEdges = new ArrayList<>();
                EdgeIterator iter = queryGraph.createEdgeExplorer().setBaseNode(closestNode);
                while (iter.next()) {
                    if (!queryGraph.isVirtualEdge(iter.getEdge())) {
                        throw new RuntimeException("Virtual nodes must only have virtual edges "
                                + "to adjacent nodes.");
                    }
                    virtualEdges.add((VirtualEdgeIteratorState)
                            queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
                }
                if( virtualEdges.size() != 2) {
                    throw new RuntimeException("Each virtual node must have exactly 2 "
                            + "virtual edges (reverse virtual edges are not returned by the "
                            + "EdgeIterator");
                }
    
                // Create a directed candidate for each of the two possible directions through
                // the virtual node. This is needed to penalize U-turns at virtual nodes
                // (see also #51). We need to add candidates for both directions because
                // we don't know yet which is the correct one. This will be figured
                // out by the Viterbi algorithm.
                //
                // Adding further candidates to explicitly allow U-turns through setting
                // incomingVirtualEdge==outgoingVirtualEdge doesn't make sense because this
                // would actually allow to perform a U-turn without a penalty by going to and
                // from the virtual node through the other virtual edge or its reverse edge.
                VirtualEdgeIteratorState e1 = virtualEdges.get(0);
                VirtualEdgeIteratorState e2 = virtualEdges.get(1);
                for (int j = 0; j < 2; j++) {
                    // get favored/unfavored edges:
                    VirtualEdgeIteratorState incomingVirtualEdge = j == 0 ? e1 : e2;
                    VirtualEdgeIteratorState outgoingVirtualEdge = j == 0 ? e2 : e1;
                    // create candidate
                    QueryResult vqr = new QueryResult(qr.getQueryPoint().lat, qr.getQueryPoint().lon);
                    vqr.setQueryDistance(qr.getQueryDistance());
                    vqr.setClosestNode(qr.getClosestNode());
                    vqr.setWayIndex(qr.getWayIndex());
                    vqr.setSnappedPosition(qr.getSnappedPosition());
                    vqr.setClosestEdge(qr.getClosestEdge());
                    vqr.calcSnappedPoint(distCalc);
                    GPXExtension candidate = new GPXExtension(observation, vqr, incomingVirtualEdge,
                            outgoingVirtualEdge);
                    candidates.add(candidate);
                }
            } else {
                // Create an undirected candidate for the real node.
                GPXExtension candidate = new GPXExtension(observation, qr);
                candidates.add(candidate);
            }
        }
    }

    public void addEmissionLogProbability(GPXExtension candidate, double emissionLogProbability) {
        if (emissionLogProbabilities.containsKey(candidate)) {
            throw new IllegalArgumentException("Candidate has already been added.");
        }
        emissionLogProbabilities.put(candidate, emissionLogProbability);
    }

    /**
     * Does not need to be called for non-existent transitions.
     */
    public void addTransitionLogProbability(GPXExtension fromPosition, GPXExtension toPosition,
                                            double transitionLogProbability) {
        final Transition<GPXExtension> transition = new Transition<>(fromPosition, toPosition);
        if (transitionLogProbabilities.containsKey(transition)) {
            throw new IllegalArgumentException("Transition has already been added.");
        }
        transitionLogProbabilities.put(transition, transitionLogProbability);
    }

    /**
     * Does not need to be called for non-existent transitions.
     */
    public void addRoadPath(GPXExtension fromPosition, GPXExtension toPosition, Path roadPath) {
        final Transition<GPXExtension> transition = new Transition<>(fromPosition, toPosition);
        if (roadPaths.containsKey(transition)) {
            throw new IllegalArgumentException("Transition has already been added.");
        }
        roadPaths.put(transition, roadPath);
    }
    
    /**
     * Adds a neighboring GPX entry
     */
    public void addNeighboringGPXEntry(GPXEntry entry) {
        gpxEntryNeighbors.add(entry);
    }
    
    public List<GPXEntry> getNeighboringEntries() {
        return gpxEntryNeighbors;
    }

}
