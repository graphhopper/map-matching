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
package com.graphhopper.matching;

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
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.DistancePlaneProjection;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

/**
 * This is a wrapper around an input MatchEntry (which is itself a wrapped around an input
 * GPXEntry), which contains additional information to work with the Viterbi algorithm (via
 * hmm-lib). For example, it stores the candidates, emission/transition probabilities, etc.
 * 
 * @author Stefan Holder
 * @author kodonnell
 */
public class ViterbiMatchEntry {

    /**
     * DistanceCalc used for e.g. detecting how far candidates are from the original point.
     * TODO: needs to be DistancePlanProject to be consistent with prior behavior - DistanceCalcEarth fails.
     */
    private static DistanceCalc distCalc = new DistancePlaneProjection();
    /**
     * The original MatchEntry (containing the original GPXEntry)
     */
    public final MatchEntry matchEntry;
    /**
     * The possible candidates for this entry (i.e. all the 'nearby' nodes/edges).
     */
    public Collection<Candidate> candidates = null;
    /**
     * The emission probabilities for each candidate.
     */
    public final Map<Candidate, Double> emissionLogProbabilities = new HashMap<>();
    /**
     * The transition probabilities for transitions (from some previous entry's candidates to
     * each of this entry's candidates).
     */
    public final Map<Transition<Candidate>, Double> transitionLogProbabilities = new HashMap<>();
    /**
     * The paths corresponding to the transitions (from some previous entry's candidates to
     * each of this entry's candidates).
     */
    public final Map<Transition<Candidate>, Path> roadPaths = new HashMap<>();
    /**
     * Comparator so we can order candidates by how close they are.
     */
    private static final Comparator<QueryResult> QR_COMPARATOR = new Comparator<QueryResult>() {
        @Override
        public int compare(QueryResult o1, QueryResult o2) {
            return Double.compare(o1.getQueryDistance(), o2.getQueryDistance());
        }
    };

    /**
     * Create a ViterbiMatchEntry.
     * @param matchEntry the original matchEntry
     */
    public ViterbiMatchEntry(MatchEntry matchEntry) {
        assert matchEntry != null;
        this.matchEntry = matchEntry;
    }
    
    /**
     * Find all (real) edges/nodes locations which are within the provided search radius. Note that
     * this searches at most 9 index cells to avoid performance problems, and hence if the radius is
     * larger than the cell width then not all edges might be returned.
     * 
     * TODO: throw an error if searchRadiusMeters > index.getMinResolutionInMeter() as not all edges
     * may be found.
     * 
     * @param graph the base graph to search
     * @param index the base location index to search
     * @param edgeFilter filter for which edges to include/exclude as candidates
     * @param searchRadiusMeters the radius around this entry within which to search.
     * @return the list of candidate locations (as QueryResults)
     */
    public List<QueryResult> findCandidateLocations(final Graph graph,
            final LocationIndexTree index, final EdgeFilter edgeFilter,
            double searchRadiusMeters) {

        final double lat = matchEntry.gpxEntry.lat;
        final double lon = matchEntry.gpxEntry.lon;
        final double returnAllResultsWithin = distCalc.calcNormalizedDist(searchRadiusMeters);

        // implement a cheap priority queue via List, sublist and Collections.sort
        final List<QueryResult> candidateLocations = new ArrayList<QueryResult>();
        GHIntHashSet set = new GHIntHashSet();
        
        // Doing 2 iterations means searching 9 tiles.
        for (int iteration = 0; iteration < 2; iteration++) {
            // should we use the return value of earlyFinish?
            index.findNetworkEntries(lat, lon, set, iteration);

            final GHBitSet exploredNodes = new GHTBitSet(new GHIntHashSet(set));
            final EdgeExplorer explorer = graph.createEdgeExplorer(edgeFilter);

            set.forEach(new IntProcedure() {

                @Override
                public void apply(int node) {
                    index.new XFirstSearchCheck(lat, lon, exploredNodes, edgeFilter) {
                        @Override
                        protected double getQueryDistance() {
                            // do not skip search if distance is 0 or near zero (equalNormedDelta)
                            return Double.MAX_VALUE;
                        }

                        @Override
                        protected boolean check(int node, double normedDist, int wayIndex,
                                EdgeIteratorState edge, QueryResult.Position pos) {
                            // TODO: refactor below:
                            //   - should only add edges within search radius (below allows the
                            //     returning of a candidate outside search radius if it's the only
                            //     one. Removing this test would simplify it a lot.
                            //   - create QueryResult first and the add/set as required - clean up
                            //     the index tracking business.
                            if (normedDist < returnAllResultsWithin
                                    || candidateLocations.isEmpty()
                                    || candidateLocations.get(0).getQueryDistance() > normedDist) {

                                int index = -1;
                                for (int qrIndex = 0; qrIndex < candidateLocations.size();
                                        qrIndex++) {
                                    QueryResult qr = candidateLocations.get(qrIndex);
                                    // overwrite older queryResults which are potentially further
                                    // away than returnAllResultsWithin
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

                                QueryResult qr = new QueryResult(lat, lon);
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
     * 
     * @param candidateLocations list of candidate location (as provided by findCandidateLocations
     *        but already looked up in queryGraph)
     * @param queryGraph the queryGraph being used
     */
    public void createCandidates(Collection<QueryResult> candidateLocations, QueryGraph queryGraph) {
        candidates = new ArrayList<Candidate>();
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
                    Candidate candidate = new Candidate(matchEntry.gpxEntry, vqr, incomingVirtualEdge,
                            outgoingVirtualEdge);
                    candidates.add(candidate);
                }
            } else {
                // Create an undirected candidate for the real node.
                Candidate candidate = new Candidate(matchEntry.gpxEntry, qr);
                candidates.add(candidate);
            }
        }
    }

    /**
     * Save the emission log probability for a given candidate.
     * 
     * @param candidate the candidate whose emission probability is saved
     * @param emissionLogProbability the emission probability to save.
     */
    public void addEmissionLogProbability(Candidate candidate, double emissionLogProbability) {
        if (emissionLogProbabilities.containsKey(candidate)) {
            throw new IllegalArgumentException("Candidate has already been added.");
        }
        emissionLogProbabilities.put(candidate, emissionLogProbability);
    }

    /**
     * Save the transition log probability for a given transition between two candidates. Note
     * that this does not need to be called for non-existent transitions.
     * 
     * @param fromCandidate the from candidate
     * @param toCandidate the to candidate
     * @param transitionLogProbability the transition log probability
     */
    public void addTransitionLogProbability(Candidate fromCandidate, Candidate toCandidate,
            double transitionLogProbability) {
        final Transition<Candidate> transition = new Transition<>(fromCandidate, toCandidate);
        if (transitionLogProbabilities.containsKey(transition)) {
            throw new IllegalArgumentException("Transition has already been added.");
        }
        transitionLogProbabilities.put(transition, transitionLogProbability);
    }

    /**
     * Save the transition path for a given transition between two candidates. Note that this does
     * not need to be called for non-existent transitions.
     * 
     * @param fromCandidate the from candidate
     * @param toCandidate the to candidate
     * @param roadPath the transition log probability
     */
    public void addRoadPath(Candidate fromCandidate, Candidate toCandidate, Path roadPath) {
        final Transition<Candidate> transition = new Transition<>(fromCandidate, toCandidate);
        if (roadPaths.containsKey(transition)) {
            throw new IllegalArgumentException("Transition has already been added.");
        }
        roadPaths.put(transition, roadPath);
    }
}
