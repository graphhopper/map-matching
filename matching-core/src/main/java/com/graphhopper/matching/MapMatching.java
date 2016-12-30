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

import com.bmw.hmm.SequenceState;
import com.bmw.hmm.ViterbiAlgorithm;
import com.graphhopper.GraphHopper;
import com.graphhopper.matching.util.HmmProbabilities;
import com.graphhopper.matching.util.TimeStep;
import com.graphhopper.routing.*;
import com.graphhopper.routing.ch.CHAlgoFactoryDecorator;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.HintsMap;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.CHGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.Map.Entry;

/**
 * This class matches real world GPX entries to the digital road network stored
 * in GraphHopper. The Viterbi algorithm is used to compute the most likely
 * sequence of map matching candidates. The Viterbi algorithm takes into account
 * the distance between GPX entries and map matching candidates as well as the
 * routing distances between consecutive map matching candidates.
 *
 * <p>
 * See http://en.wikipedia.org/wiki/Map_matching and Newson, Paul, and John
 * Krumm. "Hidden Markov map matching through noise and sparseness." Proceedings
 * of the 17th ACM SIGSPATIAL International Conference on Advances in Geographic
 * Information Systems. ACM, 2009.
 *
 * @author Peter Karich
 * @author Michael Zilske
 * @author Stefan Holder
 * @author kodonnell
 */
public class MapMatching {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    // Penalty in m for each U-turn performed at the beginning or end of a path between two
    // subsequent candidates.
    private double uTurnDistancePenalty;

    private final Graph routingGraph;
    private final LocationIndexMatch locationIndex;
    private double measurementErrorSigma = 50.0;
    private double transitionProbabilityBeta = 2.0;
    private final int nodeCount;
    private DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final RoutingAlgorithmFactory algoFactory;
    private final AlgorithmOptions algoOptions;

    public MapMatching(GraphHopper hopper, AlgorithmOptions algoOptions) {
        // Convert heading penalty [s] into U-turn penalty [m]
        final double PENALTY_CONVERSION_VELOCITY = 5;  // [m/s]
        final double headingTimePenalty = algoOptions.getHints().getDouble(
                Parameters.Routing.HEADING_PENALTY, Parameters.Routing.DEFAULT_HEADING_PENALTY);
        uTurnDistancePenalty = headingTimePenalty * PENALTY_CONVERSION_VELOCITY;

        this.locationIndex = new LocationIndexMatch(hopper.getGraphHopperStorage(),
                (LocationIndexTree) hopper.getLocationIndex());

        // create hints from algoOptions, so we can create the algorithm factory        
        HintsMap hints = new HintsMap();
        for (Entry<String, String> entry : algoOptions.getHints().toMap().entrySet()) {
            hints.put(entry.getKey(), entry.getValue());
        }

        // default is non-CH
        if (!hints.has(Parameters.CH.DISABLE)) {
            hints.put(Parameters.CH.DISABLE, true);
        }

        // TODO ugly workaround, duplicate data: hints can have 'vehicle' but algoOptions.weighting too!?
        // Similar problem in GraphHopper class
        String vehicle = hints.getVehicle();
        if (vehicle.isEmpty()) {
            if (algoOptions.hasWeighting()) {
                vehicle = algoOptions.getWeighting().getFlagEncoder().toString();
            } else {
                vehicle = hopper.getEncodingManager().fetchEdgeEncoders().get(0).toString();
            }
            hints.setVehicle(vehicle);
        }

        if (!hopper.getEncodingManager().supports(vehicle)) {
            throw new IllegalArgumentException("Vehicle " + vehicle + " unsupported. "
                    + "Supported are: " + hopper.getEncodingManager());
        }

        algoFactory = hopper.getAlgorithmFactory(hints);

        Weighting weighting = null;
        CHAlgoFactoryDecorator chFactoryDecorator = hopper.getCHFactoryDecorator();
        boolean forceFlexibleMode = hints.getBool(Parameters.CH.DISABLE, false);
        if (chFactoryDecorator.isEnabled() && !forceFlexibleMode) {
            if (!(algoFactory instanceof PrepareContractionHierarchies)) {
                throw new IllegalStateException("Although CH was enabled a non-CH algorithm "
                        + "factory was returned " + algoFactory);
            }

            weighting = ((PrepareContractionHierarchies) algoFactory).getWeighting();
            this.routingGraph = hopper.getGraphHopperStorage().getGraph(CHGraph.class, weighting);
        } else {
            weighting = algoOptions.hasWeighting()
                    ? algoOptions.getWeighting()
                    : new FastestWeighting(hopper.getEncodingManager().getEncoder(vehicle),
                    algoOptions.getHints());
            this.routingGraph = hopper.getGraphHopperStorage();
        }

        this.algoOptions = AlgorithmOptions.start(algoOptions).weighting(weighting).build();
        this.nodeCount = routingGraph.getNodes();
    }

    public void setDistanceCalc(DistanceCalc distanceCalc) {
        this.distanceCalc = distanceCalc;
    }

    /**
     * Beta parameter of the exponential distribution for modeling transition
     * probabilities.
     */
    public void setTransitionProbabilityBeta(double transitionProbabilityBeta) {
        this.transitionProbabilityBeta = transitionProbabilityBeta;
    }

    /**
     * Standard deviation of the normal distribution [m] used for modeling the
     * GPS error.
     */
    public void setMeasurementErrorSigma(double measurementErrorSigma) {
        this.measurementErrorSigma = measurementErrorSigma;
    }

    /**
     * This method does the actual map matching.
     * <p>
     * @param gpxList the input list with GPX points which should match to edges
     *                of the graph specified in the constructor
     */
    public MatchResult doWork(List<GPXEntry> gpxList) {
        if (gpxList.size() < 2) {
            throw new IllegalArgumentException("Too few coordinates in input file ("
                    + gpxList.size() + "). Correct format?");
        }

        // filter the entries (excluding e.g. ones which are too close to others)
        List<GPXEntry> filteredGPXEntries = filterGPXEntries(gpxList);
        if (filteredGPXEntries.size() < 2) {
            throw new IllegalStateException("Only " + filteredGPXEntries.size()
                    + " filtered GPX entries (from " + gpxList.size()
                    + "), but two or more are needed");
        }

        // for each entry, find the nearby (real) candidate locations
        final EdgeFilter edgeFilter = new DefaultEdgeFilter(algoOptions.getWeighting().getFlagEncoder());
        List<List<QueryResult>> candidateLocationsPerEntry = findCandidateLocationsPerEntry(
                filteredGPXEntries, edgeFilter);

        // lookup each of the real candidate locations in the query graph (which virtualizes 
        // them, if required). Note we need to do this as below since a) we need to create all
        // virtual nodes/edges in the same queryGraph, and b) we can only call 'lookup' once.
        final QueryGraph queryGraph = new QueryGraph(routingGraph).setUseEdgeExplorerCache(true);
        List<QueryResult> allCandidateLocations = new ArrayList<>();
        for (List<QueryResult> qrs: candidateLocationsPerEntry)
            allCandidateLocations.addAll(qrs);
        queryGraph.lookup(allCandidateLocations);

        // add directionality to candidates (so we can penalize U-turns):
        List<List<GPXExtension>> directedCandidatesPerEntry = createDirectedCandidatesPerEntry(
                filteredGPXEntries, candidateLocationsPerEntry, queryGraph);

        // reshape into timeSteps - a format that can be used by the viterbi algorithm:
        List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps =
                createTimeSteps(filteredGPXEntries, directedCandidatesPerEntry);
        
        // compute the most likely sequences of map matching candidates:
        List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences =
                computeViterbiSequence(timeSteps, gpxList, queryGraph);

        // finally, extract the result:
        final EdgeExplorer explorer = queryGraph.createEdgeExplorer(edgeFilter);
        MatchResult matchResult = computeMatchResult(sequences, gpxList, candidateLocationsPerEntry,
                explorer);
        
        return matchResult;
    }
    
    /**
     * Filters GPX entries to only those which will be used for map matching (i.e. those which
     * are separated by at least 2 * measurementErrorSigma)
     */
    private List<GPXEntry> filterGPXEntries(List<GPXEntry> gpxList) {
        List<GPXEntry> filtered = new ArrayList<>();
        GPXEntry prevEntry = null;
        int last = gpxList.size() - 1;
        for (int i = 0; i <= last; i++) {
            GPXEntry gpxEntry = gpxList.get(i);
            if (i == 0 || i == last || distanceCalc.calcDist(
                    prevEntry.getLat(), prevEntry.getLon(),
                    gpxEntry.getLat(), gpxEntry.getLon()) > 2 * measurementErrorSigma) {
                filtered.add(gpxEntry);
                prevEntry = gpxEntry;
            } else {
                logger.debug("Filter out GPX entry: {}", i + 1);
            }
        }
        return filtered;
    }

    /**
     * Find the possible locations of each qpxEntry in the graph.
     */
    private List<List<QueryResult>> findCandidateLocationsPerEntry(List<GPXEntry> gpxList,
            EdgeFilter edgeFilter) {

        List<List<QueryResult>> gpxEntryLocations = new ArrayList<>();
        for (GPXEntry gpxEntry : gpxList) {
            gpxEntryLocations.add(locationIndex.findNClosest(gpxEntry.lat, gpxEntry.lon, edgeFilter,
                    measurementErrorSigma));
        }
        return gpxEntryLocations;
    }

    /**
     * Creates directed candidates for virtual nodes and undirected candidates
     * for real nodes.
     */
    private List<List<GPXExtension>> createDirectedCandidatesPerEntry(
            List<GPXEntry> filteredGPXEntries, List<List<QueryResult>> candidateLocationsPerEntry,
            QueryGraph queryGraph) {

        final int n = filteredGPXEntries.size();
        if (candidateLocationsPerEntry.size() != n) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and candidateLocationsPerEntry must have same size.");
        }

        List<List<GPXExtension>> directedCandidatesPerEntry = new ArrayList<List<GPXExtension>>();
        for (int i = 0; i < n; i++) {

            GPXEntry gpxEntry = filteredGPXEntries.get(i);
            List<QueryResult> candidateLocations = candidateLocationsPerEntry.get(i);
        
            List<GPXExtension> candidates = new ArrayList<>();
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
                        vqr.calcSnappedPoint(distanceCalc);
                        GPXExtension candidate = new GPXExtension(gpxEntry, vqr, incomingVirtualEdge,
                                outgoingVirtualEdge);
                        candidates.add(candidate);
                    }
                } else {
                    // Create an undirected candidate for the real node.
                    GPXExtension candidate = new GPXExtension(gpxEntry, qr);
                    candidates.add(candidate);
                }
            }
            directedCandidatesPerEntry.add(candidates);
        }
        return directedCandidatesPerEntry;
    }

    /**
     * Creates TimeSteps with candidates for the GPX entries but does not create emission or
     * transition probabilities.
     */
    private List<TimeStep<GPXExtension, GPXEntry, Path>> createTimeSteps(
            List<GPXEntry> filteredGPXEntries, List<List<GPXExtension>> directedCandidatesPerEntry) {
        final int n = filteredGPXEntries.size();
        if (directedCandidatesPerEntry.size() != n) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and directedCandidatesPerEntry must have same size.");
        }

        final List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps = new ArrayList<>();
        for (int i = 0; i < n; i++) {

            GPXEntry gpxEntry = filteredGPXEntries.get(i);
            List<GPXExtension> candidates = directedCandidatesPerEntry.get(i);
            final TimeStep<GPXExtension, GPXEntry, Path> timeStep = new TimeStep<>(gpxEntry, candidates);
            timeSteps.add(timeStep);
        }
        return timeSteps;
    }

    /*
     * Run the viterbi algorithm on our HMM model. Note that viterbi breaks can occur (e.g. if no
     * candidates are found for a given timestep), and we handle these by return a list of complete
     * sequences (each of which is unbroken). It is possible that a sequence contains only a single
     * timestep.
     * 
     * Note: we only break sequences with 'physical' reasons (e.g. no candidates nearby) and not
     * algorithmic ones (e.g. maxVisitedNodes exceeded) - the latter should throw errors.
     */
    private List<List<SequenceState<GPXExtension, GPXEntry, Path>>> computeViterbiSequence(
            List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps, List<GPXEntry> gpxList,
            final QueryGraph queryGraph) {
        final HmmProbabilities probabilities = new HmmProbabilities(measurementErrorSigma,
                transitionProbabilityBeta);
        ViterbiAlgorithm<GPXExtension, GPXEntry, Path> viterbi = new ViterbiAlgorithm<>();
        final List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences =
                new ArrayList<List<SequenceState<GPXExtension, GPXEntry, Path>>>();
        TimeStep<GPXExtension, GPXEntry, Path> seqPrevTimeStep = null;
        for (TimeStep<GPXExtension, GPXEntry, Path> timeStep : timeSteps) {

            // if sequence is broken, then close it off and create a new viterbi:
            if (viterbi.isBroken()) {
                sequences.add(viterbi.computeMostLikelySequence());
                seqPrevTimeStep = null;
                viterbi = new ViterbiAlgorithm<>();
            }
            
            // always calculate emission probabilities regardless of place in sequence:
            computeEmissionProbabilities(timeStep, probabilities);

            if (seqPrevTimeStep == null) {
                // first step of a sequence, so initialise viterbi:
                viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates,
                        timeStep.emissionLogProbabilities);
                // it is possible viterbi is immediately broken here (e.g. no candidates) - this
                // will be caught by the first test in this loop.
            } else {
                // add this step to current sequence:
                computeTransitionProbabilities(seqPrevTimeStep, timeStep, probabilities, queryGraph);
                viterbi.nextStep(timeStep.observation, timeStep.candidates,
                        timeStep.emissionLogProbabilities, timeStep.transitionLogProbabilities,
                        timeStep.roadPaths);
                // if broken, then close off this sequence and create a new one starting with this
                // timestep. Note that we rely on the fact that if the viterbi breaks the most
                // recent step does not get added i.e. 'computeMostLikelySequence' returns the most
                // likely sequence without this (breaking) step added. Hence we can use it to start
                // the next one:
                if (viterbi.isBroken()) {
                    sequences.add(viterbi.computeMostLikelySequence());
                    viterbi = new ViterbiAlgorithm<>();
                    viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates,
                            timeStep.emissionLogProbabilities);
                    // as above, it is possible viterbi is immediately broken here (e.g. no
                    // candidates) - this will be caught by the first test in this loop.
                }
            }
            seqPrevTimeStep = timeStep;
        }

        // add the final sequence:
        sequences.add(viterbi.computeMostLikelySequence());

        // check sequence lengths:
        int sequenceSizeSum = 0;
        for (List<SequenceState<GPXExtension, GPXEntry, Path>> sequence : sequences) {
            sequenceSizeSum += sequence.size();
        }
        assert sequenceSizeSum == timeSteps.size();
        
        return sequences;
    }

    private void computeEmissionProbabilities(TimeStep<GPXExtension, GPXEntry, Path> timeStep,
                                              HmmProbabilities probabilities) {
        for (GPXExtension candidate : timeStep.candidates) {
            // road distance difference in meters
            final double distance = candidate.getQueryResult().getQueryDistance();
            timeStep.addEmissionLogProbability(candidate,
                    probabilities.emissionLogProbability(distance));
        }
    }

    private void computeTransitionProbabilities(TimeStep<GPXExtension, GPXEntry, Path> prevTimeStep,
                                                TimeStep<GPXExtension, GPXEntry, Path> timeStep,
                                                HmmProbabilities probabilities,
                                                QueryGraph queryGraph) {
        final double linearDistance = distanceCalc.calcDist(prevTimeStep.observation.lat,
                prevTimeStep.observation.lon, timeStep.observation.lat, timeStep.observation.lon);

        // time difference in seconds
        final double timeDiff
                = (timeStep.observation.getTime() - prevTimeStep.observation.getTime()) / 1000.0;
        logger.debug("Time difference: {} s", timeDiff);

        for (GPXExtension from : prevTimeStep.candidates) {
            for (GPXExtension to : timeStep.candidates) {
                // enforce heading if required:
                if (from.isDirected()) {
                    // Make sure that the path starting at the "from" candidate goes through
                    // the outgoing edge.
                    queryGraph.unfavorVirtualEdgePair(from.getQueryResult().getClosestNode(),
                            from.getIncomingVirtualEdge().getEdge());
                }
                if (to.isDirected()) {
                    // Make sure that the path ending at "to" candidate goes through
                    // the incoming edge.
                    queryGraph.unfavorVirtualEdgePair(to.getQueryResult().getClosestNode(),
                            to.getOutgoingVirtualEdge().getEdge());
                }

                // Need to create a new routing algorithm for every routing.
                RoutingAlgorithm algo = algoFactory.createAlgo(queryGraph, algoOptions);

                final Path path = algo.calcPath(from.getQueryResult().getClosestNode(),
                        to.getQueryResult().getClosestNode());

                if (path.isFound()) {
                    timeStep.addRoadPath(from, to, path);

                    // The router considers unfavored virtual edges using edge penalties
                    // but this is not reflected in the path distance. Hence, we need to adjust the
                    // path distance accordingly.
                    final double penalizedPathDistance = penalizedPathDistance(path,
                            queryGraph.getUnfavoredVirtualEdges());
                    final double transitionLogProbability = probabilities
                            .transitionLogProbability(penalizedPathDistance, linearDistance);
                    timeStep.addTransitionLogProbability(from, to, transitionLogProbability);
                } else {
                    // TODO: can we remove maxVisitedNodes completely and just set to infinity?
                    if (algo.getVisitedNodes() > algoOptions.getMaxVisitedNodes()) {
                        throw new RuntimeException(
                                "couldn't compute transition probabilities as routing failed due to too small maxVisitedNodes ("
                                        + algoOptions.getMaxVisitedNodes() + ")");
                    }
                    // TODO: can we somewhere record that this route failed? Currently all viterbi
                    // knows is that there's no transition possible, but not why. This is useful
                    // for e.g. explaining why a sequence broke.
                }
                queryGraph.clearUnfavoredStatus();

            }
        }
    }

    /**
     * Returns the path length plus a penalty if the starting/ending edge is unfavored.
     */
    private double penalizedPathDistance(Path path,
                                         Set<EdgeIteratorState> penalizedVirtualEdges) {
        double totalPenalty = 0;

        // Unfavored edges in the middle of the path should not be penalized because we are
        // only concerned about the direction at the start/end.
        final List<EdgeIteratorState> edges = path.calcEdges();
        if (!edges.isEmpty()) {
            if (penalizedVirtualEdges.contains(edges.get(0))) {
                totalPenalty += uTurnDistancePenalty;
            }
        }
        if (edges.size() > 1) {
            if (penalizedVirtualEdges.contains(edges.get(edges.size() - 1))) {
                totalPenalty += uTurnDistancePenalty;
            }
        }
        return path.getDistance() + totalPenalty;
    }
    
    private MatchResult computeMatchResult(
            List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences,
            List<GPXEntry> gpxList, List<List<QueryResult>> queriesPerEntry, EdgeExplorer explorer) {

        final Map<String, EdgeIteratorState> virtualEdgesMap = createVirtualEdgesMap(
                queriesPerEntry, explorer);
        // TODO: sequences may be only single timesteps, or disconnected. So we need to support:
        //  - fake edges e.g. 'got from X to Y' but we don't know how ...
        //  - single points e.g. 'was at X from t0 to t1'
        MatchResult matchResult = computeMatchedEdges(sequences, virtualEdgesMap);
        
        // stats:
        computeGpxStats(gpxList, matchResult);

        return matchResult;
    }

    private MatchResult computeMatchedEdges(
            List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences,
            Map<String, EdgeIteratorState> virtualEdgesMap) {
        // TODO: remove gpx extensions and just add time at start/end of edge.
        double distance = 0.0;
        long time = 0;
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        List<GPXExtension> gpxExtensions = new ArrayList<>();
        EdgeIteratorState currentEdge = null;
        for (List<SequenceState<GPXExtension, GPXEntry, Path>> sequence : sequences) {
            GPXExtension queryResult = sequence.get(0).state;
            gpxExtensions.add(queryResult);
            for (int j = 1; j < sequence.size(); j++) {
                queryResult = sequence.get(j).state;
                Path path = sequence.get(j).transitionDescriptor;
                distance += path.getDistance();
                time += path.getTime();
                for (EdgeIteratorState edgeIteratorState : path.calcEdges()) {
                    EdgeIteratorState directedRealEdge =
                            resolveToRealEdge(virtualEdgesMap, edgeIteratorState);
                    if (directedRealEdge == null) {
                        throw new RuntimeException(
                                "Did not find real edge for " + edgeIteratorState.getEdge());
                    }
                    if (currentEdge == null || !equalEdges(directedRealEdge, currentEdge)) {
                        if (currentEdge != null) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentEdge, gpxExtensions);
                            edgeMatches.add(edgeMatch);
                            gpxExtensions = new ArrayList<>();
                        }
                        currentEdge = directedRealEdge;
                    }
                }
                gpxExtensions.add(queryResult);
            }
        }
        
        // we should have some edge matches:
        if (edgeMatches.isEmpty()) {
            String sequenceSizes = "";
            for (List<SequenceState<GPXExtension, GPXEntry, Path>> sequence : sequences) {
                sequenceSizes += sequence.size() + ",";
            }
            throw new IllegalStateException(
                    "No edge matches found for path. Only single-size sequences? Sequence sizes: "
                            + sequenceSizes.substring(0, sequenceSizes.length() - 1));
        }
        EdgeMatch lastEdgeMatch = edgeMatches.get(edgeMatches.size() - 1);
        if (!gpxExtensions.isEmpty() && !equalEdges(currentEdge, lastEdgeMatch.getEdgeState())) {
            edgeMatches.add(new EdgeMatch(currentEdge, gpxExtensions));
        } else {
            lastEdgeMatch.getGpxExtensions().addAll(gpxExtensions);
        }
        MatchResult matchResult = new MatchResult(edgeMatches);
        matchResult.setMatchMillis(time);
        matchResult.setMatchLength(distance);
        return matchResult;
    }

    /**
     * Calculate GPX stats to determine quality of matching.
     */
    private void computeGpxStats(List<GPXEntry> gpxList, MatchResult matchResult) {
        double gpxLength = 0;
        GPXEntry prevEntry = gpxList.get(0);
        for (int i = 1; i < gpxList.size(); i++) {
            GPXEntry entry = gpxList.get(i);
            gpxLength += distanceCalc.calcDist(prevEntry.lat, prevEntry.lon, entry.lat, entry.lon);
            prevEntry = entry;
        }

        long gpxMillis = gpxList.get(gpxList.size() - 1).getTime() - gpxList.get(0).getTime();
        matchResult.setGPXEntriesMillis(gpxMillis);
        matchResult.setGPXEntriesLength(gpxLength);
    }

    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }

    private EdgeIteratorState resolveToRealEdge(Map<String, EdgeIteratorState> virtualEdgesMap,
                                                EdgeIteratorState edgeIteratorState) {
        if (isVirtualNode(edgeIteratorState.getBaseNode())
                || isVirtualNode(edgeIteratorState.getAdjNode())) {
            return virtualEdgesMap.get(virtualEdgesMapKey(edgeIteratorState));
        } else {
            return edgeIteratorState;
        }
    }

    private boolean isVirtualNode(int node) {
        return node >= nodeCount;
    }

    /**
     * Returns a map where every virtual edge maps to its real edge with correct orientation.
     */
    private Map<String, EdgeIteratorState> createVirtualEdgesMap(
            List<List<QueryResult>> queriesPerEntry, EdgeExplorer explorer) {
        // TODO For map key, use the traversal key instead of string!
        Map<String, EdgeIteratorState> virtualEdgesMap = new HashMap<>();
        for (List<QueryResult> queryResults: queriesPerEntry) {
            for (QueryResult qr: queryResults) {
                if (isVirtualNode(qr.getClosestNode())) {
                    EdgeIterator iter = explorer.setBaseNode(qr.getClosestNode());
                    while (iter.next()) {
                        int node = traverseToClosestRealAdj(explorer, iter);
                        if (node == qr.getClosestEdge().getAdjNode()) {
                            virtualEdgesMap.put(virtualEdgesMapKey(iter),
                                    qr.getClosestEdge().detach(false));
                            virtualEdgesMap.put(reverseVirtualEdgesMapKey(iter),
                                    qr.getClosestEdge().detach(true));
                        } else if (node == qr.getClosestEdge().getBaseNode()) {
                            virtualEdgesMap.put(virtualEdgesMapKey(iter),
                                    qr.getClosestEdge().detach(true));
                            virtualEdgesMap.put(reverseVirtualEdgesMapKey(iter),
                                    qr.getClosestEdge().detach(false));
                        } else {
                            throw new RuntimeException();
                        }
                    }
                }
            }
        }
        return virtualEdgesMap;
    }

    private String virtualEdgesMapKey(EdgeIteratorState iter) {
        return iter.getBaseNode() + "-" + iter.getEdge() + "-" + iter.getAdjNode();
    }

    private String reverseVirtualEdgesMapKey(EdgeIteratorState iter) {
        return iter.getAdjNode() + "-" + iter.getEdge() + "-" + iter.getBaseNode();
    }

    private int traverseToClosestRealAdj(EdgeExplorer explorer, EdgeIteratorState edge) {
        if (!isVirtualNode(edge.getAdjNode())) {
            return edge.getAdjNode();
        }

        EdgeIterator iter = explorer.setBaseNode(edge.getAdjNode());
        while (iter.next()) {
            if (iter.getAdjNode() != edge.getBaseNode()) {
                return traverseToClosestRealAdj(explorer, iter);
            }
        }
        throw new IllegalStateException("Cannot find adjacent edge " + edge);
    }

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

    public Path calcPath(MatchResult mr) {
        MapMatchedPath p = new MapMatchedPath(routingGraph, algoOptions.getWeighting());
        if (!mr.getEdgeMatches().isEmpty()) {
            int prevEdge = EdgeIterator.NO_EDGE;
            p.setFromNode(mr.getEdgeMatches().get(0).getEdgeState().getBaseNode());
            for (EdgeMatch em : mr.getEdgeMatches()) {
                p.processEdge(em.getEdgeState().getEdge(), em.getEdgeState().getAdjNode(), prevEdge);
                prevEdge = em.getEdgeState().getEdge();
            }

            p.setFound(true);

            return p;
        } else {
            return p;
        }
    }
}