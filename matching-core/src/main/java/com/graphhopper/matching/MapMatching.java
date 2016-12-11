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

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.util.HmmProbabilities;
import com.graphhopper.matching.util.TimeStep;
import com.graphhopper.routing.weighting.Weighting;
import com.bmw.hmm.SequenceState;
import com.bmw.hmm.ViterbiAlgorithm;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.RoutingAlgorithmFactory;
import com.graphhopper.routing.ch.CHAlgoFactoryDecorator;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.*;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.storage.CHGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

    private final Graph routingGraph;
    private final LocationIndexMatch locationIndex;
    private double measurementErrorSigma = 50.0;
    private double transitionProbabilityBeta = 0.00959442;
    private final int nodeCount;
    private DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final RoutingAlgorithmFactory algoFactory;
    private final AlgorithmOptions algoOptions;

    public MapMatching(GraphHopper hopper, AlgorithmOptions algoOptions) {
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
                throw new IllegalStateException("Although CH was enabled a non-CH algorithm factory was returned " + algoFactory);
            }

            weighting = ((PrepareContractionHierarchies) algoFactory).getWeighting();
            this.routingGraph = hopper.getGraphHopperStorage().getGraph(CHGraph.class, weighting);
        } else {
            weighting = algoOptions.hasWeighting()
                    ? algoOptions.getWeighting()
                    : new FastestWeighting(hopper.getEncodingManager().getEncoder(vehicle), algoOptions.getHints());
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

        final EdgeFilter edgeFilter = new DefaultEdgeFilter(algoOptions.getWeighting().getFlagEncoder());

        // Compute all candidates first.
        // TODO: Generate candidates on-the-fly within computeViterbiSequence() if this does not
        // degrade performance.
        // @kodonnell: it's not currently possible as that'd mean calling queryGraph.lookup
        //             multiple times, which isn't supported. So, remove TODO?
        final List<QueryResult> allCandidates = new ArrayList<>();
        List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps = createTimeSteps(gpxList,
                edgeFilter, allCandidates);

        if (allCandidates.size() < 2) {
            throw new IllegalArgumentException("Too few matching coordinates ("
                    + allCandidates.size() + "). Wrong region imported?");
        }
        if (timeSteps.size() < 2) {
            throw new IllegalStateException("Coordinates produced too few time steps "
                    + timeSteps.size() + ", gpxList:" + gpxList.size());
        }

        final QueryGraph queryGraph = new QueryGraph(routingGraph).setUseEdgeExplorerCache(true);
        queryGraph.lookup(allCandidates);

        List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences =
                computeViterbiSequence(timeSteps, gpxList, queryGraph);

        final EdgeExplorer explorer = queryGraph.createEdgeExplorer(edgeFilter);
        MatchResult matchResult = computeMatchResult(sequences, gpxList, allCandidates, explorer);

        return matchResult;
    }

    /**
     * Creates TimeSteps for the GPX entries but does not create emission or
     * transition probabilities.
     *
     * @param outAllCandidates output parameter for all candidates, must be an
     *                         empty list.
     */
    private List<TimeStep<GPXExtension, GPXEntry, Path>> createTimeSteps(List<GPXEntry> gpxList,
                                                                         EdgeFilter edgeFilter, List<QueryResult> outAllCandidates) {
        int indexGPX = 0;
        TimeStep<GPXExtension, GPXEntry, Path> prevTimeStep = null;
        final List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps = new ArrayList<>();

        for (GPXEntry gpxEntry : gpxList) {
            if (prevTimeStep == null
                    || distanceCalc.calcDist(
                            prevTimeStep.observation.getLat(), prevTimeStep.observation.getLon(),
                            gpxEntry.getLat(), gpxEntry.getLon()) > 2 * measurementErrorSigma
                    // always include last point
                    || indexGPX == gpxList.size() - 1) {
                final List<QueryResult> queryResults = locationIndex.findNClosest(
                        gpxEntry.lat, gpxEntry.lon,
                        edgeFilter, measurementErrorSigma);
                outAllCandidates.addAll(queryResults);
                final List<GPXExtension> candidates = new ArrayList<>();
                for (QueryResult candidate : queryResults) {
                    candidates.add(new GPXExtension(gpxEntry, candidate, indexGPX));
                }
                final TimeStep<GPXExtension, GPXEntry, Path> timeStep
                        = new TimeStep<>(gpxEntry, candidates);
                timeSteps.add(timeStep);
                prevTimeStep = timeStep;
            }
            indexGPX++;
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
        ViterbiAlgorithm<GPXExtension, GPXEntry, Path> viterbi = null;
        final List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences =
                new ArrayList<List<SequenceState<GPXExtension, GPXEntry, Path>>>();
        TimeStep<GPXExtension, GPXEntry, Path> prevTimeStep = null;
        for (TimeStep<GPXExtension, GPXEntry, Path> timeStep : timeSteps) {

            // always calculate emission probabilities regardless of place in sequence:
            computeEmissionProbabilities(timeStep, probabilities);

            if (prevTimeStep == null) {
                // first step, so create new viterbi:
                viterbi = new ViterbiAlgorithm<>();
                viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates,
                        timeStep.emissionLogProbabilities);
                assert !viterbi.isBroken(); // TODO: is this a no-op?
            } else {
                // add this step to current sequence:
                computeTransitionProbabilities(prevTimeStep, timeStep, probabilities, queryGraph);
                viterbi.nextStep(timeStep.observation, timeStep.candidates,
                        timeStep.emissionLogProbabilities, timeStep.transitionLogProbabilities,
                        timeStep.roadPaths);
                // if broken, then close off this sequence and create a new one. Note that we rely
                // on the fact that if the viterbi breaks the most recent step does not get added
                // i.e. 'computeMostLikelySequence' returns the most likely sequence without this
                // (breaking) step added. Hence we can use it to start the next one:
                // TODO: check the above is true
                if (viterbi.isBroken()) {
                    sequences.add(viterbi.computeMostLikelySequence());
                    viterbi = new ViterbiAlgorithm<>();
                    viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates,
                            timeStep.emissionLogProbabilities);
                    assert !viterbi.isBroken(); // TODO: is this a no-op?
                }
            }
            prevTimeStep = timeStep;
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

        for (GPXExtension from : prevTimeStep.candidates) {
            for (GPXExtension to : timeStep.candidates) {
                RoutingAlgorithm algo = algoFactory.createAlgo(queryGraph, algoOptions);
                final Path path = algo.calcPath(from.getQueryResult().getClosestNode(),
                        to.getQueryResult().getClosestNode());
                if (path.isFound()) {
                    timeStep.addRoadPath(from, to, path);
                    final double transitionLogProbability = probabilities
                            .transitionLogProbability(path.getDistance(), linearDistance, timeDiff);
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
            }
        }
    }

    private MatchResult computeMatchResult(
            List<List<SequenceState<GPXExtension, GPXEntry, Path>>> sequences,
            List<GPXEntry> gpxList, List<QueryResult> allCandidates, EdgeExplorer explorer) {
        // every virtual edge maps to its real edge where the orientation is already correct!
        // TODO use traversal key instead of string!
        final Map<String, EdgeIteratorState> virtualEdgesMap = new HashMap<>();
        for (QueryResult candidate : allCandidates) {
            fillVirtualEdges(virtualEdgesMap, explorer, candidate);
        }
        // TODO: sequences may be only single timesteps, or disconnected. So we need to support:
        //  - fake edges e.g. 'got from X to Y' but we don't know how ...
        //  - single points e.g. 'was at X from t0 to t1'
        MatchResult matchResult = computeMatchedEdges(sequences, virtualEdgesMap);
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
     * Fills the minFactorMap with weights for the virtual edges.
     */
    private void fillVirtualEdges(Map<String, EdgeIteratorState> virtualEdgesMap,
                                  EdgeExplorer explorer, QueryResult qr) {
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
                    virtualEdgesMap.put(virtualEdgesMapKey(iter), qr.getClosestEdge().detach(true));
                    virtualEdgesMap.put(reverseVirtualEdgesMapKey(iter),
                            qr.getClosestEdge().detach(false));
                } else {
                    throw new RuntimeException();
                }
            }
        }
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

    private String getSnappedCandidates(Collection<GPXExtension> candidates) {
        String str = "";
        for (GPXExtension gpxe : candidates) {
            if (!str.isEmpty()) {
                str += ", ";
            }
            str += "distance: " + gpxe.queryResult.getQueryDistance() + " to "
                    + gpxe.queryResult.getSnappedPoint();
        }
        return "[" + str + "]";
    }

    private void printMinDistances(List<TimeStep<GPXExtension, GPXEntry, Path>> timeSteps) {
        TimeStep<GPXExtension, GPXEntry, Path> prevStep = null;
        int index = 0;
        for (TimeStep<GPXExtension, GPXEntry, Path> ts : timeSteps) {
            if (prevStep != null) {
                double dist = distanceCalc.calcDist(
                        prevStep.observation.lat, prevStep.observation.lon,
                        ts.observation.lat, ts.observation.lon);
                double minCand = Double.POSITIVE_INFINITY;
                for (GPXExtension prevGPXE : prevStep.candidates) {
                    for (GPXExtension gpxe : ts.candidates) {
                        GHPoint psp = prevGPXE.queryResult.getSnappedPoint();
                        GHPoint sp = gpxe.queryResult.getSnappedPoint();
                        double tmpDist = distanceCalc.calcDist(psp.lat, psp.lon, sp.lat, sp.lon);
                        if (tmpDist < minCand) {
                            minCand = tmpDist;
                        }
                    }
                }
                System.out.println(index + ": " + Math.round(dist) + "m, minimum candidate: "
                        + Math.round(minCand) + "m");
                index++;
            }

            prevStep = ts;
        }
    }

    // TODO: Make setFromNode and processEdge public in Path and then remove this.
    private static class MyPath extends Path {

        public MyPath(Graph graph, Weighting weighting) {
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
        MyPath p = new MyPath(routingGraph, algoOptions.getWeighting());
        if (!mr.getEdgeMatches().isEmpty()) {
            int prevEdge = EdgeIterator.NO_EDGE;
            p.setFromNode(mr.getEdgeMatches().get(0).getEdgeState().getBaseNode());
            for (EdgeMatch em : mr.getEdgeMatches()) {
                p.processEdge(em.getEdgeState().getEdge(), em.getEdgeState().getAdjNode(), prevEdge);
                prevEdge = em.getEdgeState().getEdge();
            }

            // TODO p.setWeight(weight);
            p.setFound(true);

            return p;
        } else {
            return p;
        }
    }
}
