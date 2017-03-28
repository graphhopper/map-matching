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
import com.graphhopper.matching.MatchSequence.ViterbiBreakReason;
import com.graphhopper.matching.MatchSequence.SequenceType;
import com.graphhopper.matching.util.HmmProbabilities;
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
 * This class matches real world GPX entries to the digital road network stored in GraphHopper. The
 * Viterbi algorithm is used to compute the most likely sequence of map matching candidates. The
 * Viterbi algorithm takes into account the distance between GPX entries and map matching candidates
 * as well as the routing distances between consecutive map matching candidates.
 *
 * <p>
 * See http://en.wikipedia.org/wiki/Map_matching and Newson, Paul, and John Krumm. "Hidden Markov
 * map matching through noise and sparseness." Proceedings of the 17th ACM SIGSPATIAL International
 * Conference on Advances in Geographic Information Systems. ACM, 2009.
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

    private final Graph graph;
    private final Graph routingGraph;
    private final LocationIndexTree locationIndex;
    private double measurementErrorSigma = 50.0;
    private double transitionProbabilityBeta = 2.0;
    private final int nodeCount;
    private DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final RoutingAlgorithmFactory algoFactory;
    private final AlgorithmOptions algoOptions;

    public MapMatching(GraphHopper hopper, AlgorithmOptions algoOptions) {
        // Convert heading penalty [s] into U-turn penalty [m]
        final double PENALTY_CONVERSION_VELOCITY = 5; // [m/s]
        final double headingTimePenalty = algoOptions.getHints().getDouble(
                Parameters.Routing.HEADING_PENALTY, Parameters.Routing.DEFAULT_HEADING_PENALTY);
        uTurnDistancePenalty = headingTimePenalty * PENALTY_CONVERSION_VELOCITY;

        this.locationIndex = (LocationIndexTree) hopper.getLocationIndex();

        // create hints from algoOptions, so we can create the algorithm factory
        HintsMap hints = new HintsMap();
        for (Entry<String, String> entry : algoOptions.getHints().toMap().entrySet()) {
            hints.put(entry.getKey(), entry.getValue());
        }

        // default is non-CH
        if (!hints.has(Parameters.CH.DISABLE)) {
            hints.put(Parameters.CH.DISABLE, true);
        }

        // TODO ugly workaround, duplicate data: hints can have 'vehicle' but algoOptions.weighting
        // too!? Similar problem in GraphHopper class
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

        graph = hopper.getGraphHopperStorage();
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
            weighting = algoOptions.hasWeighting() ? algoOptions.getWeighting()
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
     * Beta parameter of the exponential distribution for modeling transition probabilities.
     */
    public void setTransitionProbabilityBeta(double transitionProbabilityBeta) {
        this.transitionProbabilityBeta = transitionProbabilityBeta;
    }

    /**
     * Standard deviation of the normal distribution [m] used for modeling the GPS error.
     */
    public void setMeasurementErrorSigma(double measurementErrorSigma) {
        this.measurementErrorSigma = measurementErrorSigma;
    }

    /**
     * This method does the actual map matching.
     * <p>
     * 
     * @param gpxList the input list with GPX points which should match to edges of the graph
     * specified in the constructor
     */
    public MatchResult doWork(List<GPXEntry> gpxList) {
        if (gpxList.size() < 2) {
            throw new IllegalArgumentException(
                    "Too few coordinates in input file (" + gpxList.size() + "). Correct format?");
        }

        // TODO: check GPX entries are temporally ordered (or have time == 0 for all)

        // map to matchEntries:
        List<TimeStep> matchEntries = new ArrayList<TimeStep>(gpxList.size());
        for (GPXEntry gpxEntry : gpxList) {
            matchEntries.add(new TimeStep(gpxEntry));
        }

        // create map matching events from the input match entries:
        final EdgeFilter edgeFilter = new DefaultEdgeFilter(
                algoOptions.getWeighting().getFlagEncoder());
        List<HmmTimeStep> hmmtimesteps = createHMMTimeSteps(matchEntries, edgeFilter);

        // create the candidates per event:
        final QueryGraph queryGraph = new QueryGraph(routingGraph).setUseEdgeExplorerCache(true);
        final List<QueryResult> allCandidateLocations = new ArrayList<QueryResult>();
        calculateCandidatesPerEvent(hmmtimesteps, allCandidateLocations, queryGraph);

        // TODO: refactor this into a separate methods per PR87 discussion
        logger.debug("================= Query results =================");
        int i = 1;
        for (HmmTimeStep entry : hmmtimesteps) {
            logger.debug("Query results for GPX entry {}", i++);
            for (Candidate candidate : entry.candidates) {
                QueryResult qr = candidate.getQueryResult();
                logger.debug(
                        "Node id: {}, virtual: {}, snapped on: {}, pos: {},{}, "
                                + "query distance: {}",
                        qr.getClosestNode(), isVirtualNode(qr.getClosestNode()),
                        qr.getSnappedPosition(), qr.getSnappedPoint().getLat(),
                        qr.getSnappedPoint().getLon(), qr.getQueryDistance());
            }
        }

        logger.debug("=============== Time steps ===============");
        i = 1;
        for (HmmTimeStep entry : hmmtimesteps) {
            logger.debug("Candidates for time step {}", i++);
            for (Candidate candidate : entry.candidates) {
                logger.debug(candidate.toString());
            }
        }

        // compute the most likely sequences of map matching candidates:
        List<MatchSequence> sequences = computeViterbiSequence(hmmtimesteps, queryGraph);

        // TODO: refactor this into a separate methods per PR87 discussion
        logger.debug("=============== Viterbi results =============== ");
        i = 1;
        for (MatchSequence seq : sequences) {
            int j = 1;
            for (SequenceState<Candidate, TimeStep, Path> ss : seq.matchedSequence) {
                logger.debug("{}-{}: {}, path: {}", i, j, ss.state, ss.transitionDescriptor != null
                        ? ss.transitionDescriptor.calcEdges() : null);
                j++;
            }
            i++;
        }

        // make it contiguous:
        List<MatchSequence> contiguousSequences = makeSequencesContiguous(sequences);

        // at this stage, we have a sequence of most likely results stored as viterbi/HMM
        // structures - let's convert these to more useful things:
        final EdgeExplorer explorer = queryGraph.createEdgeExplorer(edgeFilter);
        MatchResult matchResult = computeMatchResult(contiguousSequences, hmmtimesteps,
                matchEntries, allCandidateLocations, explorer);

        // TODO: refactor this into a separate methods per PR87 discussion
        logger.debug("=============== Matched real edges =============== ");
        i = 1;
        for (MatchedEdge em : matchResult.getEdgeMatches()) {
            logger.debug("{}: {}", i, em.edge);
            i++;
        }
        
        return matchResult;
    }

    /**
     * Create map match events from the input GPX entries. This is largely reshaping the data,
     * though it also clusters GPX entries which are too close together into single steps.
     */
    private List<HmmTimeStep> createHMMTimeSteps(List<TimeStep> matchEntries,
            EdgeFilter edgeFilter) {
        final List<HmmTimeStep> hmmtimesteps = new ArrayList<HmmTimeStep>();
        HmmTimeStep lastTimeStepAdded = null;
        TimeStep prevEntry = null;
        int last = matchEntries.size() - 1;
        for (int i = 0; i <= last; i++) {
            TimeStep timeStep = matchEntries.get(i);
            // ignore those which are within 2 * measurementErrorSigma of the previous (though never
            // ignore the first/last).
            if (i == 0 || i == last
                    || distanceCalc.calcDist(prevEntry.gpxEntry.getLat(),
                            prevEntry.gpxEntry.getLon(), timeStep.gpxEntry.getLat(),
                            timeStep.gpxEntry.getLon()) > 2 * measurementErrorSigma) {
                lastTimeStepAdded = new HmmTimeStep(timeStep);
                hmmtimesteps.add(lastTimeStepAdded);
                prevEntry = timeStep;
            } else {
                timeStep.markAsNotUsedForMatching();
                // TODO: refactor this into a separate methods per PR87 discussion
                logger.debug("Filter out GPX entry: {}", i + 1);
            }
        }
        return hmmtimesteps;
    }

    /**
     * Create candidates per map match event
     */
    private void calculateCandidatesPerEvent(List<HmmTimeStep> viterbiMatchEntries,
            List<QueryResult> allCandidateLocations, QueryGraph queryGraph) {

        // first, find all of the *real* candidate locations for each event i.e. the nodes/edges
        // that are nearby to the GPX entry location.
        final EdgeFilter edgeFilter = new DefaultEdgeFilter(
                algoOptions.getWeighting().getFlagEncoder());
        final List<List<QueryResult>> candidateLocationsPerEvent = new ArrayList<List<QueryResult>>();
        for (HmmTimeStep hmmTimeStep : viterbiMatchEntries) {
            // TODO: shouldn't we find those within e.g. 5 * accuracy? Otherwise we're not
            // effectively utilising the sigma distribution for emission probability?
            List<QueryResult> candidateLocations = hmmTimeStep.findCandidateLocations(graph,
                    locationIndex, edgeFilter, measurementErrorSigma);
            allCandidateLocations.addAll(candidateLocations);
            candidateLocationsPerEvent.add(candidateLocations);
            if (hmmTimeStep.timeStep.gpxEntry.lat == 51.353334 && hmmTimeStep.timeStep.gpxEntry.lon == 12.357289) {
                logger.info(candidateLocations.toString());
            }
        }

        // lookup each of the real candidate locations in the query graph (which virtualizes them,
        // if required). Note we need to do this in this manner since a) we need to create all
        // virtual nodes/edges in the same queryGraph, and b) we can only call 'lookup' once.
        queryGraph.lookup(allCandidateLocations);

        // Different QueryResult can have the same tower node as their closest node. Hence, we now
        // dedupe the query results of each GPX entry by their closest node (#91). This must be done
        // after calling queryGraph.lookup() since this replaces some of the QueryResult nodes with
        // virtual nodes. Virtual nodes are not deduped since there is at most one QueryResult per
        // edge and virtual nodes are inserted into the middle of an edge. Reducing the number of
        // QueryResults improves performance since less shortest/fastest routes need to be computed.
        final List<Collection<QueryResult>> dedupedCandidateLocationsPerEvent = new ArrayList<Collection<QueryResult>>(
                candidateLocationsPerEvent.size());
        for (List<QueryResult> candidateLocations: candidateLocationsPerEvent) {
            final Map<Integer, QueryResult> dedupedCandidateLocations = new HashMap<>(
                    candidateLocations.size());
            for (QueryResult qr : candidateLocations) {
                dedupedCandidateLocations.put(qr.getClosestNode(), qr);
            }
            dedupedCandidateLocationsPerEvent.add(dedupedCandidateLocations.values());
        }

        // create the final candidate and hmmTimeStep per event:
        for (int i = 0; i < viterbiMatchEntries.size(); i++) {
            viterbiMatchEntries.get(i).createCandidates(dedupedCandidateLocationsPerEvent.get(i),
                    queryGraph);
        }
    }

    /**
     * Run the viterbi algorithm on our HMM model. Note that viterbi breaks can occur (e.g. if no
     * candidates are found for a given hmmTimeStep), and we handle these by returning a list
     * of complete sequences (each of which is unbroken). It is possible that a sequence contains
     * only a single hmmTimeStep.
     * 
     * Note: we only break sequences with 'physical' reasons (e.g. no candidates nearby) and not
     * algorithmic ones (e.g. maxVisitedNodes exceeded) - the latter should throw errors.
     */
    private List<MatchSequence> computeViterbiSequence(List<HmmTimeStep> viterbiMatchEntries,
            final QueryGraph queryGraph) {
        final HmmProbabilities probabilities = new HmmProbabilities(measurementErrorSigma,
                transitionProbabilityBeta);
        ViterbiAlgorithm<Candidate, TimeStep, Path> viterbi = null;
        final List<MatchSequence> sequences = new ArrayList<MatchSequence>();
        HmmTimeStep seqPrevTimeStep = null;
        int currentSequenceSize = 0;
        List<HmmTimeStep> currentSequenceHMMTimeSteps = null;
        int totalSequencesSize = 0;
        ViterbiBreakReason breakReason;
        int n = viterbiMatchEntries.size();
        // TODO: refactor this into a separate methods per PR87 discussion
        logger.debug("\n=============== Paths ===============");
        for (int hmmTimeStepIdx = 0; hmmTimeStepIdx < n; hmmTimeStepIdx++) {
            // TODO: refactor this into a separate methods per PR87 discussion
            logger.debug("\nPaths to time step {}", hmmTimeStepIdx);
            HmmTimeStep hmmTimeStep = viterbiMatchEntries.get(hmmTimeStepIdx);

            // always calculate emission probabilities regardless of place in sequence:
            computeEmissionProbabilities(hmmTimeStep, probabilities);

            if (seqPrevTimeStep == null) {
                // first step of a sequence, so initialise viterbi:
                assert currentSequenceSize == 0;
                viterbi = new ViterbiAlgorithm<>();
                currentSequenceHMMTimeSteps = new ArrayList<HmmTimeStep>();
                viterbi.startWithInitialObservation(hmmTimeStep.timeStep,
                        hmmTimeStep.candidates, hmmTimeStep.emissionLogProbabilities);
            } else {
                // add this step to current sequence:
                assert currentSequenceSize > 0;
                computeTransitionProbabilities(seqPrevTimeStep, hmmTimeStep,
                        probabilities, queryGraph);
                viterbi.nextStep(hmmTimeStep.timeStep, hmmTimeStep.candidates,
                        hmmTimeStep.emissionLogProbabilities,
                        hmmTimeStep.transitionLogProbabilities, hmmTimeStep.roadPaths);
            }

            // if sequence is broken, then extract the sequence and reset for a new sequence:
            if (viterbi.isBroken()) {
                // try to guess the break reason:
                breakReason = ViterbiBreakReason.UNKNOWN;
                if (hmmTimeStep.candidates.isEmpty()) {
                    breakReason = ViterbiBreakReason.NO_CANDIDATES;
                } else if (hmmTimeStep.transitionLogProbabilities.isEmpty()) {
                    breakReason = ViterbiBreakReason.NO_POSSIBLE_TRANSITIONS;
                }
                List<SequenceState<Candidate, TimeStep, Path>> viterbiSequence = viterbi
                        .computeMostLikelySequence();
                // We need to handle two cases separately: single event sequences, and more.
                if (seqPrevTimeStep == null) {
                    // OK, we had a break immediately after initialising. In this case, we simply
                    // add the single breaking event as a new (stationary) MapMatchSequence. We rely
                    // on the fact that when there are no transitions, the 
                    // viterbi.computeMostLikelySequence will include this first broken event:
                    if (breakReason == ViterbiBreakReason.NO_CANDIDATES) {
                        viterbiSequence = new ArrayList<SequenceState<Candidate, TimeStep, Path>>(1);
                        viterbiSequence.add(new SequenceState<Candidate, TimeStep, Path>(null, hmmTimeStep.timeStep, null));
                    } else {
                        if (viterbiSequence.size() != 1)
                            throw new IllegalStateException("viterbi sequence should have on element");
                    }
                    currentSequenceHMMTimeSteps.add(hmmTimeStep);
                    sequences.add(
                            new MatchSequence(viterbiSequence, currentSequenceHMMTimeSteps,
                                    breakReason, SequenceType.STATIONARY));
                } else {
                    // OK, we had a break sometime after initialisation. In this case, we need to
                    // add the sequence *excluding* the current hmmTimeStep (that broke it)
                    // and start a new sequence with the breaking hmmTimeStep. We rely on the
                    // fact that viterbi.computeMostLikelySequence will *not* include the breaking
                    // hmmTimeStep.
                    assert viterbiSequence.size() >= 1;
                    sequences.add(
                            new MatchSequence(viterbiSequence, currentSequenceHMMTimeSteps,
                                    breakReason, viterbiSequence.size() == 1
                                            ? SequenceType.STATIONARY : SequenceType.SEQUENCE));
                    // To start a new sequence with this (breaking) hmmTimeStep, we decrement
                    // the loop counter so that this hmmTimeStep is repeated again in the next
                    // loop - though then it should be treated as a start of a sequence (not partway
                    // through one)
                    hmmTimeStepIdx--;
                }

                // In all cases, the sequence broke, so reset sequence
                // variables:
                seqPrevTimeStep = null;
                currentSequenceSize = 0;
                // record saved count for check at the end:
                totalSequencesSize += viterbiSequence.size();
            } else {
                // no breaks, so update the sequence variables:
                currentSequenceSize += 1;
                seqPrevTimeStep = hmmTimeStep;
                currentSequenceHMMTimeSteps.add(hmmTimeStep);
            }
        }

        // add the final sequence (if non-empty):
        if (seqPrevTimeStep != null) {
            final List<SequenceState<Candidate, TimeStep, Path>> viterbiSequence = viterbi
                    .computeMostLikelySequence();
            sequences.add(new MatchSequence(viterbiSequence, currentSequenceHMMTimeSteps,
                    ViterbiBreakReason.LAST_GPX_ENTRY,
                    viterbiSequence.size() == 1 ? SequenceType.STATIONARY : SequenceType.SEQUENCE));
            totalSequencesSize += viterbiSequence.size();
        }

        // check sequence lengths:
        assert totalSequencesSize == viterbiMatchEntries.size() : "totalSequencesSize ("
                + totalSequencesSize + ") != viterbiMatchEntries.size() ("
                + viterbiMatchEntries.size() + ")";
        return sequences;
    }

    /**
     * If there are multiple sequences that are discontinuous in time, insert UNKNOWN sequences in
     * between them to make it continuous.
     */
    private List<MatchSequence> makeSequencesContiguous(List<MatchSequence> matchSequences) {
        List<MatchSequence> contiguousMatchSequences = new ArrayList<MatchSequence>(
                matchSequences.size() * 2);

        int n = matchSequences.size();
        long lastToTime = matchSequences.get(0).getToTime();
        contiguousMatchSequences.add(matchSequences.get(0));
        for (int idx = 1; idx < n; idx++) {
            long fromTime = matchSequences.get(idx).getFromTime();
            // insert an UNKNOWN sequence before this if the times aren't contiguous:
            if (fromTime != lastToTime) {
                matchSequences.add(new MatchSequence(lastToTime, fromTime));
            }
            // add this one:
            contiguousMatchSequences.add(matchSequences.get(idx));
        }
        return contiguousMatchSequences;
    }

    private void computeEmissionProbabilities(HmmTimeStep hmmTimeStep,
            HmmProbabilities probabilities) {
        for (Candidate candidate : hmmTimeStep.candidates) {
            // road distance difference in meters
            final double distance = candidate.getQueryResult().getQueryDistance();
            hmmTimeStep.addEmissionLogProbability(candidate,
                    probabilities.emissionLogProbability(distance));
        }
    }

    private void computeTransitionProbabilities(HmmTimeStep prevTimeStep,
            HmmTimeStep hmmTimeStep, HmmProbabilities probabilities,
            QueryGraph queryGraph) {
        final double linearDistance = distanceCalc.calcDist(
                prevTimeStep.timeStep.gpxEntry.lat,
                prevTimeStep.timeStep.gpxEntry.lon,
                hmmTimeStep.timeStep.gpxEntry.lat,
                hmmTimeStep.timeStep.gpxEntry.lon);

        for (Candidate from : prevTimeStep.candidates) {
            for (Candidate to : hmmTimeStep.candidates) {
                // enforce heading if required:
                if (from.isDirected()) {
                    // Make sure that the path starting at the "from" candidate goes through the
                    // outgoing edge.
                    queryGraph.unfavorVirtualEdgePair(from.getQueryResult().getClosestNode(),
                            from.getIncomingVirtualEdge().getEdge());
                }
                if (to.isDirected()) {
                    // Make sure that the path ending at "to" candidate goes through the incoming
                    // edge.
                    queryGraph.unfavorVirtualEdgePair(to.getQueryResult().getClosestNode(),
                            to.getOutgoingVirtualEdge().getEdge());
                }

                // Need to create a new routing algorithm for every routing.
                RoutingAlgorithm algo = algoFactory.createAlgo(queryGraph, algoOptions);

                final Path path = algo.calcPath(from.getQueryResult().getClosestNode(),
                        to.getQueryResult().getClosestNode());

                if (path.isFound()) {
                    hmmTimeStep.addRoadPath(from, to, path);

                    // The router considers unfavored virtual edges using edge penalties but this is
                    // not reflected in the path distance. Hence, we need to adjust the path
                    // distance accordingly.
                    final double penalizedPathDistance = penalizedPathDistance(path,
                            queryGraph.getUnfavoredVirtualEdges());
                    logger.debug("Path from: {}, to: {}, penalized path length: {}", from, to,
                            penalizedPathDistance);
                    final double transitionLogProbability = probabilities
                            .transitionLogProbability(penalizedPathDistance, linearDistance);
                    hmmTimeStep.addTransitionLogProbability(from, to,
                            transitionLogProbability);
                } else {
                    // TODO: refactor this into a separate methods per PR87 discussion
                    logger.debug("No path found for from: {}, to: {}", from, to);
                    // fail if user hasn't set a high enough maxVisitedNodes
                    if (algo.getVisitedNodes() > algoOptions.getMaxVisitedNodes()) {
                        throw new RuntimeException(
                                "couldn't compute transition probabilities as routing failed due to"
                                        + " too small maxVisitedNodes ("
                                        + algoOptions.getMaxVisitedNodes() + ")");
                    }
                }
                queryGraph.clearUnfavoredStatus();
            }
        }
    }

    /**
     * Returns the path length plus a penalty if the starting/ending edge is unfavored.
     */
    private double penalizedPathDistance(Path path, Set<EdgeIteratorState> penalizedVirtualEdges) {
        double totalPenalty = 0;

        // Unfavored edges in the middle of the path should not be penalized because we are only
        // concerned about the direction at the start/end.
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

    private MatchResult computeMatchResult(List<MatchSequence> sequences,
            List<HmmTimeStep> viterbiMatchEntries, List<TimeStep> matchEntries,
            List<QueryResult> allCandidateLocations, EdgeExplorer explorer) {
        final Map<String, EdgeIteratorState> virtualEdgesMap = createVirtualEdgesMap(
                allCandidateLocations, explorer);

        MatchResult matchResult = new MatchResult(matchEntries, sequences);
        matchResult.computeMatchEdges(virtualEdgesMap, nodeCount);

        // compute GPX stats from the original GPX track:
        matchResult.computeGPXStats(distanceCalc);

        return matchResult;
    }

    private boolean isVirtualNode(int node) {
        return node >= nodeCount;
    }

    /**
     * Returns a map where every virtual edge maps to its real edge with correct orientation.
     */
    private Map<String, EdgeIteratorState> createVirtualEdgesMap(
            List<QueryResult> allCandidateLocations, EdgeExplorer explorer) {
        // TODO For map key, use the traversal key instead of string!
        Map<String, EdgeIteratorState> virtualEdgesMap = new HashMap<>();
        for (QueryResult qr : allCandidateLocations) {
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

    public Path calcPath(MatchSequence matchSequence) {
        MapMatchedPath p = new MapMatchedPath(routingGraph, algoOptions.getWeighting());
        if (!matchSequence.matchEdges.isEmpty()) {
            int prevEdge = EdgeIterator.NO_EDGE;
            p.setFromNode(matchSequence.matchEdges.get(0).edge.getBaseNode());
            for (MatchedEdge em : matchSequence.matchEdges) {
                p.processEdge(em.edge.getEdge(), em.edge.getAdjNode(), prevEdge);
                prevEdge = em.edge.getEdge();
            }
            p.setFound(true);
            return p;
        } else {
            return p;
        }
    }
}