/*
 *  Licensed to GraphHopper and Peter Karich under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for 
 *  additional information regarding copyright ownership.
 * 
 *  GraphHopper licenses this file to you under the Apache License, 
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

import com.graphhopper.routing.Dijkstra;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.FastestWeighting;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.util.Weighting;
import com.graphhopper.storage.SPTEntry;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import de.bmw.hmm.MostLikelySequence;
import de.bmw.hmm.TimeStep;
import de.bmw.offline_map_matching.map_matcher.OfflineMapMatcher;
import de.bmw.offline_map_matching.map_matcher.SpatialMetrics;
import de.bmw.offline_map_matching.map_matcher.TemporalMetrics;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.procedure.TIntProcedure;
import gnu.trove.set.hash.TIntHashSet;

import java.util.*;

/**
 * This class matches real world GPX entries to the digital road network stored
 * in GraphHopper. The algorithm is a simple 4 phase process:
 * <p>
 * <ol>
 * <li>Lookup Phase: Find some closest edges for every GPX entry</li>
 * <li>Custom Weighting Phase: Create a weighting object where those edges will
 * be preferred</li>
 * <li>Search Phase: Calculate the path and its list of edges from the best
 * start to the best end edge</li>
 * <li>Match Phase: Associate all GPX entries for every edge</li>
 * </ol>
 * <p>
 *
 * Note: currently tested with very close GPX points only. Will fail if best
 * match for start or end node is incorrect. Performance improvements possible
 * if not the full but only partial routes are calculated this will also improve
 * accuracy as currently all loops in a GPX trail are automatically removed.
 * <p>
 * See http://en.wikipedia.org/wiki/Map_matching
 *
 * @author Peter Karich
 */
public class MapMatching {

    private final Graph graph;
    private final LocationIndexMatch locationIndex;
    private final FlagEncoder encoder;
    private final TraversalMode traversalMode;
    // we split the incoming list into smaller parts (hopefully) without loops
    // later we'll detect loops and insert the correctly detected road recursivly
    // see #1
    private double separatedSearchDistance = 300;
    private int maxNodesToVisit = 500;
    private final double maxSearchWeightMultiplier = 50;
    private final int nodeCount;
    private DistanceCalc distanceCalc = new DistancePlaneProjection();
    private boolean forceRepair;
    private boolean ignoreOneways;
    private Weighting weighting;

    private static final Comparator<QueryResult> CLOSEST_MATCH = new Comparator<QueryResult>() {
        @Override
        public int compare(QueryResult o1, QueryResult o2) {
            return Double.compare(o1.getQueryDistance(), o2.getQueryDistance());
        }
    };

    public MapMatching(Graph graph, LocationIndexMatch locationIndex, FlagEncoder encoder) {
        this.graph = graph;
        this.nodeCount = graph.getNodes();
        this.locationIndex = locationIndex;

        // TODO initialization of start values for the algorithm is currently done explicitely via node IDs! 
        // To fix this use instead: traversalMode.createTraversalId(iter, false);
//        this.traversalMode = graph.getExtension() instanceof TurnCostExtension
//                ? TraversalMode.EDGE_BASED_2DIR : TraversalMode.NODE_BASED;                
        this.traversalMode = TraversalMode.NODE_BASED;
        this.encoder = encoder;
        this.weighting = new FastestWeighting(encoder);
    }

    /**
     * This method overwrites the default fastest weighting.
     */
    public void setWeighting(Weighting weighting) {
        this.weighting = weighting;
    }

    /**
     * This methods forces ignoring oneway directions.
     */
    public void setIgnoreOneways(boolean ignoreOneways) {
        this.ignoreOneways = ignoreOneways;
    }

    public void setDistanceCalc(DistanceCalc distanceCalc) {
        this.distanceCalc = distanceCalc;
    }

    /**
     * Specify the length of the route parts to improve map matching in case of
     * loops in meter. Use -1 if no route splitting should happen. Default is
     * 500m
     */
    public MapMatching setSeparatedSearchDistance(int separatedSearchDistance) {
        this.separatedSearchDistance = separatedSearchDistance;
        return this;
    }

    public void setMaxNodesToVisit(int maxNodesToVisit) {
        this.maxNodesToVisit = maxNodesToVisit;
    }

    public void setForceRepair(boolean forceRepair) {
        this.forceRepair = forceRepair;
    }

    /**
     * This method does the actual map matchting.
     * <p>
     * @param gpxList the input list with GPX points which should match to edges
     * of the graph specified in the constructor
     */
    public MatchResult doWork(List<GPXEntry> gpxList) {
        EdgeFilter edgeFilter = new DefaultEdgeFilter(encoder);
        List<TimeStep<QueryResult, GPXEntry>> timeSteps = new ArrayList<TimeStep<QueryResult, GPXEntry>>();
        List<QueryResult> allQueryResults = new ArrayList<QueryResult>();
        for (GPXEntry entry : gpxList) {
            List<QueryResult> qResults = locationIndex.findNClosest(entry.lat, entry.lon, edgeFilter);
            allQueryResults.addAll(qResults);
            System.out.printf("Candidates: %d\n", qResults.size());
            TimeStep<QueryResult, GPXEntry> timeStep = new TimeStep<QueryResult, GPXEntry>(entry, qResults);
            timeSteps.add(timeStep);
        }
        TemporalMetrics<GPXEntry> temporalMetrics = new TemporalMetrics<GPXEntry>() {
            @Override
            public double timeDifference(GPXEntry m1, GPXEntry m2) {
                long deltaT = m2.getTime() - m1.getTime();
                System.out.printf("Time diff: %d\n", deltaT);
                return deltaT;
            }
        };
        SpatialMetrics<QueryResult, GPXEntry> spatialMetrics = new SpatialMetrics<QueryResult, GPXEntry>() {
            @Override
            public double measurementDistance(QueryResult roadPosition, GPXEntry measurement) {
                System.out.printf("Measurement dist: %f\n", roadPosition.getQueryDistance());
                return roadPosition.getQueryDistance();
            }
            @Override
            public double linearDistance(GPXEntry formerMeasurement, GPXEntry laterMeasurement) {
                double v = distanceCalc.calcDist(formerMeasurement.lat, formerMeasurement.lon, laterMeasurement.lat, laterMeasurement.lon);
                System.out.printf("Linear dist: %f\n", v);
                if (v == 0) {
                    System.out.println("Wurst.");
                }
                return v;
            }
            @Override
            public Double routeLength(QueryResult sourcePosition, QueryResult targetPosition) {
                Dijkstra dijkstra = new Dijkstra(graph, encoder, weighting, traversalMode);
                double distance = dijkstra.calcPath(sourcePosition.getClosestNode(), targetPosition.getClosestNode()).getDistance();
                System.out.printf("Dist: %f\n", distance);
                return distance;
            }
        };
        MostLikelySequence<QueryResult, GPXEntry> seq = OfflineMapMatcher.computeMostLikelySequence(timeSteps, temporalMetrics, spatialMetrics);

        System.out.println(seq.isBroken);
        System.out.println(seq.sequence);
        System.out.printf("%d -> %d\n", timeSteps.size(), seq.sequence.size());

        // every virtual edge maps to its real edge where the orientation is already correct!
        TIntObjectHashMap<EdgeIteratorState> virtualEdgesMap = new TIntObjectHashMap<EdgeIteratorState>();

        QueryGraph queryGraph = new QueryGraph(graph);
        queryGraph.lookup(allQueryResults);

        final EdgeExplorer explorer = queryGraph.createEdgeExplorer(edgeFilter);

        for (TimeStep<QueryResult, GPXEntry> timeStep : timeSteps) {
            for (QueryResult candidate : timeStep.candidates) {
                fillVirtualEdges(virtualEdgesMap, explorer, candidate);
            }
        }


        List<EdgeMatch> edgeMatches = new ArrayList<EdgeMatch>();
        final TIntList nodes = new TIntArrayList();
        double distance = 0.0;
        long time = 0;
        if (!seq.isBroken) {
            // TODO: assert that no two consecutive nodes are the same.
            // TODO: (if so, debounce.)
            List<List<GPXExtension>> gpxExtensions = new ArrayList<List<GPXExtension>>();
            QueryResult queryResult = seq.sequence.get(0);
            nodes.add(queryResult.getClosestNode());
            gpxExtensions.add(Collections.singletonList(new GPXExtension(gpxList.get(0), queryResult, 0)));
            for (int j=1; j<seq.sequence.size(); j++) {
                QueryResult nextQueryResult = seq.sequence.get(j);
                Dijkstra dijkstra = new Dijkstra(queryGraph, encoder, weighting, traversalMode);
                Path path = dijkstra.calcPath(queryResult.getClosestNode(), nextQueryResult.getClosestNode());
                distance += path.getDistance();
                time += path.getTime();
                TIntList tIntList = path.calcNodes();
                for (int k=1; k<tIntList.size(); ++k) {
                    nodes.add(tIntList.get(k));
                }
                for (int k=1; k<tIntList.size()-1; ++k) {
                    gpxExtensions.add(Collections.<GPXExtension>emptyList());
                }
                gpxExtensions.add(Collections.singletonList(new GPXExtension(gpxList.get(j), nextQueryResult, j)));
                queryResult = nextQueryResult;
            }
            System.out.println(nodes);

            final TIntList realNodes = new TIntArrayList();

            List<List<GPXExtension>> realEdgeGpxExtensions = new ArrayList<List<GPXExtension>>();
            List<GPXExtension> oneBucketGPXExtensions = new ArrayList<GPXExtension>();
            for (int j=0; j<nodes.size(); ++j) {
                List<GPXExtension> gpxExtensions1 = gpxExtensions.get(j);
                oneBucketGPXExtensions.addAll(gpxExtensions1);
                int node = nodes.get(j);
                if (node < nodeCount) {
                    if (realNodes.isEmpty() || realNodes.get(realNodes.size()-1) != node) {
                        realEdgeGpxExtensions.add(oneBucketGPXExtensions);
                        oneBucketGPXExtensions = new ArrayList<GPXExtension>();
                        realNodes.add(node);
                    }
                } else if (!realNodes.isEmpty()) {
                    EdgeIterator edgeIterator = explorer.setBaseNode(node);
                    edgeIterator.next();
                    if (edgeIterator.getAdjNode() == realNodes.get(realNodes.size()-1)) {
                        edgeIterator.next();
                    }
                    if (edgeIterator.getAdjNode() < nodeCount) {
                        realEdgeGpxExtensions.add(oneBucketGPXExtensions);
                        oneBucketGPXExtensions = new ArrayList<GPXExtension>();
                        realNodes.add(edgeIterator.getAdjNode());
                    }
                }
            }

            if (nodes.get(0) >= nodeCount) {
                EdgeIterator edgeIterator = explorer.setBaseNode(nodes.get(0));
                edgeIterator.next();
                if (edgeIterator.getAdjNode() == realNodes.get(0)) {
                    edgeIterator.next();
                }
                int realNode = traverseToClosestRealAdj(explorer, edgeIterator);
                realNodes.insert(0, realNode);
            }
            System.out.println(realNodes);

            int n1 = realNodes.get(0);
            for (int j=1; j<realNodes.size(); j++) {
                int n2 = realNodes.get(j);
//                EdgeIteratorState edge = GHUtility.getEdge(graph, n1, n2);
                // TODO: I only want to get the edge, but the previous line gives me the wrong edge, which
                // somehow doesn't know the street name. In fact, there seem to be several edges between
                // the same pair of nodes. A multigraph.
                EdgeIteratorState edge = new Dijkstra(graph, encoder, weighting, traversalMode).calcPath(n1, n2).calcEdges().get(0);
                edgeMatches.add(new EdgeMatch(edge, realEdgeGpxExtensions.get(j-1)));
                n1 = n2;
            }

        }
        System.out.println(edgeMatches);

        MatchResult matchResult = new MatchResult(edgeMatches);
        matchResult.setMatchMillis(time);
        matchResult.setMatchLength(distance);


        //////// Calculate stats to determine quality of matching //////// 
        double gpxLength = 0;
        GPXEntry prevEntry = gpxList.get(0);
        for (int i = 1; i < gpxList.size(); i++) {
            GPXEntry entry = gpxList.get(i);
            gpxLength += distanceCalc.calcDist(prevEntry.lat, prevEntry.lon, entry.lat, entry.lon);
            prevEntry = entry;
        }

        long gpxMillis = gpxList.get(gpxList.size() - 1).getTime()- gpxList.get(0).getTime();
        matchResult.setGPXEntriesMillis(gpxMillis);
        matchResult.setGPXEntriesLength(gpxLength);

        return matchResult;
    }

    private boolean isVirtualNode(int node) {
        return node >= nodeCount;
    }

    /**
     * Fills the minFactorMap with weights for the virtual edges.
     */
    private void fillVirtualEdges(TIntObjectHashMap<EdgeIteratorState> virtualEdgesMap,
                                  EdgeExplorer explorer, QueryResult qr) {
        EdgeIterator iter = explorer.setBaseNode(qr.getClosestNode());
        while (iter.next()) {
            if (isVirtualNode(qr.getClosestNode())) {
                if (traverseToClosestRealAdj(explorer, iter) == qr.getClosestEdge().getAdjNode()) {
                    virtualEdgesMap.put(iter.getEdge(), qr.getClosestEdge());
                } else {
                    virtualEdgesMap.put(iter.getEdge(), qr.getClosestEdge().detach(true));
                }
            }

        }
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

    // TODO instead of checking for edge duplicates check for missing matches
    List<EdgeMatch> checkOrCleanup(List<EdgeMatch> inputList, boolean forceRepair) {
        int prevNode = -1;
        int prevEdge = -1;
        List<String> errors = null;
        List<EdgeMatch> repairedResult = null;
        if (forceRepair) {
            repairedResult = new ArrayList<EdgeMatch>(inputList.size());
        } else {
            errors = new ArrayList<String>();
        }

        for (int i = 0; i < inputList.size(); i++) {
            EdgeMatch em = inputList.get(i);
            EdgeIteratorState edge = em.getEdgeState();
            String str = edge.getName() + ":" + edge.getBaseNode() + "->" + edge.getAdjNode();
            if (prevEdge >= 0) {
                if (edge.getEdge() == prevEdge) {
                    if (forceRepair) {
                        // in all cases skip current edge
                        boolean hasNextEdge = i + 1 < inputList.size();
                        if (hasNextEdge) {
                            EdgeIteratorState nextEdge = inputList.get(i + 1).getEdgeState();
                            // remove previous edge in case of a u-turn
                            if (edge.getAdjNode() == nextEdge.getBaseNode()) {
                                repairedResult.remove(repairedResult.size() - 1);
                                if (!repairedResult.isEmpty()) {
                                    em = repairedResult.get(repairedResult.size() - 1);
                                    edge = em.getEdgeState();
                                    prevEdge = edge.getEdge();
                                    prevNode = edge.getAdjNode();
                                } else {
                                    prevEdge = -1;
                                    prevNode = -1;
                                }
                            }
                        }
                        continue;
                    } else {
                        errors.add("duplicate edge:" + str);
                    }
                }
            }

            if (prevNode >= 0) {
                if (edge.getBaseNode() != prevNode) {
                    if (forceRepair) {
                        if (edge.getAdjNode() != prevNode) {
                            // both nodes inequal to prev adjacent node
                            continue;
                        } else {
                            // really an orientation problem
                            em = new EdgeMatch(edge = em.getEdgeState().detach(true), em.getGpxExtensions());
                        }
                    } else {
                        errors.add("wrong orientation:" + str);
                    }
                }
            }

            if (forceRepair) {
                repairedResult.add(em);
            }

            prevEdge = edge.getEdge();
            prevNode = edge.getAdjNode();
        }

        if (!forceRepair && !errors.isEmpty()) {
            String str = " Result contains illegal edges."
                    + " Try to decrease the separatedSearchDistance (" + separatedSearchDistance + ")"
                    + " or use forceRepair=true. Errors:";
            throw new IllegalStateException(str + errors);
        }

        if (forceRepair) {
            return repairedResult;
        } else {
            return inputList;
        }
    }

    private static class MyPath extends Path {

        public MyPath(Graph graph, FlagEncoder encoder) {
            super(graph, encoder);
        }

        @Override
        public Path setFromNode(int from) {
            return super.setFromNode(from);
        }

        @Override
        public void processEdge(int edgeId, int adjNode) {
            super.processEdge(edgeId, adjNode);
        }
    };

    public Path calcPath(MatchResult mr) {
        MyPath p = new MyPath(graph, encoder);
        if (!mr.getEdgeMatches().isEmpty()) {
            p.setFromNode(mr.getEdgeMatches().get(0).getEdgeState().getBaseNode());
            for (EdgeMatch em : mr.getEdgeMatches()) {
                p.processEdge(em.getEdgeState().getEdge(), em.getEdgeState().getAdjNode());
            }

            // TODO p.setWeight(weight);
            p.setFound(true);

            return p;
        } else {
            return p;
        }
    }
}
