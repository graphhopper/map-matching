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
import com.graphhopper.routing.util.*;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import de.bmw.hmm.Hmm;
import de.bmw.hmm.MostLikelySequence;
import de.bmw.hmm.TimeStep;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;

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
 * @author Michael Zilske
 */
public class MapMatching {

    private final Graph graph;
    private final LocationIndexMatch locationIndex;
    private final FlagEncoder encoder;
    private final TraversalMode traversalMode;

    /**
     * Standard deviation of the normal distribution [m] used for modeling the GPS error taken from
     * Newson&Krumm.
     */
    private double measurementErrorSigma = 10.0;

    int wurst=0;

    /**
     * Beta parameter of the exponential distribution for modeling transition probabilities.
     * Empirically computed from the Microsoft ground truth data for shortest route lengths and
     * 60 s sampling interval but also works for other sampling intervals.
     *
     */
    private double transitionProbabilityBeta = 0.00959442;

    private int maxNodesToVisit = 500;
    private final int nodeCount;
    private DistanceCalc distanceCalc = new DistancePlaneProjection();
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

    public void setDistanceCalc(DistanceCalc distanceCalc) {
        this.distanceCalc = distanceCalc;
    }

    public void setMaxNodesToVisit(int maxNodesToVisit) {
        this.maxNodesToVisit = maxNodesToVisit;
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
        final Map<String, Path> paths = new HashMap<String, Path>();
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
                double deltaTs = (m2.getTime() - m1.getTime()) / 1000.0;
                System.out.printf("Time diff: %.2f\n", deltaTs);
                return deltaTs;
            }
        };
        final QueryGraph queryGraph = new QueryGraph(graph);
        queryGraph.lookup(allQueryResults);
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
                return v;
            }
            @Override
            public Double routeLength(QueryResult sourcePosition, QueryResult targetPosition) {
                ++wurst; // 83129
                if (wurst % 100 == 0) {
                    System.out.println(wurst);
                }
//                final QueryGraph queryGraph = new QueryGraph(graph);
//                queryGraph.lookup(sourcePosition, targetPosition);

                Dijkstra dijkstra = new Dijkstra(queryGraph, encoder, weighting, traversalMode);
                Path path = dijkstra.calcPath(sourcePosition.getClosestNode(), targetPosition.getClosestNode());
                paths.put(hash(sourcePosition, targetPosition), path);
                double distance = path.getDistance();
                System.out.printf("Dist: %f\n", distance);
//                return Math.max(distance, distanceCalc.calcDist(sourcePosition.getQueryPoint().getLat(), sourcePosition.getQueryPoint().getLon(), targetPosition.getQueryPoint().getLat(), targetPosition.getQueryPoint().getLon()))+1;
                return distance;
            }
        };
        MapMatchingHmmProbabilities<QueryResult, GPXEntry> probabilities =
                new MapMatchingHmmProbabilities<QueryResult, GPXEntry>(timeSteps, spatialMetrics, temporalMetrics, measurementErrorSigma, transitionProbabilityBeta);
        MostLikelySequence<QueryResult, GPXEntry> seq = Hmm.computeMostLikelySequence(probabilities, timeSteps.iterator());

        System.out.println(seq.isBroken);
        System.out.println(seq.sequence);
        System.out.printf("%d -> %d\n", timeSteps.size(), seq.sequence.size());

        // every virtual edge maps to its real edge where the orientation is already correct!
        Map<String, EdgeIteratorState> virtualEdgesMap = new HashMap<String, EdgeIteratorState>();

        final EdgeExplorer explorer = queryGraph.createEdgeExplorer(edgeFilter);

        for (QueryResult candidate : allQueryResults) {
            fillVirtualEdges(virtualEdgesMap, explorer, candidate);
        }


        List<EdgeMatch> edgeMatches = new ArrayList<EdgeMatch>();
        double distance = 0.0;
        long time = 0;
        System.out.println("GPX points: " + gpxList.size());
        if (!seq.isBroken) {
            EdgeIteratorState currentEdge = null;
            List<GPXExtension> gpxExtensions = new ArrayList<GPXExtension>();
            QueryResult queryResult = seq.sequence.get(0);
            gpxExtensions.add(new GPXExtension(gpxList.get(0), queryResult, 0));
            for (int j=1; j<seq.sequence.size(); j++) {
                QueryResult nextQueryResult = seq.sequence.get(j);
                Path path = paths.get(hash(queryResult, nextQueryResult));
                distance += path.getDistance();
                time += path.getTime();
                TIntList tIntList = path.calcNodes();
                System.out.println("---");
                for (int k=0; k<tIntList.size(); ++k) {
                    System.out.print(tIntList.get(k) + " ");
                }
                System.out.println();
                for (EdgeIteratorState edgeIteratorState : path.calcEdges()) {
                    EdgeIteratorState directedRealEdge = resolveToRealEdge(virtualEdgesMap, edgeIteratorState);
                    System.out.println("Wurst: " + directedRealEdge);
                    if (directedRealEdge == null) {
                        throw new RuntimeException();
                    }
                    if (currentEdge == null || !equals(directedRealEdge, currentEdge)) {
                        if (currentEdge != null) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentEdge, gpxExtensions);
                            edgeMatches.add(edgeMatch);
                            gpxExtensions = new ArrayList<GPXExtension>();
                        }
                        currentEdge = directedRealEdge;
                    }
                }
                gpxExtensions.add(new GPXExtension(gpxList.get(j), nextQueryResult, j));
                queryResult = nextQueryResult;
            }
            EdgeMatch lastEdgeMatch = edgeMatches.get(edgeMatches.size() - 1);
            if (!gpxExtensions.isEmpty() && !equals(currentEdge, lastEdgeMatch.getEdgeState())) {
                edgeMatches.add(new EdgeMatch(currentEdge, gpxExtensions));
            } else {
                lastEdgeMatch.getGpxExtensions().addAll(gpxExtensions);
            }
        } else {
            throw new RuntimeException("Broken.");
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

    private boolean equals(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }

    private EdgeIteratorState resolveToRealEdge(Map<String, EdgeIteratorState> virtualEdgesMap, EdgeIteratorState edgeIteratorState) {
        if (isVirtualNode(edgeIteratorState.getBaseNode()) || isVirtualNode(edgeIteratorState.getAdjNode())) {
            return virtualEdgesMap.get(virtualEdgesMapKey(edgeIteratorState));
        } else {
            return edgeIteratorState;
        }
    }

    private String hash(QueryResult sourcePosition, QueryResult targetPosition) {
        return sourcePosition.hashCode() + "_" + targetPosition.hashCode();
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
                    virtualEdgesMap.put(virtualEdgesMapKey(iter), qr.getClosestEdge().detach(false));
                    virtualEdgesMap.put(reverseVirtualEdgesMapKey(iter), qr.getClosestEdge().detach(true));
                } else if (node == qr.getClosestEdge().getBaseNode()) {
                    virtualEdgesMap.put(virtualEdgesMapKey(iter), qr.getClosestEdge().detach(true));
                    virtualEdgesMap.put(reverseVirtualEdgesMapKey(iter), qr.getClosestEdge().detach(false));
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
