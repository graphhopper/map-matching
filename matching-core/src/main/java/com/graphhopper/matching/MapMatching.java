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
    private double measurementErrorSigma = 1.0;

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
                QueryGraph queryGraph = new QueryGraph(graph);
                queryGraph.lookup(sourcePosition, targetPosition);
                Dijkstra dijkstra = new Dijkstra(queryGraph, encoder, weighting, traversalMode);
                Path path = dijkstra.calcPath(sourcePosition.getClosestNode(), targetPosition.getClosestNode());
                paths.put(hash(sourcePosition, targetPosition), path);
                double distance = path.getDistance();
                System.out.printf("Dist: %f\n", distance);
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
        System.out.println("GPX points: " + gpxList.size());
        if (!seq.isBroken) {
            // TODO: assert that no two consecutive nodes are the same.
            // TODO: (if so, debounce.)
            List<List<GPXExtension>> gpxExtensions = new ArrayList<List<GPXExtension>>();
            QueryResult queryResult = seq.sequence.get(0);
            nodes.add(queryResult.getClosestNode());
            gpxExtensions.add(new ArrayList<GPXExtension>(Arrays.asList(new GPXExtension(gpxList.get(0), queryResult, 0))));
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
                for (int k=1; k<tIntList.size(); ++k) {
                    gpxExtensions.add(new ArrayList<GPXExtension>());
                }
                gpxExtensions.get(gpxExtensions.size()-1).add(new GPXExtension(gpxList.get(j), nextQueryResult, j));
                queryResult = nextQueryResult;
            }
            System.out.println("Nodes: " + nodes.size());
            System.out.println("GPX slots: " + gpxExtensions.size());
            int nNMatchedPoints = 0;
            for (List<GPXExtension> gpxExtension : gpxExtensions) {
                nNMatchedPoints += gpxExtension.size();
            }
            System.out.println("Matched GPX points: "+nNMatchedPoints);

            System.out.println(nodes);

            final TIntList realNodes = new TIntArrayList();

            List<List<GPXExtension>> realEdgeGpxExtensions = new ArrayList<List<GPXExtension>>();
            List<GPXExtension> oneBucketGPXExtensions = new ArrayList<GPXExtension>();
            for (int j=0; j<nodes.size(); ++j) {
                oneBucketGPXExtensions.addAll(gpxExtensions.get(j));
                int node = nodes.get(j);
                if (j>0 && node == nodes.get(j-1)) {
                    throw new RuntimeException();
                }
                if (node < nodeCount) {
                    if (realNodes.isEmpty() && j > 0 || (!realNodes.isEmpty()) && realNodes.get(realNodes.size()-1) == node) {
                        // Verfolge zum anderen Ende, adde das andere Ende als realNode
                        // und, wenn nicht am Anfang, eine *leere* GPX-Liste
                        if (!realNodes.isEmpty()) {
                            realEdgeGpxExtensions.add(new ArrayList<GPXExtension>());
                        }
                        EdgeIterator edgeIterator = explorer.setBaseNode(node);
                        while (edgeIterator.next() && edgeIterator.getAdjNode() != nodes.get(j-1));
                        int realNode = traverseToClosestRealAdj(explorer, edgeIterator);
                        if ((!realNodes.isEmpty()) && realNode == realNodes.get(realNodes.size()-1)) {
                            throw new RuntimeException();
                        }
                        realNodes.add(realNode);
                        // Vielleicht könnte man die Punkte auch eher dem Hinweg als dem Rückweg
                        // zuordnen, das wäre konsistenter mit der Abschlusskante.
                        // Andererseits muss man vermutlich ohnehin eigentlich gerichtete Kanten unterscheiden.
                    }
                    realEdgeGpxExtensions.add(oneBucketGPXExtensions);
                    oneBucketGPXExtensions = new ArrayList<GPXExtension>();
                    if ((!realNodes.isEmpty()) && node == realNodes.get(realNodes.size()-1)) {
                        throw new RuntimeException();
                    }
                    realNodes.add(node);
                }
            }
            if (nodes.size() >= 2 && nodes.get(nodes.size()-1) >= nodeCount) {
                EdgeIterator edgeIterator = explorer.setBaseNode(nodes.get(nodes.size()-1));
                edgeIterator.next();
                int realNode = traverseToClosestRealAdj(explorer, edgeIterator);
                if (realNode == realNodes.get(realNodes.size()-1)) {
                    edgeIterator.next();
                    realNode = traverseToClosestRealAdj(explorer, edgeIterator);
                }
                realNodes.add(realNode);
                realEdgeGpxExtensions.add(oneBucketGPXExtensions);
            }
            System.out.println(realNodes);

            int n1 = realNodes.get(0);
            for (int j=1; j<realNodes.size(); j++) {
                int n2 = realNodes.get(j);
                EdgeIteratorState edge = GHUtility.getEdge(graph, n1, n2);
                if (edge == null) {
                    throw new RuntimeException();
                }
                edgeMatches.add(new EdgeMatch(edge, realEdgeGpxExtensions.get(j-1)));
                n1 = n2;
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

    private String hash(QueryResult sourcePosition, QueryResult targetPosition) {
        return sourcePosition.hashCode() + "_" + targetPosition.hashCode();
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
