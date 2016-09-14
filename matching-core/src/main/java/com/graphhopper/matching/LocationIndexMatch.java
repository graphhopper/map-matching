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

import com.graphhopper.coll.GHBitSet;
import com.graphhopper.coll.GHTBitSet;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;

import gnu.trove.procedure.TIntProcedure;
import gnu.trove.set.hash.TIntHashSet;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 *
 * @author Peter Karich
 */
public class LocationIndexMatch extends LocationIndexTree {

    private static final Comparator<QueryResult> QR_COMPARATOR = new Comparator<QueryResult>() {
        @Override
        public int compare(QueryResult o1, QueryResult o2) {
            return Double.compare(o1.getQueryDistance(), o2.getQueryDistance());
        }
    };

    private final LocationIndexTree index;

    public LocationIndexMatch(GraphHopperStorage graph, LocationIndexTree index) {
        super(graph, graph.getDirectory());
        this.index = index;
    }

    /**
     * This method finds all the edges which are within a radial distance from a given point.
     * 
     *  @param maxDistance edges must be maxDistance or less from (queryLat, queryLon)     *  
     *  @return a list of such edges, sorted by the distance from the query point (closest first).
     */
    public List<QueryResult> findEdgesWithinRadius(final double queryLat, final double queryLon, final EdgeFilter edgeFilter, double maxDistance) {
    	return findEdgesWithinRadius(queryLat, queryLon, edgeFilter, 0, maxDistance);
    }

    /**
     * This method finds all the edges which are within a radial distance from a given point.
     * 
     *  @param minDistance edges must be minDistance or further from (queryLat, queryLon)
     *  @param maxDistance edges must be maxDistance or less from (queryLat, queryLon)     *  
     *  @return a list of such edges, sorted by the distance from the query point (closest first).
     */
    public List<QueryResult> findEdgesWithinRadius(final double queryLat, final double queryLon, final EdgeFilter edgeFilter, double minDistance, double maxDistance) {
            	
    	final double dLat = index.deltaLat;
    	final double dLon = index.deltaLon;
    	    	
        // implement a cheap priority queue via List, sublist and Collections.sort
        final List<QueryResult> queryResults = new ArrayList<QueryResult>();
        final TIntHashSet found = new TIntHashSet();

        // get boundaries:
        final GHPoint outerNorth = distCalc.projectCoordinate(queryLat, queryLon, maxDistance, 0);
        final GHPoint outerEast = distCalc.projectCoordinate(queryLat, queryLon, maxDistance, 90);
        final GHPoint outerSouth = distCalc.projectCoordinate(queryLat, queryLon, maxDistance, 180);
        final GHPoint outerWest = distCalc.projectCoordinate(queryLat, queryLon, maxDistance, 270);
        final double latMin = outerSouth.lat - dLat;
        final double latMax = outerNorth.lat + dLat;
        final double lonMin = outerWest.lon - dLon;
        final double lonMax = outerEast.lon + dLon;
        
        // get the radii in normed units:
        final double normedMinDistance = distCalc.calcNormalizedDist(minDistance);
        final double normedMaxDistance = distCalc.calcNormalizedDist(maxDistance);
        
        // get the min/max allowed radii: we add/remove the max tile dimension (i.e. it's diagonal), as this means that
        // we still get x, even in this case:
        // 
        // |-------|-------|
        // | .     |      x|
        // |     . |       |
        // |       |.      |
        // |-------|--.----|
        // |       |   .   |
        //
        // where the dots represent the radius (with center somewhere down to left), and the x represents a valid point. That is, we still
        // include the tile containing x, even though it's outside the radius.        
        final double deltaR = distCalc.calcDist(queryLat, queryLon, queryLat + dLat, queryLon + dLon);
        final double lowerBoundNormedMinDistance = distCalc.calcNormalizedDist(Math.max(0, minDistance - deltaR));
        final double upperBoundNormedMaxDistance = distCalc.calcNormalizedDist(maxDistance + deltaR);

        // loop through tiles, and only consider those within minInner/maxOuter radius. If they are, add all
        // the entries from that tile.
        for (double lat = latMin; lat <= latMax; lat += dLat) {
        	for (double lon = lonMin; lon <= lonMax; lon += dLon) {
        		// find the points here if they're in bounds (including tolerance):
        		double d = distCalc.calcNormalizedDist(queryLat, queryLon, lat, lon);
        		if (lowerBoundNormedMinDistance < d && d < upperBoundNormedMaxDistance) {
        			index.findNetworkEntriesSingleRegion(found, lat, lon);
        		}
        	}
        }
        
        // now loop through and filter to only include those which match the edgeFilter, and are (exactly)
        // within the inner/outer radius. 
        final GHBitSet exploredNodes = new GHTBitSet(new TIntHashSet(found));
        final EdgeExplorer explorer = graph.createEdgeExplorer(edgeFilter);

        found.forEach(new TIntProcedure() {

            @Override
            public boolean execute(int node) {
                new XFirstSearchCheck(queryLat, queryLon, exploredNodes, edgeFilter) {
                    @Override
                    protected double getQueryDistance() {
                        // do not skip search if distance is 0 or near zero (equalNormedDelta)
                        return Double.MAX_VALUE;
                    }

                    @Override
                    protected boolean check(int node, double normedDist, int wayIndex, EdgeIteratorState edge, QueryResult.Position pos) {                    	
                        if (normedMinDistance <= normedDist && normedDist <= normedMaxDistance) {
                        	
                        	// check we don't already have it:
                            for (int qrIndex = 0; qrIndex < queryResults.size(); qrIndex++) {
                                QueryResult qr = queryResults.get(qrIndex);
                                if (qr.getClosestEdge().getEdge() == edge.getEdge()) {
                                	return true;
                                }
                            }

                            // cool, let's add it:
                            QueryResult qr = new QueryResult(queryLat, queryLon);
                            qr.setQueryDistance(normedDist);
                            qr.setClosestNode(node);
                            qr.setClosestEdge(edge.detach(false));
                            qr.setWayIndex(wayIndex);
                            qr.setSnappedPosition(pos);
                            queryResults.add(qr);
                        }
                        return true;
                    }
                }.start(explorer, node);
                return true;
            }
        });

        // sorted list:
        Collections.sort(queryResults, QR_COMPARATOR);

        // denormalize distances and calculate the snapped point:
        for (QueryResult qr : queryResults) {
            if (qr.isValid()) {
                qr.setQueryDistance(distCalc.calcDenormalizedDist(qr.getQueryDistance()));
                qr.calcSnappedPoint(distCalc);
            } else {
                throw new IllegalStateException("Invalid QueryResult should not happen here: " + qr);
            }
        }
        return queryResults;
    }
}