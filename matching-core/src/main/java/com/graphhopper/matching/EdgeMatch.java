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

import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;

import java.util.List;

/**
 *
 * @author Peter Karich
 */
public class EdgeMatch {

    private final EdgeIteratorState edgeState;
    private final List<GPXExtension> gpxExtensions;
    private final PointList wayGeometry;

    public EdgeMatch(EdgeIteratorState edgeState, List<GPXExtension> gpxExtension, PointList wayGeometry) {
        this.edgeState = edgeState;
        this.wayGeometry = wayGeometry;

        if (edgeState == null) {
            throw new IllegalStateException("Cannot fetch null EdgeState");
        }

        this.gpxExtensions = gpxExtension;
        if (this.gpxExtensions == null) {
            throw new IllegalStateException("extension list cannot be null");
        }
    }

    public EdgeMatch(EdgeIteratorState edgeState, List<GPXExtension> gpxExtension){
        this(edgeState, gpxExtension, null);
    }

    public boolean isEmpty() {
        return gpxExtensions.isEmpty();
    }

    public EdgeIteratorState getEdgeState() {
        return edgeState;
    }

    public List<GPXExtension> getGpxExtensions() {
        return gpxExtensions;
    }

    public double getMinDistance() {
        if (isEmpty()) {
            throw new IllegalStateException("No minimal distance for " + edgeState);
        }

        double min = Double.MAX_VALUE;
        for (GPXExtension gpxExt : gpxExtensions) {
            if (gpxExt.queryResult.getQueryDistance() < min) {
                min = gpxExt.queryResult.getQueryDistance();
            }
        }
        return min;
    }

    /**
     * For OSM a way is often a curve not just a straight line. These nodes are called pillar nodes
     * and are between tower nodes (which are used for routing), they are necessary to have a more
     * exact geometry. Updates to the returned list are not reflected in the graph, for that you've
     * to use setWayGeometry.
     * <p>
     * @param mode can be <ul> <li>0 = only pillar nodes, no tower nodes</li> <li>1 = inclusive the
     * base tower node only</li> <li>2 = inclusive the adjacent tower node only</li> <li>3 =
     * inclusive the base and adjacent tower node</li> </ul>
     * @return pillar nodes
     */
    public PointList getWayGeometry(int mode){
        if(wayGeometry != null)
            switch (mode){
                case 0:
                    return wayGeometry.copy(1,wayGeometry.size() - 1);
                case 1:
                    return wayGeometry.copy(0,wayGeometry.size() - 1);
                case 2:
                    return wayGeometry.copy(1,wayGeometry.size());
                case 3:
                default:
                    return wayGeometry;
            }
        else
            return edgeState.fetchWayGeometry(mode);
    }

    public PointList getWayGeometry() {
        return getWayGeometry(3);
    }

    @Override
    public String toString() {
        return "edge:" + edgeState + ", extensions:" + gpxExtensions;
    }
}
