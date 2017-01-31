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

import com.graphhopper.util.EdgeIteratorState;

/**
 * A MatchEdge is an edge on a MatchSequence - i.e. one of the edges making up the final map-matched
 * route. This includes the time at which the edge was travelled.
 * 
 * @author kodonnell
 */
public class MatchEdge {

    /**
     * The actual (GraphHopper) edge.
     */
    public final EdgeIteratorState edge;
    /**
     * Time (inclusive, in milliseconds) when first was on this edge. -1 if not set.
     */
    public final long fromTime;
    /**
     * Time (exclusive, in milliseconds) when last was on this edge. -1 if not set.
     */
    public final long toTime;

    /**
     * Create a MatchEdge
     * 
     * @param edge
     * @param fromTime
     * @param toTime
     */
    public MatchEdge(EdgeIteratorState edge, long fromTime, long toTime) {
        assert edge != null : "edge should not be null";
        this.edge = edge;
        this.fromTime = fromTime;
        this.toTime = toTime;
    }
}
