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
package com.graphhopper.matching.http;

import org.json.JSONArray;
import org.json.JSONObject;

import com.graphhopper.matching.MatchedEdge;
import com.graphhopper.matching.TimeStep;
import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.Candidate;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.MatchSequence;
import com.graphhopper.routing.Path;
import com.graphhopper.util.PointList;

/**
 * Transform MatchResult in JSON Object with following structure:
 * <pre>
 * { "diary": {
 *   "routes": [
 *     { "links": [
 *        { "geometry": String,
 *          "wpts": [{ "x": Number, "y": Number, "timestamp": Number }]
 *        }]
 *     }]
 * }}
 * </pre>
 */
public class MatchResultToJson {

    protected MatchResult result;

    public MatchResultToJson(MatchResult result) {
        this.result = result;
    }

    public JSONObject exportTo() {
        JSONObject root = new JSONObject();
        JSONObject diary = new JSONObject();
        JSONArray entries = new JSONArray();
        for (MatchSequence matchSequence: result.sequences) {
        	JSONObject route = new JSONObject();
        	JSONArray links = new JSONArray();
        	
        	// add sequence geometry
        	int emIndex = 0;
        	for (MatchedEdge matchEdge: matchSequence.matchEdges) {
            	JSONObject link = new JSONObject();
                JSONObject geometry = new JSONObject();
                PointList pointList = matchEdge.edge.fetchWayGeometry(emIndex == 0 ? 3 : 2);

                if (pointList.size() < 2) {
                    geometry.put("coordinates", pointList.toGeoJson().get(0));
                    geometry.put("type", "Point");
                } else {
                    geometry.put("coordinates", pointList.toGeoJson());
                    geometry.put("type", "LineString");
                }

                link.put("id", matchEdge.edge.getEdge());
                link.put("geometry", geometry.toString());
                System.out.println(matchEdge.edge.getName());

        	
	        	// add waypoints:
	            JSONArray wpts = new JSONArray();
	            link.put("wpts", wpts);
//	            for (SequenceState<Candidate, TimeStep, Path> step : matchSequence.matchedSequence) {
//	                JSONObject wpt = new JSONObject();
//	                wpt.put("x", step.state.getQueryResult().getSnappedPoint().lon);
//	                wpt.put("y", step.state.getQueryResult().getSnappedPoint().lat);
//	                wpt.put("timestamp", step.observation.gpxEntry.getTime());
//	                wpts.put(wpt);
//	            }
	            links.put(link);
                emIndex++;
	        }
	        route.put("links", links);
	        entries.put(route);
        }
        diary.put("entries", entries);
        root.put("diary", diary);
        return root;
    }
}
