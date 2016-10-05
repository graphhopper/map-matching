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
package com.graphhopper.matching.tools;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.PathWrapper;
import com.graphhopper.coll.GHBitSet;
import com.graphhopper.matching.LocationIndexMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.*;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.GHPoint3D;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.List;
import java.util.ArrayList;
import java.util.Date;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.TreeMap;

/**
 * @author Peter Karisch
 * @author kodonnell
 */
public class Measurement {
    private static final Logger logger = LoggerFactory.getLogger(Measurement.class);
    private final Map<String, String> properties = new TreeMap<String, String>();
    private long seed;
    private int count;

    public static void main(String[] strs) {
        new Measurement().start(CmdArgs.read(strs));
    }

    // creates properties file in the format key=value
    // Every value is one y-value in a separate diagram with an identical x-value for every Measurement.start call
    void start(CmdArgs args) {
        String graphLocation = args.get("graph.location", "");
        String propLocation = args.get("measurement.location", "");
        if (Helper.isEmpty(propLocation))
            propLocation = "measurement" + new SimpleDateFormat("yyyy-MM-dd_HH_mm_ss").format(new Date()) + ".properties";

        seed = args.getLong("measurement.seed", 123);
        String gitCommit = args.get("measurement.gitinfo", "");
        count = args.getInt("measurement.count", 5000);

        GraphHopper hopper = new GraphHopperOSM();
        hopper.init(args).forDesktop();
        hopper.getCHFactoryDecorator().setDisablingAllowed(true);
        hopper.importOrLoad();
        GraphHopperStorage g = hopper.getGraphHopperStorage();
        String vehicleStr = args.get("graph.flag_encoders", "car");
        FlagEncoder encoder = hopper.getEncodingManager().getEncoder(vehicleStr);
        Weighting weighting = hopper.getCHFactoryDecorator().getWeightings().get(0);
        

        GraphHopperStorage graph = hopper.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph, (LocationIndexTree) hopper.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);
        
        StopWatch sw = new StopWatch().start();
        try {
//            maxNode = g.getNodes();
//            GHBitSet allowedEdges = printGraphDetails(g, vehicleStr);
            boolean isCH = false;
            printLocationIndexMatchQuery(g, locationIndex);
            printTimeOfMapMatchQuery(hopper, mapMatching, g);
//            printMiscUnitPerfTests(g, isCH, encoder, count * 100, allowedEdges);
//            printLocationIndexQuery(g, hopper.getLocationIndex(), count);

//            printTimeOfRouteQuery(hopper, isCH, count / 20, "routing", vehicleStr, true);

            System.gc();

//            CHGraph lg = g.getGraph(CHGraph.class, weighting);
//            fillAllowedEdges(lg.getAllEdges(), allowedEdges);
//            isCH = true;
//            printMiscUnitPerfTests(lg, isCH, encoder, count * 100, allowedEdges);
//            printTimeOfRouteQuery(hopper, isCH, count, "routingCH", vehicleStr, true);
//            printTimeOfRouteQuery(hopper, isCH, count, "routingCH_no_instr", vehicleStr, false);
            logger.info("store into " + propLocation);
        } catch (Exception ex) {
            logger.error("Problem while measuring " + graphLocation, ex);
            put("error", ex.toString());
        } finally {
//            put("measurement.gitinfo", gitCommit);
            put("measurement.count", count);
            put("measurement.seed", seed);
            put("measurement.time", sw.stop().getTime());
            System.gc();
            put("measurement.totalMB", Helper.getTotalMB());
            put("measurement.usedMB", Helper.getUsedMB());
            try {
                store(new FileWriter(propLocation));
            } catch (IOException ex) {
                logger.error("Problem while storing properties " + graphLocation + ", " + propLocation, ex);
            }
        }
    }

    
    private void printLocationIndexMatchQuery(Graph g, final LocationIndexMatch idx) {
        final BBox bbox = g.getBounds();
        final double latDelta = bbox.maxLat - bbox.minLat;
        final double lonDelta = bbox.maxLon - bbox.minLon;
        final Random rand = new Random(seed);
        MiniPerfTest miniPerf = new MiniPerfTest() {
            @Override
            public int doCalc(boolean warmup, int run) {
                double lat = rand.nextDouble() * latDelta + bbox.minLat;
                double lon = rand.nextDouble() * lonDelta + bbox.minLon;
                int val = idx.findNClosest(lat, lon, EdgeFilter.ALL_EDGES, rand.nextDouble() * 500).size();
                return val;
            }
        }.setIterations(count).start();
        print("location_index_match", miniPerf);
    }

    private void printTimeOfMapMatchQuery(final GraphHopper hopper, final MapMatching mapMatching, Graph g) {
    	
    	// pick random endpoints to create a route, then pick random points from the route,
    	// and then run the random points through map-matching.
    	final BBox bbox = g.getBounds();
        final double latDelta = bbox.maxLat - bbox.minLat;
        final double lonDelta = bbox.maxLon - bbox.minLon;
        final Random rand = new Random(seed);
        DistanceCalcEarth distCalc = new DistanceCalcEarth();
        mapMatching.setMaxVisitedNodes((int) 1e10); 
        MiniPerfTest miniPerf = new MiniPerfTest() {
            @Override
            public int doCalc(boolean warmup, int run) {
            	boolean foundPath = false;
            	while (!foundPath) {
		            double lat0 = rand.nextDouble() * latDelta + bbox.minLat;
		            double lon0 = rand.nextDouble() * lonDelta + bbox.minLon;
		            double lat1 = rand.nextDouble() * latDelta + bbox.minLat;
		            double lon1 = rand.nextDouble() * lonDelta + bbox.minLon;
		            double sampleProportion = rand.nextDouble();
		            GHResponse r = hopper.route(new GHRequest(lat0, lon0, lat1, lon1));
		            if (!r.hasErrors()) {
		            	foundPath = true;
		            	long t = 0;
		                List<GPXEntry> mock = new ArrayList<GPXEntry>();
		                GHPoint prev = null;
		                PointList points = r.getBest().getPoints();
		            	for (GHPoint p : points) {
		            		if (null != prev && rand.nextDouble() < sampleProportion) {
		            			double dx = distCalc.calcDist(prev.lat, prev.lon, p.lat, p.lon);
		            			double speedKPH = rand.nextDouble() * 100;
		            			double dt = (dx / 1000) / speedKPH * 3600000;
		            			t += (long) dt;
		            			mock.add(new GPXEntry(p, t));
		            		}
		            	}		            	
		            	// now match:
		            	if (mock.size() > 2) {
		            		MatchResult match = mapMatching.doWork(mock);		            		
		            	}
		            	// return something else?
		            	return 0;
		            }		            
            	}
				return 0;
            }
        }.setIterations(count).start();
        print("map_match", miniPerf);
    	
    }
 

    void print(String prefix, MiniPerfTest perf) {
        logger.info(prefix + ": " + perf.getReport());
        put(prefix + ".sum", perf.getSum());
//        put(prefix+".rms", perf.getRMS());
        put(prefix + ".min", perf.getMin());
        put(prefix + ".mean", perf.getMean());
        put(prefix + ".max", perf.getMax());
    }

    void put(String key, Object val) {
        // convert object to string to make serialization possible
        properties.put(key, "" + val);
    }

    private void store(FileWriter fileWriter) throws IOException {
        for (Entry<String, String> e : properties.entrySet()) {
            fileWriter.append(e.getKey());
            fileWriter.append("=");
            fileWriter.append(e.getValue());
            fileWriter.append("\n");
        }
        fileWriter.flush();
    }
}
