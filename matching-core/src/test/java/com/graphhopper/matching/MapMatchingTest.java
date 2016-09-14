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

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.PathWrapper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.*;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.BreadthFirstSearch;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.Helper;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PathMerger;
import com.graphhopper.util.Translation;
import com.graphhopper.util.TranslationMap;
import com.graphhopper.util.shapes.GHPoint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.junit.AfterClass;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 *
 * @author Peter Karich
 */
public class MapMatchingTest {

    // disable turn restrictions in encoder:
    private static final CarFlagEncoder ENCODER = new CarFlagEncoder();
    private static final TestGraphHopper HOPPER = new TestGraphHopper();
    public final static TranslationMap SINGLETON = new TranslationMap().doImport();

    @BeforeClass
    public static void doImport() {
        HOPPER.setDataReaderFile("../map-data/leipzig_germany.osm.pbf");
        HOPPER.setGraphHopperLocation("../target/mapmatchingtest");
        HOPPER.setEncodingManager(new EncodingManager(ENCODER));
        HOPPER.getCHFactoryDecorator().setEnabled(false);
        // hopper.clean();
        HOPPER.importOrLoad();
    }

    @AfterClass
    public static void doClose() {
        HOPPER.close();
    }

    @Test
    public void testDoWork() {
        GraphHopperStorage graph = HOPPER.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) HOPPER.getLocationIndex());

        MapMatching mapMatching = new MapMatching(graph, locationIndex, ENCODER);
        mapMatching.setMeasurementErrorSigma(5);

        // printOverview(graph, hopper.getLocationIndex(), 51.358735, 12.360574, 500);
        // https://graphhopper.com/maps/?point=51.358735%2C12.360574&point=51.358594%2C12.360032&layer=Lyrk
        List<GPXEntry> inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.358735, 12.360574),
                new GHPoint(51.358594, 12.360032));

        MatchResult mr = mapMatching.doWork(inputGPXEntries);

        // make sure no virtual edges are returned
        int edgeCount = graph.getAllEdges().getMaxId();
        for (EdgeMatch em : mr.getEdgeMatches()) {
            assertTrue("result contains virtual edges:" + em.getEdgeState().toString(), em.getEdgeState().getEdge() < edgeCount);
        }

        // create street names
        assertEquals(Arrays.asList("Platnerstraße", "Platnerstraße", "Platnerstraße"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 1.5);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis());

        Path path = mapMatching.calcPath(mr);
        PathWrapper matchGHRsp = new PathWrapper();
        new PathMerger().doWork(matchGHRsp, Collections.singletonList(path), SINGLETON.get("en"));
        InstructionList il = matchGHRsp.getInstructions();

        assertEquals(il.toString(), 2, il.size());
        assertEquals("Platnerstraße", il.get(0).getName());

        // https://graphhopper.com/maps/?point=51.33099%2C12.380267&point=51.330689%2C12.380776&layer=Lyrk
        inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.33099, 12.380267),
                new GHPoint(51.330689, 12.380776));
        mr = mapMatching.doWork(inputGPXEntries);

        assertEquals(Arrays.asList("Windmühlenstraße", "Windmühlenstraße",
                "Bayrischer Platz", "Bayrischer Platz", "Bayrischer Platz"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), .1);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 1);

        path = mapMatching.calcPath(mr);
        matchGHRsp = new PathWrapper();
        new PathMerger().doWork(matchGHRsp, Collections.singletonList(path), SINGLETON.get("en"));
        il = matchGHRsp.getInstructions();

        assertEquals(il.toString(), 3, il.size());
        assertEquals("Windmühlenstraße", il.get(0).getName());
        assertEquals("Bayrischer Platz", il.get(1).getName());

        // full path
        // https://graphhopper.com/maps/?point=51.377781%2C12.338333&point=51.323317%2C12.387085&layer=Lyrk
        inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.377781, 12.338333),
                new GHPoint(51.323317, 12.387085));
        mapMatching = new MapMatching(graph, locationIndex, ENCODER);
        mapMatching.setMeasurementErrorSigma(5);
        // new GPXFile(inputGPXEntries).doExport("test-input.gpx");
        mr = mapMatching.doWork(inputGPXEntries);
        // new GPXFile(mr).doExport("test.gpx");
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 0.5);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 200);
        assertEquals(138, mr.getEdgeMatches().size());

        // TODO full path with 20m distortion
        // TODO full path with 40m distortion
    }

    @Test
    public void testSmallSeparatedSearchDistance() {
        GraphHopperStorage graph = HOPPER.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) HOPPER.getLocationIndex());

        MapMatching mapMatching = new MapMatching(graph, locationIndex, ENCODER);
        
        // import sample where two GPX entries are on one edge which is longer than 'separatedSearchDistance' aways (66m)
        // https://graphhopper.com/maps/?point=51.359723%2C12.360108&point=51.359621%2C12.360243&point=51.358591%2C12.358584&point=51.358189%2C12.357876&point=51.358007%2C12.357403&point=51.358627%2C12.356612&point=51.358709%2C12.356511&locale=en-GB&vehicle=car&weighting=fastest&elevation=true&use_miles=false&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour3-with-long-edge.gpx").getEntries();
        
        // fuzzy match: we exclude the endpoints with large sigma:        
        mapMatching.setMeasurementErrorSigma(50);
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        assertEquals(Arrays.asList("Weinligstraße", "Weinligstraße", "Fechnerstraße"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 16);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 4500);

        // more exact match: include the endpoints:
        mapMatching.setMeasurementErrorSigma(5);
        mr = mapMatching.doWork(inputGPXEntries);
        assertEquals(Arrays.asList("Marbachstraße", "Weinligstraße", "Weinligstraße",
        		"Fechnerstraße", "Fechnerstraße"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 11);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 3000);
        
    }

    @Test
    public void testLoop() {
        GraphHopperStorage graph = HOPPER.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) HOPPER.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, ENCODER);

        // printOverview(graph, hopper.getLocationIndex(), 51.345796,12.360681, 1000);
        // https://graphhopper.com/maps/?point=51.343657%2C12.360708&point=51.344982%2C12.364066&point=51.344841%2C12.361223&point=51.342781%2C12.361867&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour2-with-loop.gpx").getEntries();
        mapMatching.setMeasurementErrorSigma(20);
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        // new GPXFile(mr).doExport("testLoop-matched.gpx");

        // Expected is ~800m. If too short like 166m then the loop was skipped    
        assertEquals(Arrays.asList("Gustav-Adolf-Straße", "Gustav-Adolf-Straße",
                "Gustav-Adolf-Straße", "Leibnizstraße", "Hinrichsenstraße",
                "Hinrichsenstraße", "Tschaikowskistraße", "Tschaikowskistraße"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 5);
        // TODO why is there such a big difference for millis?
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 6000);
    }

    @Test
    public void testLoop2() {
        GraphHopperStorage graph = HOPPER.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) HOPPER.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, ENCODER);
        // NOTE: larger sigma leads to odd route - it looks like the map is incorrect when compared to Google Maps.
        mapMatching.setMeasurementErrorSigma(20);
        // https://graphhopper.com/maps/?point=51.342439%2C12.361615&point=51.343719%2C12.362784&point=51.343933%2C12.361781&point=51.342325%2C12.362607&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour-with-loop.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        assertEquals(Arrays.asList("Jahnallee, B 87, B 181", "Jahnallee, B 87, B 181",
                "Jahnallee, B 87, B 181", "Jahnallee, B 87, B 181", "Funkenburgstraße",
                "Gustav-Adolf-Straße", "Tschaikowskistraße", "Jahnallee, B 87, B 181",
                "Lessingstraße", "Lessingstraße"),
                fetchStreets(mr.getEdgeMatches()));
    }

    @Test
    public void testUTurns() {
        GraphHopperStorage graph = HOPPER.getGraphHopperStorage();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) HOPPER.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, ENCODER);

        // https://graphhopper.com/maps/?point=51.343618%2C12.360772&point=51.34401%2C12.361776&point=51.343977%2C12.362886&point=51.344734%2C12.36236&point=51.345233%2C12.362055&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour4-with-uturn.gpx").getEntries();
        
        // exclude U-turn
        mapMatching.setMeasurementErrorSigma(50);
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        assertEquals(Arrays.asList("Gustav-Adolf-Straße", "Gustav-Adolf-Straße", "Funkenburgstraße"), fetchStreets(mr.getEdgeMatches()));
                
        // include U-turn
        mapMatching.setMeasurementErrorSigma(10);
        mr = mapMatching.doWork(inputGPXEntries);
        assertEquals(Arrays.asList("Gustav-Adolf-Straße", "Gustav-Adolf-Straße",
                "Funkenburgstraße", "Funkenburgstraße", "Funkenburgstraße", "Funkenburgstraße"),
                fetchStreets(mr.getEdgeMatches()));
    }

    List<String> fetchStreets(List<EdgeMatch> emList) {
        List<String> list = new ArrayList<String>();
        int prevNode = -1;
        List<String> errors = new ArrayList<String>();
        for (EdgeMatch em : emList) {
            String str = em.getEdgeState().getName();// + ":" + em.getEdgeState().getBaseNode() + "->" + em.getEdgeState().getAdjNode();
            list.add(str);
            if (prevNode >= 0) {
                if (em.getEdgeState().getBaseNode() != prevNode) {
                    errors.add(str);
                }
            }
            prevNode = em.getEdgeState().getAdjNode();
        }

        if (!errors.isEmpty()) {
            throw new IllegalStateException("Errors:" + errors);
        }
        return list;
    }

    private List<GPXEntry> createRandomGPXEntries(GHPoint start, GHPoint end) {
        HOPPER.route(new GHRequest(start, end).setWeighting("fastest"));
        return HOPPER.getEdges(0);
    }

    private void printOverview(GraphHopperStorage graph, LocationIndex locationIndex,
            final double lat, final double lon, final double length) {
        final NodeAccess na = graph.getNodeAccess();
        int node = locationIndex.findClosest(lat, lon, EdgeFilter.ALL_EDGES).
                getClosestNode();
        final EdgeExplorer explorer = graph.createEdgeExplorer();
        new BreadthFirstSearch() {

            double currDist = 0;

            @Override
            protected boolean goFurther(int nodeId) {
                double currLat = na.getLat(nodeId);
                double currLon = na.getLon(nodeId);
                currDist = Helper.DIST_PLANE.calcDist(currLat, currLon, lat, lon);
                return currDist < length;
            }

            @Override
            protected boolean checkAdjacent(EdgeIteratorState edge) {
                System.out.println(edge.getBaseNode() + "->" + edge.getAdjNode()
                        + " (" + Math.round(edge.getDistance()) + "): " + edge.getName()
                        + "\t\t , distTo:" + currDist);
                return true;
            }
        }.start(explorer, node);
    }

    // use a workaround to get access to 
    static class TestGraphHopper extends GraphHopperOSM {

        private List<Path> paths;

        List<GPXEntry> getEdges(int index) {
            Path path = paths.get(index);
            Translation tr = getTranslationMap().get("en");
            InstructionList instr = path.calcInstructions(tr);
            // GPXFile.write(path, "calculated-route.gpx", tr);
            return instr.createGPXList();
        }

        @Override
        public List<Path> calcPaths(GHRequest request, GHResponse rsp) {
            paths = super.calcPaths(request, rsp);
            return paths;
        }
    }
}
