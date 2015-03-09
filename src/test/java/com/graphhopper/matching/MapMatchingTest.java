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

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.*;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.GraphStorage;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.BreadthFirstSearch;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.GHUtility;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.Helper;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.Translation;
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

    // enable turn cost in encoder:
    private static final CarFlagEncoder encoder = new CarFlagEncoder(5, 5, 3);
    private static final TestGraphHopper hopper = new TestGraphHopper();

    @BeforeClass
    public static void doImport() {
        hopper.setOSMFile("./map-data/leipzig_germany.osm.pbf");
        hopper.setGraphHopperLocation("./target/mapmatchingtest");
        hopper.setEncodingManager(new EncodingManager(encoder));
        hopper.setCHEnable(false);
        // hopper.clean();
        hopper.importOrLoad();
    }

    @AfterClass
    public static void doClose() {
        hopper.close();
    }

    @Test
    public void testDoWork() {
        GraphStorage graph = hopper.getGraph();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex());

        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);

        // printOverview(graph, hopper.getLocationIndex(), 51.358735, 12.360574, 500);
        List<GPXEntry> inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.358735, 12.360574),
                new GHPoint(51.358594, 12.360032));
        MatchResult mr = mapMatching.doWork(inputGPXEntries);

        // make sure no virtual edges are returned
        int edgeCount = graph.getAllEdges().getCount();
        for (EdgeMatch em : mr.getEdgeMatches()) {
            assertTrue("result contains virtual edges:" + em.getEdgeState().toString(), em.getEdgeState().getEdge() < edgeCount);
        }

        // create street names
        assertEquals(Arrays.asList("Platnerstraße:19163->20072", "Platnerstraße:20072->20071",
                "Platnerstraße:20071->1551"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 1.5);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis());

        inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.33099, 12.380267),
                new GHPoint(51.330689, 12.380776));
        mr = mapMatching.doWork(inputGPXEntries);

        assertEquals(Arrays.asList("Windmühlenstraße:22135->15546", "Windmühlenstraße:15546->22056",
                "Bayrischer Platz:22056->2657", "Bayrischer Platz:2657->22046", "Bayrischer Platz:22046->22058"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), .1);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis());

        // full path
        inputGPXEntries = createRandomGPXEntries(
                new GHPoint(51.377781, 12.338333),
                new GHPoint(51.323317, 12.387085));
        mapMatching = new MapMatching(graph, locationIndex, encoder);

        // new GPXFile(inputGPXEntries).doExport("test-input.gpx");
        mr = mapMatching.doWork(inputGPXEntries);
        // new GPXFile(mr).doExport("test.gpx");

        // System.out.println(fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 0.5);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 20);
        assertEquals(138, mr.getEdgeMatches().size());

        // TODO full path with 20m distortion
        // TODO full path with 40m distortion
    }

    @Test
    public void testSmallSeparatedSearchDistance() {
        GraphStorage graph = hopper.getGraph();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex());

        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);
        mapMatching.setSeparatedSearchDistance(30);

        // import sample where two GPX entries are on one edge which is longer than 'separatedSearchDistance' aways (66m)
        // https://graphhopper.com/maps/?point=51.359723%2C12.360108&point=51.358748%2C12.358798&point=51.358001%2C12.357597&point=51.358709%2C12.356511&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour3-with-long-edge.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        // new GPXFile(mr).doExport("testSmallSeparatedSearchDistance.gpx");
        assertEquals(Arrays.asList("Marbachstraße:5592->2195", "Weinligstraße:2195->2196",
                "Weinligstraße:2196->5598", "Fechnerstraße:5598->5595", "Fechnerstraße:5595->16615"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 11);
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 1000);
    }

    @Test
    public void testLoop() {
        GraphStorage graph = hopper.getGraph();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);

        // printOverview(graph, hopper.getLocationIndex(), 51.345796,12.360681, 1000);
        // https://graphhopper.com/maps/?point=51.343657%2C12.360708&point=51.344982%2C12.364066&point=51.344841%2C12.361223&point=51.342781%2C12.361867&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour2-with-loop.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);
        // new GPXFile(mr).doExport("testLoop-matched.gpx");

        // Expected is ~800m. If too short like 166m then the loop was skipped        
        assertEquals(Arrays.asList("Gustav-Adolf-Straße:1078->1393", "Gustav-Adolf-Straße:1393->205",
                "Gustav-Adolf-Straße:205->204", "Leibnizstraße:204->206", "Hinrichsenstraße:206->1381",
                "Hinrichsenstraße:1381->1392", "Tschaikowskistraße:1392->1393", "Tschaikowskistraße:1393->1394"),
                fetchStreets(mr.getEdgeMatches()));
        assertEquals(mr.getGpxEntriesLength(), mr.getMatchLength(), 1);
        // TODO why is there such a big difference for millis?
        assertEquals(mr.getGpxEntriesMillis(), mr.getMatchMillis(), 6000);
    }

    @Test
    public void testLoop2() {
        GraphStorage graph = hopper.getGraph();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);
        mapMatching.setSeparatedSearchDistance(200);
        // TODO wrong direction?
        // https://graphhopper.com/maps/?point=51.342439%2C12.361615&point=51.343719%2C12.362784&point=51.343933%2C12.361781&point=51.342325%2C12.362607&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour-with-loop.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);

        // new GPXFile(mr).doExport("testLoop2-matched.gpx");
        assertEquals(Arrays.asList("Jahnallee, B 87, B 181:16829->1394", "Jahnallee, B 87, B 181:1394->1680",
                "Jahnallee, B 87, B 181:1680->21712", "Jahnallee, B 87, B 181:21712->207", "Funkenburgstraße:207->205",
                "Gustav-Adolf-Straße:205->1393", "Tschaikowskistraße:1393->1394", "Jahnallee, B 87, B 181:1394->1680",
                "Lessingstraße:1680->21711", "Lessingstraße:21711->1679"),
                fetchStreets(mr.getEdgeMatches()));
    }

    @Test
    public void testAvoidOffRoadUTurns() {
        GraphStorage graph = hopper.getGraph();
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex());
        MapMatching mapMatching = new MapMatching(graph, locationIndex, encoder);
        mapMatching.setSeparatedSearchDistance(200);

        // https://graphhopper.com/maps/?point=51.343618%2C12.360772&point=51.34401%2C12.361776&point=51.343977%2C12.362886&point=51.344734%2C12.36236&point=51.345233%2C12.362055&layer=Lyrk
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport("./src/test/resources/tour4-with-uturn.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);

        assertEquals(Arrays.asList("Gustav-Adolf-Straße:1078->1393", "Gustav-Adolf-Straße:1393->205",
                "Funkenburgstraße:205->1381", "Funkenburgstraße:1381->1379"),
                fetchStreets(mr.getEdgeMatches()));
    }

    @Test
    public void testCheckOrRepair() {
        Graph graph = hopper.getGraph();
        MapMatching mm = new MapMatching(graph, null, null);
        List<EdgeMatch> list = new ArrayList<EdgeMatch>();

        // System.out.println(GHUtility.getNeighbors(graph.createEdgeExplorer().setBaseNode(24627)));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 0, 24627), Collections.<GPXExtension>emptyList()));

        // incorrect orientation
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 880, 24627), Collections.<GPXExtension>emptyList()));
        // duplicate edge        
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 24627, 880), Collections.<GPXExtension>emptyList()));

        try {
            mm.checkOrCleanup(list, false, true);
            assertTrue(false);
        } catch (Exception ex) {

        }

        // repair
        List<EdgeMatch> res = mm.checkOrCleanup(list, true, true);
        // 2nd edge is flipped and 3rd is removed as a duplicate
        assertEquals(Arrays.asList("A 9:0->24627", "A 9:24627->880"), fetchStreets(res));

        // now repaired list must not throw an exception
        mm.checkOrCleanup(res, false, true);
    }

    @Test
    public void testRepairSkipUTurn() {
        Graph graph = hopper.getGraph();
        MapMatching mm = new MapMatching(graph, null, null);
        List<EdgeMatch> list = new ArrayList<EdgeMatch>();

        list.add(new EdgeMatch(GHUtility.getEdge(graph, 0, 24627), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 24627, 880), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 880, 24627), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 24627, 880), Collections.<GPXExtension>emptyList()));

        // Repair needed to skip UTurn
        List<EdgeMatch> res = mm.checkOrCleanup(list, true, true);
        // two edges are removed
        assertEquals(Arrays.asList("A 9:0->24627", "A 9:24627->880"), fetchStreets(res));

        // now repaired list must not throw an exception
        mm.checkOrCleanup(res, false, true);
    }

    @Test
    public void testUTurn() {
        Graph graph = hopper.getGraph();
        MapMatching mm = new MapMatching(graph, null, null);
        List<EdgeMatch> list = new ArrayList<EdgeMatch>();

        list.add(new EdgeMatch(GHUtility.getEdge(graph, 0, 24627), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 24627, 880), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 880, 24627), Collections.<GPXExtension>emptyList()));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 24627, 880), Collections.<GPXExtension>emptyList()));

        // Repair not needed
        List<EdgeMatch> res = mm.checkOrCleanup(list, false, false);
        // Accept UTurns
        assertEquals(Arrays.asList("A 9:0->24627", "A 9:24627->880", "A 9:880->24627", "A 9:24627->880"), fetchStreets(res));

        // now repaired list must not throw an exception
        mm.checkOrCleanup(res, false, false);
    }

    @Test
    public void testRepairUTurn() {
        Graph graph = hopper.getGraph();
        MapMatching mm = new MapMatching(graph, null, null);
        List<EdgeMatch> list = new ArrayList<EdgeMatch>();

        // System.out.println(GHUtility.getNeighbors(graph.createEdgeExplorer().setBaseNode(24627)));
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 0, 24627), Collections.<GPXExtension>emptyList()));

        // incorrect orientation
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 880, 24627), Collections.<GPXExtension>emptyList()));
        // accept Uturn
        list.add(new EdgeMatch(GHUtility.getEdge(graph, 880, 24627), Collections.<GPXExtension>emptyList()));

        // repair orientation
        List<EdgeMatch> res = mm.checkOrCleanup(list, true, false);
        // accept UTurn
        assertEquals(Arrays.asList("A 9:0->24627", "A 9:24627->880", "A 9:880->24627"), fetchStreets(res));

        // now repaired list must not throw an exception
        mm.checkOrCleanup(res, false, false);
    }


    List<String> fetchStreets(List<EdgeMatch> emList) {
        List<String> list = new ArrayList<String>();
        int prevNode = -1;
        List<String> errors = new ArrayList<String>();
        for (EdgeMatch em : emList) {
            String str = em.getEdgeState().getName() + ":" + em.getEdgeState().getBaseNode() + "->" + em.getEdgeState().getAdjNode();
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
        hopper.route(new GHRequest(start, end).setWeighting("fastest"));
        return hopper.getEdges(0);
    }

    private void printOverview(GraphStorage graph, LocationIndex locationIndex,
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
    static class TestGraphHopper extends GraphHopper {

        private List<Path> paths;

        List<GPXEntry> getEdges(int index) {
            Path path = paths.get(index);
            Translation tr = getTranslationMap().get("en");
            InstructionList instr = path.calcInstructions(tr);
            // GPXFile.write(path, "calculated-route.gpx", tr);
            return instr.createGPXList();
        }

        @Override
        protected List<Path> getPaths(GHRequest request, GHResponse rsp) {
            paths = super.getPaths(request, rsp);
            return paths;
        }
    }
}
