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

import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FootFlagEncoder;
import com.graphhopper.storage.GraphStorage;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.GPXEntry;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.List;

/**
 * @author Christoph Balthaus
 */
public class MapMatchingDontCutTheLoopTest {

    public static final String GPX_IN_FILE = "./src/test/resources/GraphhopperBerlin(loop)Dgz-ring.gpx";
    public static final String GPX_OUT_FILE = "./src/test/resources/GraphhopperBerlin(loop)Dgz-ring-result.gpx";
    // enable turn cost in encoder:
    private static final FootFlagEncoder encoder = new FootFlagEncoder(5, 5);
    private static final TestGraphHopper hopper = new TestGraphHopper();

    @BeforeClass
    public static void doImport() {
        hopper.setOSMFile("./map-data/Berlin_DGZ-Ring.osm.xml");
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

        List<GPXEntry> gpxEntryList = new GPXFile().doImport(GPX_IN_FILE).getEntries();

        MatchResult mr = mapMatching.doWork(gpxEntryList);

        System.out.println("\tmatches:\t" + mr.getEdgeMatches().size());
        System.out.println("\tgpx length:\t" + mr.getGpxEntriesLength() + " vs " + mr.getMatchLength());
        System.out.println("\tgpx time:\t" + mr.getGpxEntriesMillis() / 1000f + " vs " + mr.getMatchMillis() / 1000f);

        System.out.println("\texport results to:" + GPX_OUT_FILE);

        System.out.println(Helper.getStreets(mr.getEdgeMatches()));

        new GPXFile(mr).doExport(GPX_OUT_FILE);


    }

}
