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

import com.fasterxml.jackson.dataformat.xml.XmlMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.matching.gpx.Gpx;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.util.Helper;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import static com.graphhopper.matching.MapMatchingTest.fetchStreets;
import static org.junit.Assert.assertEquals;

/**
 * @author Peter Karich
 */
public class MapMatching3Test {

    private static final String DIR = "../target/mapmatchingtest-147";

    @BeforeClass
    public static void setUp() {
        Helper.removeDir(new File(DIR));
    }

    @Test
    public void testIssue147() throws IOException {
        CarFlagEncoder encoder = new CarFlagEncoder() {
            {
                // TODO NOW this test is only green for a very low value like 2
                defaultSpeedMap.put("motorway_link", 50);
            }
        };
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setDataReaderFile("../map-data/map-issue147.osm.gz");
        hopper.setGraphHopperLocation(DIR);
        EncodingManager em = new EncodingManager.Builder(4).add(encoder).build();
        hopper.setEncodingManager(em);
        hopper.getCHFactoryDecorator().setDisablingAllowed(true);
        hopper.importOrLoad();

        AlgorithmOptions opts = AlgorithmOptions.start().build();
        MapMatching mapMatching = new MapMatching(hopper, opts);

        Gpx gpx = new XmlMapper().readValue(getClass().getResourceAsStream("/issue-147.gpx"), Gpx.class);
        MatchResult mr = mapMatching.doWork(gpx.trk.get(0).getEntries());

        // always match motorway
        assertEquals(Arrays.asList("I 90", "I 90", "I 90", "I 90", "I 90"), fetchStreets(mr.getEdgeMatches()));
        assertEquals(3047, mr.getMatchLength(), 1);
    }
}
