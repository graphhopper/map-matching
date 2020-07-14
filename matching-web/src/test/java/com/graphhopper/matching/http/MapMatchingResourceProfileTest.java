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

import com.fasterxml.jackson.databind.JsonNode;
import com.graphhopper.config.CHProfile;
import com.graphhopper.config.LMProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.http.WebHelper;
import com.graphhopper.util.Helper;
import io.dropwizard.testing.junit.DropwizardAppRule;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.ClassRule;
import org.junit.Test;

import javax.ws.rs.client.Entity;
import javax.ws.rs.core.Response;
import java.io.File;
import java.util.Arrays;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

/**
 * @author easbar
 */
public class MapMatchingResourceProfileTest {

    private static final String DIR = "../target/mapmatchingtest";

    private static MapMatchingServerConfiguration createConfig() {
        MapMatchingServerConfiguration config = new MapMatchingServerConfiguration();
        config.getGraphHopperConfiguration().
                putObject("graph.flag_encoders", "car|turn_costs=true").
                putObject("datareader.file", "../map-data/leipzig_germany.osm.pbf").
                putObject("graph.location", DIR).
                setProfiles(Arrays.asList(
                        new Profile("car").setVehicle("car").setWeighting("fastest").setTurnCosts(true),
                        new Profile("car_no_tc").setVehicle("car").setWeighting("fastest"))
                ).
                setLMProfiles(Arrays.asList(
                        new LMProfile("car"),
                        new LMProfile("car_no_tc").setPreparationProfile("car")
                )).
                setCHProfiles(Collections.singletonList(
                        new CHProfile("car_no_tc")
                ));
        return config;
    }

    @ClassRule
    public static final DropwizardAppRule<MapMatchingServerConfiguration> app = new DropwizardAppRule<>(MapMatchingApplication.class, createConfig());

    @BeforeClass
    @AfterClass
    public static void cleanUp() {
        Helper.removeDir(new File(DIR));
    }

    @Test
    public void useDefault() {
        runTest("");
    }

    @Test
    public void useVehicle() {
        // see map-matching/#178
        runTest("vehicle=car");
    }

    @Test
    public void useProfile() {
        runTest("profile=car");
    }

    @Test
    public void useProfileNoTC() {
        runTest("profile=car_no_tc");
    }

    private void runTest(String urlParams) {
        final Response response = app.client().target("http://localhost:8080/match?" + urlParams)
                .request()
                .buildPost(Entity.xml(getClass().getResourceAsStream("tour2-with-loop.gpx")))
                .invoke();
        JsonNode json = response.readEntity(JsonNode.class);
        assertFalse(json.toString(), json.has("message"));
        assertEquals(200, response.getStatus());
        JsonNode path = json.get("paths").get(0);

        assertEquals(5, path.get("instructions").size());
        assertEquals(5, WebHelper.decodePolyline(path.get("points").asText(), 10, false).size());
        assertEquals(106.15, path.get("time").asLong() / 1000f, 0.1);
        assertEquals(106.15, json.get("map_matching").get("time").asLong() / 1000f, 0.1);
        assertEquals(811.56, path.get("distance").asDouble(), 1);
        assertEquals(811.56, json.get("map_matching").get("distance").asDouble(), 1);
    }

}
