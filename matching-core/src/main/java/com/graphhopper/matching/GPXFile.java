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

import com.graphhopper.routing.Path;
import com.graphhopper.util.Constants;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.Helper;
import com.graphhopper.util.Instruction;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PointList;
import com.graphhopper.util.Translation;
import java.io.*;
import java.text.SimpleDateFormat;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;
import java.util.TimeZone;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/**
 * A simple utility method to import from and export to GPX files.
 * <p>
 * @author Peter Karich
 */
public class GPXFile {

    static final String DATE_FORMAT = "yyyy-MM-dd'T'HH:mm:ssZ";
    static final String DATE_FORMAT_Z = "yyyy-MM-dd'T'HH:mm:ss'Z'";
    static final String DATE_FORMAT_Z_MS = "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'";
    private final List<GPXEntry> entries;
    private boolean includeElevation = false;
    private List<InstructionList> instructions;

    public GPXFile() {
        entries = new ArrayList<GPXEntry>();
    }

    public GPXFile(List<GPXEntry> entries) {
        this.entries = entries;
    }

    public GPXFile(MatchResult mr, List<InstructionList> il) {
        this.instructions = il;
        this.entries = new ArrayList<GPXEntry>(mr.getEdgeMatches().size());
        // TODO fetch time from GPX or from calculated route?
        long time = 0;
        for (int emIndex = 0; emIndex < mr.getEdgeMatches().size(); emIndex++) {
            MatchedEdge em = mr.getEdgeMatches().get(emIndex);
            PointList pl = em.edge.fetchWayGeometry(emIndex == 0 ? 3 : 2);
            if (pl.is3D()) {
                includeElevation = true;
            }
            for (int i = 0; i < pl.size(); i++) {
                if (pl.is3D()) {
                    entries.add(new GPXEntry(pl.getLatitude(i), pl.getLongitude(i), pl.getElevation(i), time));
                } else {
                    entries.add(new GPXEntry(pl.getLatitude(i), pl.getLongitude(i), time));
                }
            }
        }
    }

    public List<GPXEntry> getEntries() {
        return entries;
    }

    public GPXFile doImport(String fileStr) {
        try {
            return doImport(new FileInputStream(fileStr), 20);
        } catch (FileNotFoundException ex) {
            throw new RuntimeException(ex);
        }
    }

    /**
     * This method creates a GPXFile object filled with lat,lon values from the
     * xml inputstream is.
     *
     * @param defaultSpeed if no time element is found the time value will be
     *                     guessed from the distance and this provided default speed in kph.
     */
    public GPXFile doImport(InputStream is, double defaultSpeed) {
        SimpleDateFormat formatter = new SimpleDateFormat(DATE_FORMAT);
        SimpleDateFormat formatterZ = new SimpleDateFormat(DATE_FORMAT_Z);
        SimpleDateFormat formatterZMS = new SimpleDateFormat(DATE_FORMAT_Z_MS);
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        factory.setValidating(false);
        factory.setIgnoringElementContentWhitespace(true);
        DistanceCalc distCalc = Helper.DIST_PLANE;
        try {
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(is);
            NodeList nl = doc.getElementsByTagName("trkpt");
            double prevLat = 0, prevLon = 0;
            long prevMillis = 0;
            for (int index = 0; index < nl.getLength(); index++) {
                Node n = nl.item(index);
                if (!(n instanceof Element)) {
                    continue;
                }

                Element e = (Element) n;
                double lat = Double.parseDouble(e.getAttribute("lat"));
                double lon = Double.parseDouble(e.getAttribute("lon"));
                NodeList timeNodes = e.getElementsByTagName("time");
                long millis = prevMillis;
                if (timeNodes.getLength() == 0) {
                    if (index > 0) {
                        millis += Math.round(distCalc.calcDist(prevLat, prevLon, lat, lon) * 3600 / defaultSpeed);
                    }

                } else {
                    String text = timeNodes.item(0).getTextContent();
                    if (text.contains("Z")) {
                        try {
                            // Try whole second matching
                            millis = formatterZ.parse(text).getTime();
                        } catch (ParseException ex) {
                            // Error: try looking at milliseconds
                            millis = formatterZMS.parse(text).getTime();
                        }
                    } else {
                        millis = formatter.parse(revertTZHack(text)).getTime();
                    }
                }

                NodeList eleNodes = e.getElementsByTagName("ele");
                if (eleNodes.getLength() == 0) {
                    entries.add(new GPXEntry(lat, lon, millis));
                } else {
                    double ele = Double.parseDouble(eleNodes.item(0).getTextContent());
                    entries.add(new GPXEntry(lat, lon, ele, millis));
                }
                prevLat = lat;
                prevLon = lon;
                prevMillis = millis;
            }
            return this;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Hack to parse time in Java and convert +0100 into +01:00
     */
    private static String revertTZHack(String str) {
        return str.substring(0, str.length() - 3) + str.substring(str.length() - 2);
    }

    @Override
    public String toString() {
        return "entries " + entries.size() + ", " + entries;
    }

    // TODO DUPLICATE CODE from GraphHopper InstructionList!
    //
    public String createString() {
        long startTimeMillis = 0;
        SimpleDateFormat formatter = new SimpleDateFormat(DATE_FORMAT_Z);
        formatter.setTimeZone(TimeZone.getTimeZone("UTC"));

        String header = "<?xml version='1.0' encoding='UTF-8' standalone='no' ?>"
                + "<gpx xmlns='http://www.topografix.com/GPX/1/1' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'"
                + " creator='Graphhopper MapMatching " + Constants.VERSION + "' version='1.1'"
                // This xmlns:gh acts only as ID, no valid URL necessary.
                // Use a separate namespace for custom extensions to make basecamp happy.
                + " xmlns:gh='https://graphhopper.com/public/schema/gpx/1.1'>"
                + "\n<metadata>"
                + "<copyright author=\"OpenStreetMap contributors\"/>"
                + "<link href='http://graphhopper.com'>"
                + "<text>GraphHopper GPX</text>"
                + "</link>"
                + "<time>" + formatter.format(startTimeMillis) + "</time>"
                + "</metadata>";
        StringBuilder gpxOutput = new StringBuilder(header);
        gpxOutput.append("\n<trk><name>").append("GraphHopper MapMatching").append("</name>");

        // TODO: is this correct? how do we know there's a 'gap' in the instructions i.e. multiple
        // sequences? Do instructions only make sense for a single sequence?
        if (instructions != null && !instructions.isEmpty()) {
            gpxOutput.append("\n<rte>");
            for (InstructionList instr: instructions) {
                Instruction nextInstr = null;
                for (Instruction currInstr : instr) {
                    if (null != nextInstr) {
                        instr.createRteptBlock(gpxOutput, nextInstr, currInstr);
                    }
                    nextInstr = currInstr;
                }
                instr.createRteptBlock(gpxOutput, nextInstr, null);
            }
            gpxOutput.append("\n</rte>");
        }

        gpxOutput.append("<trkseg>");
        for (GPXEntry entry : entries) {
            gpxOutput.append("\n<trkpt lat='").append(Helper.round6(entry.getLat()));
            gpxOutput.append("' lon='").append(Helper.round6(entry.getLon())).append("'>");
            if (includeElevation) {
                gpxOutput.append("<ele>").append(Helper.round2(entry.getEle())).append("</ele>");
            }
            gpxOutput.append("<time>").append(formatter.format(startTimeMillis + entry.getTime())).append("</time>");
            gpxOutput.append("</trkpt>");
        }
        gpxOutput.append("</trkseg>");
        gpxOutput.append("</trk>");

        // we could now use 'wpt' for via points
        gpxOutput.append("</gpx>");
        return gpxOutput.toString().replaceAll("\\'", "\"");
    }

    public GPXFile doExport(String gpxFile) {
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter(gpxFile));
            writer.append(createString());
            return this;
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        } finally {
            Helper.close(writer);
        }
    }

    public static void write(Path path, String gpxFile, Translation translation) {
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter(gpxFile));
            writer.append(path.calcInstructions(translation).createGPX());
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        } finally {
            Helper.close(writer);
        }
    }
}
