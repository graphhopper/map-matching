package vaadinexample;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.*;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint3D;
import com.vaadin.annotations.Theme;
import com.vaadin.annotations.VaadinServletConfiguration;
import com.vaadin.data.Item;
import com.vaadin.data.util.HierarchicalContainer;
import com.vaadin.server.VaadinRequest;
import com.vaadin.server.VaadinServlet;
import com.vaadin.ui.HorizontalLayout;
import com.vaadin.ui.TreeTable;
import com.vaadin.ui.UI;
import com.vaadin.ui.VerticalLayout;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.vaadin.addon.leaflet.*;
import org.vaadin.addon.leaflet.shared.Point;

import javax.servlet.annotation.WebServlet;
import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.List;

@Theme("mytheme")
@SuppressWarnings("serial")
public class IsochronesUI extends UI {

    enum ContainerProperties {
        FROM, TO, N_GEOMETRIES;
    }

    enum CyclingColors {
        red, black, green, orange;
    }

    private LMap map;
    private HierarchicalContainer container;


    private final Logger logger = LoggerFactory.getLogger(getClass());


    @WebServlet(value = "/*", asyncSupported = true)
    @VaadinServletConfiguration(productionMode = false, ui = IsochronesUI.class, widgetset = "playground.vaadinexample.AppWidgetSet")
    public static class Servlet extends VaadinServlet {
    }


    File[] getFiles(String gpxLocation) {
        if (gpxLocation.contains("*")) {
            int lastIndex = gpxLocation.lastIndexOf(File.separator);
            final String pattern;
            File dir = new File(".");
            if (lastIndex >= 0) {
                dir = new File(gpxLocation.substring(0, lastIndex));
                pattern = gpxLocation.substring(lastIndex + 1);
            } else {
                pattern = gpxLocation;
            }

            return dir.listFiles(new FilenameFilter() {
                @Override
                public boolean accept(File dir, String name) {
                    return name.matches(pattern);
                }
            });
        } else {
            return new File[]{
                    new File(gpxLocation)
            };
        }
    }


    @Override
    protected void init(VaadinRequest request) {
        String[] s = new String[]{"action=match", "gpx=/Users/michaelzilske/wurst/peter/tmp/1001.gpx"};
        CmdArgs args = CmdArgs.read(s);
        args.put("graph.location", "../graph-cache");

        GraphHopper hopper = new GraphHopper().init(args);
        hopper.setCHEnable(false);
        logger.info("loading graph from cache");
        hopper.load("../graph-cache");
        FlagEncoder firstEncoder = hopper.getEncodingManager().fetchEdgeEncoders().get(0);
        GraphHopperStorage graph = hopper.getGraphHopperStorage();

        int gpxAccuracy = args.getInt("gpxAccuracy", 15);
        String instructions = args.get("instructions", "");
        logger.info("Setup lookup index. Accuracy filter is at " + gpxAccuracy + "m");
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex(), gpxAccuracy);
        MapMatching mapMatching = new MapMatching(graph, locationIndex, firstEncoder);
        mapMatching.setSeparatedSearchDistance(args.getInt("separatedSearchDistance", 500));
        mapMatching.setMaxNodesToVisit(args.getInt("maxNodesToVisit", 1000));
        mapMatching.setForceRepair(args.getBool("forceRepair", false));

        // do the actual matching, get the GPX entries from a file or via stream
        String gpxLocation = args.get("gpx", "");
        File[] files = getFiles(gpxLocation);

        logger.info("Now processing " + files.length + " files");
        StopWatch importSW = new StopWatch();
        StopWatch matchSW = new StopWatch();

        Translation tr = new TranslationMap().doImport().get(instructions);

        container = new HierarchicalContainer();
        container.addContainerProperty(ContainerProperties.FROM, Integer.class, 0);
        container.addContainerProperty(ContainerProperties.TO, Integer.class, 0);
        container.addContainerProperty(ContainerProperties.N_GEOMETRIES, Integer.class, 0);

        TreeTable table = new TreeTable("Isochrones", container);
        table.setColumnHeader(ContainerProperties.FROM, "From");
        table.setColumnHeader(ContainerProperties.TO, "To");
        table.setColumnHeader(ContainerProperties.N_GEOMETRIES, "Number of sub-geometries");

        VerticalLayout layout = new VerticalLayout();
        layout.setMargin(true);

        map = new LMap();

        for (File gpxFile : files) {
            importSW.start();
            List<GPXEntry> inputGPXEntries = new GPXFile().doImport(gpxFile.getAbsolutePath()).getEntries();
            importSW.stop();
            LLayerGroup input = new LLayerGroup();
            for (GPXEntry inputGPXEntry : inputGPXEntries) {
                input.addComponent(new LCircleMarker(inputGPXEntry.getLat(), inputGPXEntry.getLon(), 3.0));
            }
            map.addOverlay(input, "Input");
            matchSW.start();
            MatchResult mr = mapMatching.doWork(inputGPXEntries);
            matchSW.stop();
            System.out.println(gpxFile);
            System.out.println("\tmatches:\t" + mr.getEdgeMatches().size() + ", gps entries:" + inputGPXEntries.size());
            System.out.println("\tgpx length:\t" + (float) mr.getGpxEntriesLength() + " vs " + (float) mr.getMatchLength());
            System.out.println("\tgpx time:\t" + mr.getGpxEntriesMillis() / 1000f + " vs " + mr.getMatchMillis() / 1000f);
            LLayerGroup originLayer = new LLayerGroup();
            List<EdgeMatch> edgeMatches = mr.getEdgeMatches();
            for (int i = 0; i < edgeMatches.size(); i++) {
                String color = CyclingColors.values()[i % CyclingColors.values().length].toString();
                EdgeMatch edgeMatch = mr.getEdgeMatches().get(i);
                PointList points = edgeMatch.getEdgeState().fetchWayGeometry(i == 0 ? 3 : 3);
                ArrayList<Point> points1 = new ArrayList<>();
                for (GHPoint3D point : points) {
                    Point point1 = new Point(point.getLat(), point.getLon());
                    points1.add(point1);
                }
                LPolyline line = new LPolyline(points1.toArray(new Point[]{}));
                line.setColor(color);
                originLayer.addComponent(line);
                Object linkId = container.addItem();
                Item link = container.getItem(linkId);
                link.getItemProperty(ContainerProperties.FROM).setValue(edgeMatch.getEdgeState().getBaseNode());
                link.getItemProperty(ContainerProperties.TO).setValue(edgeMatch.getEdgeState().getAdjNode());
                link.getItemProperty(ContainerProperties.N_GEOMETRIES).setValue(points.size());
                for (GPXExtension gpxExtension : edgeMatch.getGpxExtensions()) {
                    LCircleMarker c = new LCircleMarker(gpxExtension.getEntry().getLat(), gpxExtension.getEntry().getLon(), 3.0);
                    c.setColor(color);
                    originLayer.addComponent(c);
                }
            }
            map.addOverlay(originLayer, "Match");
            GPXFile gpxFile1 = new GPXFile(mr, new InstructionList(null));
            for (GPXEntry gpxEntry : gpxFile1.getEntries()) {
//                map.addLayer(new LMarker(gpxEntry.getLat(), gpxEntry.getLon()));
            }

        }
        System.out.println("gps import took:" + importSW.getSeconds() + "s, match took: " + matchSW.getSeconds());
        LTileLayer osmTiles = new LTileLayer("http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png");
        osmTiles.setAttributionString("© OpenStreetMap Contributors");

        map.addBaseLayer(osmTiles, "OSM");

        HorizontalLayout options = new HorizontalLayout();

        layout.addComponents(options, map, table);
        layout.setExpandRatio(map, 0.8f);
        layout.setSizeFull();
        setContent(layout);

        map.zoomToContent();
    }

}
