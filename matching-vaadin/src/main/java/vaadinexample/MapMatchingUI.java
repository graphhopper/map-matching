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
import com.vaadin.event.ItemClickEvent;
import com.vaadin.server.VaadinRequest;
import com.vaadin.server.VaadinServlet;
import com.vaadin.ui.HorizontalLayout;
import com.vaadin.ui.Table;
import com.vaadin.ui.UI;
import com.vaadin.ui.VerticalLayout;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.vaadin.addon.leaflet.*;
import org.vaadin.addon.leaflet.shared.Point;

import javax.servlet.annotation.WebServlet;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Theme("mytheme")
@SuppressWarnings("serial")
public class MapMatchingUI extends UI {

	private GraphHopperStorage graph;

	enum ContainerProperties {
        IDX, FROM, TO;
    }

    enum CyclingColors {
        red, black, green, orange;
    }

	private final Logger logger = LoggerFactory.getLogger(getClass());

	private LMap map;
    private HierarchicalContainer container;

    @WebServlet(value = "/*", asyncSupported = true)
    @VaadinServletConfiguration(productionMode = false, ui = MapMatchingUI.class, widgetset = "playground.vaadinexample.AppWidgetSet")
    public static class Servlet extends VaadinServlet {
    }

    @Override
    protected void init(VaadinRequest request) {
        String[] s = new String[]{"action=match", "gpx=/home/peterk/Downloads/misc/tmp/1001.gpx"};
        CmdArgs args = CmdArgs.read(s);
        args.put("graph.location", "../graph-cache");

        GraphHopper hopper = new GraphHopper().init(args);
        hopper.setCHEnable(false);
        logger.info("loading graph from cache");
        hopper.load("../graph-cache");
        FlagEncoder firstEncoder = hopper.getEncodingManager().fetchEdgeEncoders().get(0);
		graph = hopper.getGraphHopperStorage();

        int gpxAccuracy = args.getInt("gpxAccuracy", 15);
        String instructions = args.get("instructions", "");
        logger.info("Setup lookup index. Accuracy filter is at " + gpxAccuracy + "m");
        LocationIndexMatch locationIndex = new LocationIndexMatch(graph,
                (LocationIndexTree) hopper.getLocationIndex(), gpxAccuracy);
        MapMatching mapMatching = new MapMatching(graph, locationIndex, firstEncoder);

        // do the actual matching, get the GPX entries from a file or via stream
        String gpxLocation = args.get("gpx", "");

        StopWatch importSW = new StopWatch();
        StopWatch matchSW = new StopWatch();

        container = new HierarchicalContainer();
		container.addContainerProperty(ContainerProperties.IDX, Integer.class, 0);
		container.addContainerProperty(ContainerProperties.FROM, Integer.class, 0);
        container.addContainerProperty(ContainerProperties.TO, Integer.class, 0);

        Table table = new Table("Links", container);
        table.setColumnHeader(ContainerProperties.IDX, "Index");
        table.setColumnHeader(ContainerProperties.FROM, "From");
        table.setColumnHeader(ContainerProperties.TO, "To");

        VerticalLayout layout = new VerticalLayout();
        layout.setMargin(true);

        map = new LMap();
        File gpxFile = new File(gpxLocation);

        importSW.start();
        List<GPXEntry> inputGPXEntries = new GPXFile().doImport(gpxFile.getAbsolutePath()).getEntries();
        importSW.stop();
        LLayerGroup input = new LLayerGroup();
        for (GPXEntry inputGPXEntry : inputGPXEntries) {
            input.addComponent(new LCircleMarker(inputGPXEntry.getLat(), inputGPXEntry.getLon(), 3.0));
        }
        map.addOverlay(input, "Input");
        matchSW.start();
        final MatchResult mr = mapMatching.doWork(inputGPXEntries);
        matchSW.stop();
        System.out.println(gpxFile);
        System.out.println("\tmatches:\t" + mr.getEdgeMatches().size() + ", gps entries:" + inputGPXEntries.size());
        System.out.println("\tgpx length:\t" + (float) mr.getGpxEntriesLength() + " vs " + (float) mr.getMatchLength());
        System.out.println("\tgpx time:\t" + mr.getGpxEntriesMillis() / 1000f + " vs " + mr.getMatchMillis() / 1000f);
        final LLayerGroup originLayer = new LLayerGroup();
        List<EdgeMatch> edgeMatches = mr.getEdgeMatches();
        int ii = edgeMatches.size();
        fillUntil(mr, originLayer, ii);
        table.addItemClickListener(new ItemClickEvent.ItemClickListener() {
            @Override
            public void itemClick(ItemClickEvent itemClickEvent) {
                int ii = (Integer) itemClickEvent.getItem().getItemProperty(ContainerProperties.IDX).getValue();
                fillUntil(mr, originLayer, ii);
            }
        });
        map.addOverlay(originLayer, "Match");
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

    private void fillUntil(MatchResult mr, LLayerGroup originLayer, int ii) {
        container.removeAllItems();
        originLayer.removeAllComponents();
        for (int i = 0; i < mr.getEdgeMatches().size(); i++) {
			String color = CyclingColors.values()[i % CyclingColors.values().length].toString();
			EdgeMatch edgeMatch = mr.getEdgeMatches().get(i);
			PointList points = edgeMatch.getEdgeState().fetchWayGeometry(i == 0 ? 3 : 3);
			ArrayList<Point> points1 = new ArrayList<>();
			for (GHPoint3D point : points) {
				Point point1 = new Point(point.getLat(), point.getLon());
				points1.add(point1);
			}
            if (i<=ii) {
                LPolyline line = new LPolyline(points1.toArray(new Point[]{}));
                line.setColor(color);
                originLayer.addComponent(line);
            }
			Object linkId = container.addItem();
			Item link = container.getItem(linkId);
			link.getItemProperty(ContainerProperties.FROM).setValue(edgeMatch.getEdgeState().getBaseNode());
			link.getItemProperty(ContainerProperties.TO).setValue(edgeMatch.getEdgeState().getAdjNode());
			link.getItemProperty(ContainerProperties.IDX).setValue(i);
			for (final GPXExtension gpxExtension : edgeMatch.getGpxExtensions()) {
				LCircleMarker c = new LCircleMarker(gpxExtension.getEntry().getLat(), gpxExtension.getEntry().getLon(), 3.0);
				c.setColor(color);
				final int nodeIdx = i;
				c.addClickListener(new LeafletClickListener() {
					@Override
					public void onClick(LeafletClickEvent leafletClickEvent) {
						System.out.println(nodeIdx+ " " + gpxExtension.getQueryResult().toString() + " " + gpxExtension);
						EdgeIterator iter = graph.createEdgeExplorer().setBaseNode(gpxExtension.getQueryResult().getClosestEdge().getBaseNode());
						while(iter.next()) {
							System.out.println(iter.getAdjNode());
						}
					}
				});
				originLayer.addComponent(c);
			}
		}
    }

}
