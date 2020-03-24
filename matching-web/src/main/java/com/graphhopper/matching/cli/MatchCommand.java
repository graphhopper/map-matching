package com.graphhopper.matching.cli;

import com.fasterxml.jackson.dataformat.xml.XmlMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.PathWrapper;
import com.graphhopper.config.ProfileConfig;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.gpx.Gpx;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.*;
import com.graphhopper.util.gpx.GpxFromInstructions;
import io.dropwizard.cli.Command;
import io.dropwizard.setup.Bootstrap;
import net.sourceforge.argparse4j.inf.Namespace;
import net.sourceforge.argparse4j.inf.Subparser;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Collections;
import java.util.Date;
import java.util.List;

import static com.graphhopper.util.Parameters.Routing.MAX_VISITED_NODES;

public class MatchCommand extends Command {

    public MatchCommand() {
        super("match", "map-match one or more gpx files");
    }

    @Override
    public void configure(Subparser subparser) {
        subparser.addArgument("gpx")
                .type(File.class)
                .required(true)
                .nargs("+")
                .help("GPX file");
        subparser.addArgument("--instructions")
                .type(String.class)
                .required(false)
                .setDefault("")
                .help("Locale for instructions");
        subparser.addArgument("--max_visited_nodes")
                .type(Integer.class)
                .required(false)
                .setDefault(1000);
        subparser.addArgument("--gps_accuracy")
                .type(Integer.class)
                .required(false)
                .setDefault(40);
        subparser.addArgument("--transition_probability_beta")
                .type(Double.class)
                .required(false)
                .setDefault(2.0);
    }

    @Override
    public void run(Bootstrap bootstrap, Namespace args) {
        GraphHopperConfig graphHopperConfiguration = new GraphHopperConfig();
        graphHopperConfiguration.putObject("graph.location", "graph-cache");
        GraphHopper hopper = new GraphHopperOSM().init(graphHopperConfiguration);
        System.out.println("loading graph from cache");
        // since we are loading GraphHopper from disk we more or less have to guess the configuration
        String vehicle = args.getString("vehicle");
        if (Helper.isEmpty(vehicle))
            vehicle = hopper.getEncodingManager().fetchEdgeEncoders().get(0).toString();

        // Penalizing inner-link U-turns only works with fastest weighting, since
        // shortest weighting does not apply penalties to unfavored virtual edges.
        String weightingStr = "fastest";
        hopper.setProfiles(new ProfileConfig("profile").setVehicle(vehicle).setWeighting(weightingStr).setTurnCosts(false));
        hopper.load(graphHopperConfiguration.getString("graph.location", "graph-cache"));

        PMap hints = new PMap().putObject(MAX_VISITED_NODES, args.get("max_visited_nodes"));
        MapMatching mapMatching = new MapMatching(hopper, hints);
        mapMatching.setTransitionProbabilityBeta(args.getDouble("transition_probability_beta"));
        mapMatching.setMeasurementErrorSigma(args.getInt("gps_accuracy"));

        StopWatch importSW = new StopWatch();
        StopWatch matchSW = new StopWatch();

        Translation tr = new TranslationMap().doImport().getWithFallBack(Helper.getLocale(args.getString("instructions")));
        final boolean withRoute = !args.getString("instructions").isEmpty();
        XmlMapper xmlMapper = new XmlMapper();

        Weighting weighting = hopper.createWeighting(hopper.getProfiles().get(0), hints);

        for (File gpxFile : args.<File>getList("gpx")) {
            try {
                importSW.start();
                Gpx gpx = xmlMapper.readValue(gpxFile, Gpx.class);
                if (gpx.trk == null) {
                    throw new IllegalArgumentException("No tracks found in GPX document. Are you using waypoints or routes instead?");
                }
                if (gpx.trk.size() > 1) {
                    throw new IllegalArgumentException("GPX documents with multiple tracks not supported yet.");
                }
                List<Observation> measurements = gpx.trk.get(0).getEntries();
                importSW.stop();
                matchSW.start();
                MatchResult mr = mapMatching.doWork(measurements);
                matchSW.stop();
                System.out.println(gpxFile);
                System.out.println("\tmatches:\t" + mr.getEdgeMatches().size() + ", gps entries:" + measurements.size());
                System.out.println("\tgpx length:\t" + (float) mr.getGpxEntriesLength() + " vs " + (float) mr.getMatchLength());

                String outFile = gpxFile.getAbsolutePath() + ".res.gpx";
                System.out.println("\texport results to:" + outFile);

                PathWrapper pathWrapper = new PathWrapper();
                new PathMerger(mr.getGraph(), weighting).
                        doWork(pathWrapper, Collections.singletonList(mr.getMergedPath()), hopper.getEncodingManager(), tr);
                if (pathWrapper.hasErrors()) {
                    System.err.println("Problem with file " + gpxFile + ", " + pathWrapper.getErrors());
                    continue;
                }

                try (BufferedWriter writer = new BufferedWriter(new FileWriter(outFile))) {
                    long time = gpx.trk.get(0).getStartTime()
                            .map(Date::getTime)
                            .orElse(System.currentTimeMillis());
                    writer.append(GpxFromInstructions.createGPX(pathWrapper.getInstructions(), gpx.trk.get(0).name != null ? gpx.trk.get(0).name : "", time, hopper.hasElevation(), withRoute, true, false, Constants.VERSION, tr));
                }
            } catch (Exception ex) {
                importSW.stop();
                matchSW.stop();
                System.err.println("Problem with file " + gpxFile);
                ex.printStackTrace(System.err);
            }
        }
        System.out.println("gps import took:" + importSW.getSeconds() + "s, match took: " + matchSW.getSeconds());
    }

}
