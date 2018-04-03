package org.jlab.service.dc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.detector.base.DetectorType;
import org.jlab.detector.base.GeometryFactory;
import org.jlab.detector.calib.utils.DatabaseConstantProvider;
import org.jlab.detector.geant4.v2.DCGeant4Factory;
import org.jlab.geom.base.ConstantProvider;
import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.dc.banks.HitReader;
import org.jlab.rec.dc.banks.RecoBankWriter;
import org.jlab.rec.dc.cluster.ClusterCleanerUtilities;
import org.jlab.rec.dc.cluster.ClusterFinder;
import org.jlab.rec.dc.cluster.ClusterFitter;
import org.jlab.rec.dc.cluster.FittedCluster;
import org.jlab.rec.dc.cross.Cross;
import org.jlab.rec.dc.cross.CrossList;
import org.jlab.rec.dc.cross.CrossListFinder;
import org.jlab.rec.dc.cross.CrossMaker;
import org.jlab.rec.dc.hit.FittedHit;
import org.jlab.rec.dc.segment.Segment;
import org.jlab.rec.dc.segment.SegmentFinder;
import org.jlab.rec.dc.timetodistance.TableLoader;
import org.jlab.rec.dc.timetodistance.TimeToDistanceEstimator;
import org.jlab.rec.dc.track.Track;
import org.jlab.rec.dc.track.TrackCandListFinder;
import org.jlab.rec.dc.trajectory.RoadFinder;

public class DCTBEngine extends ReconstructionEngine {

    int Run = 0;

    double[][][][] T0 ;
    double[][][][] T0ERR ;
    DCGeant4Factory dcDetector;

    double TORSCALE;
    double SOLSCALE;

    private TimeToDistanceEstimator tde;
    public DCTBEngine() {
        super("DCTB","ziegler","4.0");
    }
    @Override
    public boolean init() {
        String[]  dcTables = new String[]{
            "/calibration/dc/signal_generation/doca_resolution",
            // "/calibration/dc/time_to_distance/t2d",
            "/calibration/dc/time_to_distance/time2dist",
            //    "/calibration/dc/time_corrections/T0_correction",
        };
        requireConstants(Arrays.asList(dcTables));
        // Get the constants for the correct variation
        this.getConstantsManager().setVariation("default");

        // Load the geometry
        ConstantProvider provider = GeometryFactory.getConstants(DetectorType.DC, 11, "default");
        dcDetector = new DCGeant4Factory(provider, DCGeant4Factory.MINISTAGGERON);

        //T0s
        T0 = new double[6][6][7][6]; //nSec*nSL*nSlots*nCables
        T0ERR = new double[6][6][7][6]; //nSec*nSL*nSlots*nCables
        //DatabaseConstantProvider dbprovider = new DatabaseConstantProvider(800, "default");
        //dbprovider.loadTable("/calibration/dc/time_corrections/T0Corrections");
        //disconnect from database. Important to do this after loading tables.
        //dbprovider.disconnect();
        // T0-subtraction

        //for (int i = 0; i < dbprovider.length("/calibration/dc/time_corrections/T0Corrections/Sector"); i++) {
        //    int iSec = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Sector", i);
        //    int iSly = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Superlayer", i);
        //    int iSlot = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Slot", i);
        //    int iCab = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Cable", i);
        //    double t0 = dbprovider.getDouble("/calibration/dc/time_corrections/T0Corrections/T0Correction", i);
        //    double t0Error = dbprovider.getDouble("/calibration/dc/time_corrections/T0Corrections/T0Error", i);

        //    T0[iSec - 1][iSly - 1][iSlot - 1][iCab - 1] = t0;
        //    T0ERR[iSec - 1][iSly - 1][iSlot - 1][iCab - 1] = t0Error;
        //}
        tde = new TimeToDistanceEstimator();
        return true;
    }
    @Override
    public boolean processDataEvent(DataEvent event) {
        //setRunConditionsParameters( event) ;
        if(event.hasBank("RUN::config")==false) {
            System.err.println("RUN CONDITIONS NOT READ AT TIMEBASED LEVEL!");
            return true;
        }
        //if(event.getBank("RECHB::Event").getFloat("STTime", 0)<0)
        //    return true; // require the start time to reconstruct the tracks in the event

        DataBank bank = event.getBank("RUN::config");
        // Load the constants
        //-------------------
        int newRun = bank.getInt("run", 0);

        if(Run!=newRun) {
            DatabaseConstantProvider dbprovider = new DatabaseConstantProvider(newRun, "default");
            dbprovider.loadTable("/calibration/dc/time_corrections/T0Corrections");
            //disconnect from database. Important to do this after loading tables.
            dbprovider.disconnect();
            // T0-subtraction

            for (int i = 0; i < dbprovider.length("/calibration/dc/time_corrections/T0Corrections/Sector"); i++) {
                int iSec = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Sector", i);
                int iSly = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Superlayer", i);
                int iSlot = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Slot", i);
                int iCab = dbprovider.getInteger("/calibration/dc/time_corrections/T0Corrections/Cable", i);
                double t0 = dbprovider.getDouble("/calibration/dc/time_corrections/T0Corrections/T0Correction", i);
                double t0Error = dbprovider.getDouble("/calibration/dc/time_corrections/T0Corrections/T0Error", i);

                T0[iSec - 1][iSly - 1][iSlot - 1][iCab - 1] = t0; 
                T0ERR[iSec - 1][iSly - 1][iSlot - 1][iCab - 1] = t0Error;
            }
                //CCDBTables.add(this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_corrections/T0_correction"));
                TORSCALE = (double)bank.getFloat("torus", 0);
                SOLSCALE = (double)bank.getFloat("solenoid", 0);
                // TableLoader.Fill(this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/t2d"));
                TableLoader.Fill(this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist")); 

                Run = newRun;
        }

        System.out.println(" RUNNING TIME BASED....................................");
        ClusterFitter cf = new ClusterFitter();
        ClusterCleanerUtilities ct = new ClusterCleanerUtilities();

        List<FittedHit> fhits = new ArrayList<FittedHit>();	
        List<FittedCluster> clusters = new ArrayList<FittedCluster>();
        List<Segment> segments = new ArrayList<Segment>();
        List<Cross> crosses = new ArrayList<Cross>();
        List<Track> trkcands = new ArrayList<Track>();

        //instantiate bank writer
        RecoBankWriter rbc = new RecoBankWriter();

        HitReader hitRead = new HitReader();
        hitRead.read_HBHits(event, 
            this.getConstantsManager().getConstants(newRun, "/calibration/dc/signal_generation/doca_resolution"),
            this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist"),
            T0, T0ERR, dcDetector, tde);
        hitRead.read_TBHits(event, 
            this.getConstantsManager().getConstants(newRun, "/calibration/dc/signal_generation/doca_resolution"),
            this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist"), tde, T0, T0ERR);
        List<FittedHit> hits = new ArrayList<FittedHit>();
        //I) get the hits
        if(hitRead.get_TBHits().isEmpty()) {
            hits = hitRead.get_HBHits();

        } else {
            hits = hitRead.get_TBHits();
        }

        //II) process the hits
        //1) exit if hit list is empty
        if(hits.isEmpty() ) {
                return true;
        }

        //2) find the clusters from these hits
        ClusterFinder clusFinder = new ClusterFinder();

        clusters = clusFinder.FindTimeBasedClusters(hits, cf, ct, this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist"), dcDetector, tde);

        if(clusters.isEmpty()) {
            rbc.fillAllTBBanks(event, rbc, hits, null, null, null, null);
            return true;
        }

        //3) find the segments from the fitted clusters
        SegmentFinder segFinder = new SegmentFinder();

        List<FittedCluster> pclusters = segFinder.selectTimeBasedSegments(clusters);

        segments =  segFinder.get_Segments(pclusters, event, dcDetector);

        if(segments.isEmpty()) { // need 6 segments to make a trajectory
            for(FittedCluster c : clusters) {					
                for(FittedHit hit : c) {		
                    hit.set_AssociatedClusterID(c.get_Id());
                    hit.set_AssociatedHBTrackID(c.get(0).get_AssociatedHBTrackID());
                    fhits.add(hit);						
                }
            }
            rbc.fillAllTBBanks( event, rbc, fhits, clusters, null, null, null);
            return true;
        }

        for(Segment seg : segments) {					
            for(FittedHit hit : seg.get_fittedCluster()) {	
                fhits.add(hit);						
            }
        }

        CrossMaker crossMake = new CrossMaker();
        crosses = crossMake.find_Crosses(segments, dcDetector);

        if(crosses.isEmpty() ) {			
            rbc.fillAllTBBanks(event, rbc, fhits, clusters, segments, null, null);
            return true;
        }
        /*
        //
        // also need Track bank
        if (event.hasBank("HitBasedTrkg::HBTracks") == false) {
            return true;
        }
        
        DataBank trkbank = event.getBank("HitBasedTrkg::HBTracks");
        int trkrows = trkbank.rows();
        Track[] TrackArray = new Track[trkrows];
        for (int i = 0; i < trkrows; i++) {
            Track HBtrk = new Track();
            HBtrk.set_Id(trkbank.getShort("id", i));
            HBtrk.set_Sector(trkbank.getByte("sector", i));
            HBtrk.set_Q(trkbank.getByte("q", i));
            HBtrk.set_pAtOrig(new Vector3D(trkbank.getFloat("p0_x", i), trkbank.getFloat("p0_y", i), trkbank.getFloat("p0_z", i)));
            HBtrk.set_Vtx0(new Point3D(trkbank.getFloat("Vtx0_x", i), trkbank.getFloat("Vtx0_y", i), trkbank.getFloat("Vtx0_z", i)));
            TrackArray[HBtrk.get_Id()-1] = HBtrk; 
        }
        
        for(Segment seg : segments) {
            TrackArray[seg.get(0).get_AssociatedHBTrackID()-1].get_ListOfHBSegments().add(seg);
        }
        
        */
        //
        CrossListFinder crossLister = new CrossListFinder();
        CrossList crosslist = crossLister.candCrossLists(crosses, false, this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist"), dcDetector, null);

        //6) find the list of  track candidates
        TrackCandListFinder trkcandFinder = new TrackCandListFinder("TimeBased");
        trkcands = trkcandFinder.getTrackCands(crosslist, dcDetector, TORSCALE) ;


        // track found	
        int trkId = 1;

        if(trkcands.size()>0) {
            trkcandFinder.removeOverlappingTracks(trkcands);		// remove overlaps

            for(Track trk: trkcands) {
                // reset the id
                trk.set_Id(trkId);
                trkcandFinder.matchHits(trk.get_Trajectory(), trk, dcDetector);
                for(Cross c : trk) { 
                    c.get_Segment1().isOnTrack=true;
                    c.get_Segment2().isOnTrack=true;

                    for(FittedHit h1 : c.get_Segment1()) { 
                        h1.set_AssociatedTBTrackID(trk.get_Id());

                    }
                    for(FittedHit h2 : c.get_Segment2()) {
                        h2.set_AssociatedTBTrackID(trk.get_Id());                              
                    }
                }
                trkId++;
            }
        }    
        RoadFinder pcrossLister = new RoadFinder();
        List<Segment> pSegments =pcrossLister.findPseudoSegList(segments, dcDetector);

        segments.addAll(pSegments);

        List<Cross> pcrosses = crossMake.find_Crosses(segments, dcDetector);

        //
        CrossList pcrosslist = crossLister.candCrossLists(pcrosses, false, this.getConstantsManager().getConstants(newRun, "/calibration/dc/time_to_distance/time2dist"), dcDetector, null);

        List<Track> mistrkcands =trkcandFinder.getTrackCands(pcrosslist, dcDetector, TORSCALE);
        
        if(mistrkcands.size()>0) {    
            trkcandFinder.removeOverlappingTracks(mistrkcands);		// remove overlaps

            for(Track trk: mistrkcands) {
                // reset the id
                trk.set_Id(trkId);
                trkcandFinder.matchHits(trk.get_Trajectory(), trk, dcDetector);
                for(Cross c : trk) { 
                    for(FittedHit h1 : c.get_Segment1()) { 
                        h1.set_AssociatedTBTrackID(trk.get_Id());

                    }
                    for(FittedHit h2 : c.get_Segment2()) {
                        h2.set_AssociatedTBTrackID(trk.get_Id());                              
                    }
                }
                trkId++;
            }
        }
        trkcands.addAll(mistrkcands) ;

        if(trkcands.isEmpty()) {

            rbc.fillAllTBBanks(event, rbc, fhits, clusters, segments, crosses, null); // no cand found, stop here and save the hits, the clusters, the segments, the crosses
            return true;
        }
        rbc.fillAllTBBanks(event, rbc, fhits, clusters, segments, crosses, trkcands);

        return true;
    }

}
