package org.jlab.rec.rich;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.io.base.DataEvent;
import org.jlab.io.base.DataBank;

import org.jlab.utils.groups.IndexedTable;
import java.util.Arrays;
import java.util.Optional;

import org.jlab.geom.prim.Plane3D;
import org.jlab.geom.prim.Line3D;

import org.jlab.detector.geom.RICH.RICHGeoFactory;

public class RICHEBEngine extends ReconstructionEngine {

    private int Run = -1;
    private int Ncalls = 0;

    private long EBRICH_start_time;
    
    private RICHGeoFactory       geo;

    //// prova prova

    // ----------------
    public RICHEBEngine() {
    // ----------------
        super("RICHEB", "mcontalb", "3.0");

    }


    @Override
    // ----------------
    public boolean init() {
    // ----------------

        int debugMode = 0;
        if(debugMode>=1)System.out.format("I am in RICHEBEngine \n");


        String[] richTables = new String[]{
                    "/calibration/rich/parameterss",
                    "/calibration/rich/aerogel",
                    "/calibration/rich/misalignments",
                    "/calibration/rich/electro",
                    "/calibration/rich/time_walk",
                    "/calibration/rich/time_offset",
                    "/calibration/rich/pixels",
                 };

        requireConstants(Arrays.asList(richTables));

        // initialize constants manager default variation, will be then modified based on yaml settings
        // Get the constants for the correct variation
        String engineVariation = Optional.ofNullable(this.getEngineConfigString("variation")).orElse("default");         
        this.getConstantsManager().setVariation(engineVariation);

        // Get the constant tables for reconstruction parameters, geometry and optical characterization
        int run = 11;

        geo  = new RICHGeoFactory(1, this.getConstantsManager(), 11);

        return true;

    }


    @Override
    // ----------------
    public boolean processDataEvent(DataEvent event) {
    // ----------------

        int debugMode = 0;

        // create instances of all event-dependent classes in processDataEvent to avoid interferences between different threads when running in clara
        RICHEvent              richevent = new RICHEvent();
        RICHio                 richio    = new RICHio();
        RICHTool               tool      = new RICHTool(geo, Ncalls);

        RICHPMTReconstruction  rpmt      = new RICHPMTReconstruction(richevent, tool, richio);
        RICHEventBuilder       reb       = new RICHEventBuilder(event, richevent, tool, richio);
        
	//  Initialize the RICH event
        int run = richevent.get_RunID(); 
        if(run>0){
            tool.load_CCDB(this.getConstantsManager(), run);
        }else{
            tool.load_CCDB(this.getConstantsManager(), 11);
        }

        if(debugMode>=1){
            System.out.println("---------------------------------");
            System.out.println("RICH Engine call: "+Ncalls+" New Event Process "+reb.getEventID()+"\n");
            System.out.println("---------------------------------");
        }

        // clear RICH output banks
        if(tool.recpar.REDO_RICH_RECO==1)richio.clear_Banks(event); // would be better to move all io operation here rather than passing richio around

	/*
	Process RICH signals to get hits and clusters
	*/
        rpmt.processRawData(event);

        tool.save_ProcessTime(0);

	/*
	Process RICH-DC event reconstruction
	*/
        if( !reb.process_Data(event)) return false;

        tool.save_ProcessTime(6);
        //tool.dump_ProcessTime();

        Ncalls++;

        return true;

    }

}
