package org.jlab.rec.rich;


import org.jlab.detector.calib.utils.ConstantsManager;
import org.jlab.utils.groups.IndexedTable;

import java.io.FileReader;
import java.io.BufferedReader;

/**
 *
 * @author mcontalb
 */
public class RICHParameters{

    // Default values of RICH reconstruction parameters to be re-loaded from CCDB or TxT

    public int     DO_ALIGNMENT                           =   1;        // if 1 apply alignment
    public int     FORCE_DC_MATCH                         =   0;        // if 1 force the hadron track to hit the cluster
    public int     ALIGN_RICH_REF                         =   1;        // if 1 use local RICH frame (instead of Lab frame)
    public int     ALIGN_PMT_PIVOT                        =   1;        // if 1 use MAPMT barycenter for rotations
    public int     APPLY_SURVEY                           =   0;        // if 1 apply the survey data for misalignment

    public int     DO_ANALYTIC                            =   1;        // if 1 calculate analytic solution
    public int     THROW_ELECTRONS                        =   1;        // if 1 throw photons for electron hypothesis
    public int     THROW_PIONS                            =   0;        // if 1 throw photons for pion hypothesis
    public int     THROW_KAONS                            =   0;        // if 1 throw photons for kaon hypothesis
    public int     THROW_PROTONS                          =   0;        // if 1 throw photons for proton hypothesis
    public int     THROW_PHOTON_NUMBER                    =   50;       // number of photon trials for every hypothesis
    public int     TRACE_PHOTONS                          =   1;        // if 1 ray-trace phtoons

    public int     REDO_RICH_RECO                         =   1;        // if 1 rewrite the RICH banks
    public int     DO_MIRROR_HADS                         =   1;        // if 1 reconstruct hadrons pointing to mirror
    public int     DO_CURVED_AERO                         =   1;        // if 1 use spherical surface of aerogel

    public int     USE_ELECTRON_ANGLES                    =   0;        // Get Cherenkov angle and rms from electrons control sample
    public int     USE_PIXEL_PROPERTIES                   =   0;        // Use pixel status and efficiency in the likelihood
    public int     SAVE_THROWS                            =   0;        // Store throwed photons in the photons bank
    public int     QUADRANT_NUMBER                        =   15;       // Number of quadrants (square root of)

    public double  GOODHIT_FRAC                           =   80.;      // Maximum duration (in % of local max) to flag xtalk  
    public double  RICH_DCMATCH_CUT                       =   15.;      // RICH cluster matching cut with tracks 
    public double  RICH_HITMATCH_RMS                      =   0.6;      // RICH - particle matching chi2 reference (cm)
    public double  RICH_DIRECT_RMS                        =   6.0e-3;   // Expected single photon angular resolution (rad)

    public double  SHOW_PROGRESS_INTERVAL                 =   10.;      // Time interval between progress dumping (sec)
    public double  THROW_ASSOCIATION_CUT                  =   10.;      // Max distance to set initial values for tracing photons (cm)

    public double  RICH_TIME_RMS                          =   1.5;      // Expected single photon Time resolution (ns)
    public double  ALIGN_SHIFT_SCALE                      =   1.0;      // Scale factor for misalignment shifts
    public double  ALIGN_ANGLE_SCALE                      =   1.0;      // Scale factor for misalignment angles

    public int     DEBUG_ALLCOST                          =   0;        // if 1 activate debug of All Constants and Parameters
    public int     DEBUG_GEOCOST                          =   0;        // if 1 activate debug of Geomatry Constants
    public int     DEBUG_RECOPAR                          =   0;        // Number of events to printout for debug of Parameters
    public int     DEBUG_CALCOST                          =   0;        // Number of events to printout for debug of Calibration Constants
    //public int     DEBUG_ALIGN_TABLE                      =   1;        // Number of events to printout for debug of Alignment
    //public int     DEBUG_AERO_OPTICS                      =   1;        // Number of events to printout for debug of Aerogel Nominal Optics
    public int     DEBUG_PROC_TIME                        =   0;        // if 1 activate the sub-process time consumption printout

    public static final double  RICH_BKG_PROBABILITY      =   1.e-5;    // Background probability for likelihood

    enum parlist{
        DO_ALIGNMENT (0, "I"), FORCE_DC_MATCH (1, "I"), ALIGN_SHIFT_SCALE (2, "D");

        private String partype;
        private int id;

        private parlist(int id, String partype){ this.id=id; this.partype = partype;}

        public String type() {return partype;} 
        public int    id() {return id;} 
    }

    //String par_filename = new String("calibration/rich/richModule1/reco_parameter.txt");

    // -----------------
    public RICHParameters() {
    // -----------------
    }


    //------------------------------
    public void load_CCDB(ConstantsManager manager, int run, int ncalls){
    //------------------------------

        int debugMode = 0;

        if(RICHConstants.RECOPAR_FROM_FILE==0){

            if(debugMode>0){
                System.out.format("------------------------------------------------------------- \n");
                System.out.format("RICH: Load reconstruction parameters from CCDB for run %6d \n", run);
                System.out.format("------------------------------------------------------------- \n");
            }
            init_ParametersCCDB( manager.getConstants(run, "/calibration/rich/parameterss") );

        }else{

            if(debugMode>0)System.out.format("RICHFactory: Load calibration parameters from TxT\n");
            init_ParametersTxT();
        }

        if((debugMode>=1 || DEBUG_RECOPAR>=1) && ncalls<DEBUG_RECOPAR) dump_Parameters(run);
 
    }

    //------------------------------
    public void init_ParametersCCDB(IndexedTable paraConstants) {
    //------------------------------

        int debugMode = 0;

        /*
        * RECONSTRUCTION PARAMETERS
        */

        DO_ALIGNMENT                =  paraConstants.getIntValue("flag1", 4, 0, 0);
        FORCE_DC_MATCH              =  paraConstants.getIntValue("flag2", 4, 0, 0);
        ALIGN_RICH_REF              =  paraConstants.getIntValue("flag3", 4, 0, 0);
        ALIGN_PMT_PIVOT             =  paraConstants.getIntValue("flag4", 4, 0, 0);
        APPLY_SURVEY                =  paraConstants.getIntValue("flag5", 4, 0, 0);

        DO_ANALYTIC                 =  paraConstants.getIntValue("flag6", 4, 0, 0);
        THROW_ELECTRONS             =  paraConstants.getIntValue("flag7", 4, 0, 0);
        THROW_PIONS                 =  paraConstants.getIntValue("flag8", 4, 0, 0);
        THROW_KAONS                 =  paraConstants.getIntValue("flag9", 4, 0, 0);
        THROW_PROTONS               =  paraConstants.getIntValue("flag10", 4, 0, 0);
        THROW_PHOTON_NUMBER         =  paraConstants.getIntValue("flag11", 4, 0, 0);
        TRACE_PHOTONS               =  paraConstants.getIntValue("flag12", 4, 0, 0);

        REDO_RICH_RECO              =  paraConstants.getIntValue("flag13", 4, 0, 0);
        DO_MIRROR_HADS              =  paraConstants.getIntValue("flag14", 4, 0, 0);
        DO_CURVED_AERO              =  paraConstants.getIntValue("flag15", 4, 0, 0);

        USE_ELECTRON_ANGLES         =  paraConstants.getIntValue("flag16", 4, 0, 0);
        USE_PIXEL_PROPERTIES        =  paraConstants.getIntValue("flag17", 4, 0, 0);
        SAVE_THROWS                 =  paraConstants.getIntValue("flag18", 4, 0, 0);
        QUADRANT_NUMBER             =  paraConstants.getIntValue("flag19", 4, 0, 0);

        GOODHIT_FRAC                =  paraConstants.getDoubleValue("par1", 4, 0, 0);
        RICH_DCMATCH_CUT            =  paraConstants.getDoubleValue("par2", 4, 0, 0);
        RICH_HITMATCH_RMS           =  paraConstants.getDoubleValue("par3", 4, 0, 0);
        RICH_DIRECT_RMS             =  paraConstants.getDoubleValue("par4", 4, 0, 0) / 1000.;
        SHOW_PROGRESS_INTERVAL      =  paraConstants.getDoubleValue("par5", 4, 0, 0);
        THROW_ASSOCIATION_CUT       =  paraConstants.getDoubleValue("par6", 4, 0, 0);

        DEBUG_ALLCOST               =  (int) paraConstants.getDoubleValue("par7", 4, 0, 0);
        RICH_TIME_RMS               =  paraConstants.getDoubleValue("par8", 4, 0, 0);
        ALIGN_SHIFT_SCALE           =  paraConstants.getDoubleValue("par9", 4, 0, 0);
        ALIGN_ANGLE_SCALE           =  paraConstants.getDoubleValue("par10", 4, 0, 0);

        // pass1: uniformate all the debugs to the general one
        //ATT da togliare 
       // DEBUG_ALLCOST               =  1;
       // DEBUG_GEOCOST               =  DEBUG_ALLCOST;
        //DEBUG_RECOPAR               =  DEBUG_ALLCOST;
        //DEBUG_CALCOST               =  DEBUG_ALLCOST;

    }


    //------------------------------
    public void init_ParametersTxT() {
    //------------------------------

        /*int debugMode = 0;

        IndexedTable prova = new IndexedTable(3, "DO_ALIGNMENT/I:FORCE_DC_MATCHD");

        try {

            BufferedReader bf = new BufferedReader(new FileReader(par_filename));
            String currentLine = null;

            while ( (currentLine = bf.readLine()) != null) {

                String[] array = currentLine.split(" ");
                int    isec    = Integer.parseInt(array[0]);
                int    ila     = Integer.parseInt(array[1]);
                int    ico     = Integer.parseInt(array[2]);
                String name    = array[4];

                if(name.equals("DO_ALIGNMENT") DO_ALIGNMENT = Integer.parseInt(array[2]);
                String type    = array[3];
                if(type.equals("I") ival = 
                double val     = Double.parseDouble(array[5]);

                prova.addEntry(isec, ila, ico);
                prova.setDoubleValue(val, "value", isec, ila, ico);
                int id = parlist.valueOf(name).id();
                prova.setIntValue(id, "name", isec, ila, ico);

                 double eval  = prova.getDoubleValue("value", isec, ila, ico);
                 int etype = prova.getIntValue("name", isec, ila, ico);

                 System.out.format(" PROVA Value %s %7.2f -->  %4d %7.2f \n",name, val, etype, eval);
            }

        } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", par_filename);
                e.printStackTrace();
        }*/

    }

 
    //------------------------------
    public void dump_Parameters(int run) {
    //------------------------------

        System.out.format("dump_Parameters");
        if(run==11){
            // Only geometry parameters

            System.out.format(" \n");
            System.out.format("CCDB RICH PARA    DO_ALIGNMENT                 %7d \n", DO_ALIGNMENT);
            System.out.format("CCDB RICH PARA    FORCE_DC_MATCH               %9d \n", FORCE_DC_MATCH);
            System.out.format("CCDB RICH PARA    ALIGN_RICH_REF               %7d \n", ALIGN_RICH_REF);
            System.out.format("CCDB RICH PARA    ALIGN_PMT_PIVOT              %7d \n", ALIGN_PMT_PIVOT);
            System.out.format("CCDB RICH PARA    APPLY_SURVEY                 %7d \n", APPLY_SURVEY);

            System.out.format("CCDB RICH PARA    ALIGN_SHIFT_SCALE            %7.3f \n", ALIGN_SHIFT_SCALE);
            System.out.format("CCDB RICH PARA    ALIGN_ANGLE_SCALE            %7.3f \n \n", ALIGN_ANGLE_SCALE);

        }else{
            // All parameters

            System.out.format(" \n");
            System.out.format("CCDB RICH PARA    DO_ALIGNMENT                 %7d \n", DO_ALIGNMENT);
            System.out.format("CCDB RICH PARA    FORCE_DC_MATCH               %9d \n", FORCE_DC_MATCH);
            System.out.format("CCDB RICH PARA    ALIGN_RICH_REF               %7d \n", ALIGN_RICH_REF);
            System.out.format("CCDB RICH PARA    ALIGN_PMT_PIVOT              %7d \n", ALIGN_PMT_PIVOT);
            System.out.format("CCDB RICH PARA    APPLY_SURVEY                 %7d \n", APPLY_SURVEY);

            System.out.format("CCDB RICH PARA    DO_ANALYTIC                  %9d \n", DO_ANALYTIC);
            System.out.format("CCDB RICH PARA    THROW_ELECTRONS              %9d \n", THROW_ELECTRONS);
            System.out.format("CCDB RICH PARA    THROW_PIONS                  %9d \n", THROW_PIONS);
            System.out.format("CCDB RICH PARA    THROW_KAONS                  %9d \n", THROW_KAONS);
            System.out.format("CCDB RICH PARA    THROW_PROTONS                %9d \n", THROW_PROTONS);
            System.out.format("CCDB RICH PARA    THROW_PHOTON_NUMBER          %9d \n", THROW_PHOTON_NUMBER);
            System.out.format("CCDB RICH PARA    TRACE_PHOTONS                %9d \n", TRACE_PHOTONS);

            System.out.format("CCDB RICH PARA    REDO_RICH_RECO               %9d \n", REDO_RICH_RECO);
            System.out.format("CCDB RICH PARA    DO_MIRROR_HADS               %9d \n", DO_MIRROR_HADS);
            System.out.format("CCDB RICH PARA    DO_CURVED_AERO               %9d \n", DO_CURVED_AERO);

            System.out.format("CCDB RICH PARA    USE_ELECTRON_ANGLES          %9d \n", USE_ELECTRON_ANGLES);
            System.out.format("CCDB RICH PARA    USE_PIXEL_PROPERTIES         %9d \n", USE_PIXEL_PROPERTIES);
            System.out.format("CCDB RICH PARA    SAVE_THROWS                  %9d \n", SAVE_THROWS);
            System.out.format("CCDB RICH PARA    QUADRANT_NUMBER              %9d \n \n", QUADRANT_NUMBER);

            System.out.format("CCDB RICH PARA    GOODHIT_FRAC                 %9.4f \n", GOODHIT_FRAC);
            System.out.format("CCDB RICH PARA    RICH_DCMATCH_CUT             %9.4f \n", RICH_DCMATCH_CUT);
            System.out.format("CCDB RICH PARA    RICH_HITMATCH_RMS            %9.4f \n", RICH_HITMATCH_RMS);
            System.out.format("CCDB RICH PARA    RICH_DIRECT_RMS              %9.4f \n", RICH_DIRECT_RMS);
            System.out.format("CCDB RICH PARA    SHOW_PROGRESS_INTERVAL       %9.4f \n", SHOW_PROGRESS_INTERVAL);
            System.out.format("CCDB RICH PARA    THROW_ASSOCIATION_CUT        %9.4f \n", THROW_ASSOCIATION_CUT);

            System.out.format("CCDB RICH PARA    RICH_TIME_RMS                %9.4f \n", RICH_TIME_RMS);
            System.out.format("CCDB RICH PARA    ALIGN_SHIFT_SCALE            %7.3f \n", ALIGN_SHIFT_SCALE);
            System.out.format("CCDB RICH PARA    ALIGN_ANGLE_SCALE            %7.3f \n", ALIGN_ANGLE_SCALE);

            System.out.format("CCDB RICH PARA    DEBUG_ALLCOST                %9d \n", DEBUG_ALLCOST);
            System.out.format("CCDB RICH PARA    DEBUG_GEOCOST                %9d \n", DEBUG_GEOCOST);
            System.out.format("CCDB RICH PARA    DEBUG_RECOPAR                %9d \n", DEBUG_RECOPAR);
            System.out.format("CCDB RICH PARA    DEBUG_CALCOST                %9d \n", DEBUG_CALCOST);
            System.out.format(" \n");

        }
    }

}
