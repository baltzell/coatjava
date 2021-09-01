package org.jlab.detector.geom.RICH;

import org.jlab.detector.calib.utils.ConstantsManager;
import org.jlab.utils.groups.IndexedTable;

public class RICHGeoParameters {

    // Default values of RICH Reconstruction parameters to be re-loaded from CCDB or TxT

    public int     DO_ALIGNMENT                           =   1;        // if 1 apply alignment
    public int     ALIGN_RICH_REF                         =   1;        // if 1 use local RICH frame (instead of Lab frame)
    public int     ALIGN_PMT_PIVOT                        =   1;        // if 1 use MAPMT barycenter for rotations
    public int     APPLY_SURVEY                           =   0;        // if 1 apply the survey data for misalignment

    public double  ALIGN_SHIFT_SCALE                      =   1.0;      // Scale factor for misalignment shifts
    public double  ALIGN_ANGLE_SCALE                      =   1.0;      // Scale factor for misalignment angles
    
    public int     DEBUG_GEO_PARAMS                       =   0;        // if 1 activate debug of geometry parameters
    public int     DEBUG_GEO_CONSTS                       =   0;        // if 1 activate debug of geometry calibration
    public int     GEOPAR_FROM_FILE                       =   0;        // if 1 read geometry parameters from file


    // ----------------
    public RICHGeoParameters() {
    // ----------------
    }


    //------------------------------
    public void load_CCDB(ConstantsManager manager, int run, int ncalls){
    //------------------------------

        int debugMode = 0;

        init_ParametersCCDB( manager.getConstants(run, "/calibration/rich/parameterss") );

        if(debugMode>0 || DEBUG_GEO_PARAMS>=1) {
            System.out.format("------------------------------------------------------------- \n");
            System.out.format("RICH: Load RECO parameters from CCDB for run %6d \n", run);
            System.out.format("------------------------------------------------------------- \n");
 
            dump_Parameters();
        }

        if(GEOPAR_FROM_FILE==1){

            init_ParametersTxT();

            if(debugMode>=1 || DEBUG_GEO_PARAMS>=1) {
                System.out.format("------------------------------------------------------------- \n");
                System.out.format("RICH: Load RECO parameters from local TxT file for run %6d \n", run);
                System.out.format("------------------------------------------------------------- \n");

                dump_Parameters();
            }
        }

    }

    //------------------------------
    public void init_ParametersCCDB(IndexedTable paraConstants) {
    //------------------------------

        int debugMode = 0;

        DO_ALIGNMENT                =  paraConstants.getIntValue("flag1", 4, 0, 0);
        ALIGN_RICH_REF              =  paraConstants.getIntValue("flag3", 4, 0, 0);
        ALIGN_PMT_PIVOT             =  paraConstants.getIntValue("flag4", 4, 0, 0);
        APPLY_SURVEY                =  paraConstants.getIntValue("flag5", 4, 0, 0);

        ALIGN_SHIFT_SCALE           =  paraConstants.getDoubleValue("par9", 4, 0, 0);
        ALIGN_ANGLE_SCALE           =  paraConstants.getDoubleValue("par10", 4, 0, 0);

        DEBUG_GEO_PARAMS            =  (int) paraConstants.getDoubleValue("par7", 4, 0, 0);

        // QUE: da rimuovere nella produzione
        //DEBUG_GEO_PARAMS            =  1;
        //DEBUG_GEO_CONSTS            =  1;

    }

    //------------------------------
    public void init_ParametersTxT() {
    //------------------------------

        int debugMode = 0;

        IndexedTable prova = new IndexedTable(3, "first/D:second/I");

        prova.addEntry(1,1,1);

        prova.setDoubleValue(0.455, "first", 1,1,1);
        prova.setIntValue(776, "first", 1,1,1);

        double value = prova.getDoubleValue("first", 1,1,1);
        int ivalue = prova.getIntValue("second", 1,1,1);

        System.out.format(" Value %7.2f %6d \n",value,ivalue);

    }


    //------------------------------
    public void dump_Parameters() {
    //------------------------------

        System.out.format(" \n");
        System.out.format("CCDB RICH PARA    DO_ALIGNMENT                 %7d \n", DO_ALIGNMENT);
        System.out.format("CCDB RICH PARA    ALIGN_RICH_REF               %7d \n", ALIGN_RICH_REF);
        System.out.format("CCDB RICH PARA    ALIGN_PMT_PIVOT              %7d \n", ALIGN_PMT_PIVOT);
        System.out.format("CCDB RICH PARA    APPLY_SURVEY                 %7d \n", APPLY_SURVEY);

        System.out.format("CCDB RICH PARA    ALIGN_SHIFT_SCALE            %7.3f \n", ALIGN_SHIFT_SCALE);
        System.out.format("CCDB RICH PARA    ALIGN_ANGLE_SCALE            %7.3f \n \n", ALIGN_ANGLE_SCALE);

        System.out.format("CCDB RICH PARA    DEBUG_GEO_PARAMS             %7d \n", DEBUG_GEO_PARAMS);
        System.out.format("CCDB RICH PARA    DEBUG_GEO_CONSTS             %7d \n", DEBUG_GEO_CONSTS);
        System.out.format("CCDB RICH PARA    GEOPAR_FROM_FILE             %7d \n \n", GEOPAR_FROM_FILE);
     }

}
