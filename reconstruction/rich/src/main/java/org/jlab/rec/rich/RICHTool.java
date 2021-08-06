/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.jlab.rec.rich;

import org.jlab.detector.geom.RICH.RICHComponent;
import org.jlab.detector.geom.RICH.RICHIntersection;
import org.jlab.detector.geom.RICH.RICHFrame;
import org.jlab.detector.geom.RICH.RICHLayer;
import org.jlab.detector.geom.RICH.RICHRay;
import org.jlab.detector.geom.RICH.RICHComponent;
import org.jlab.detector.geom.RICH.RICHIntersection;
import org.jlab.detector.geom.RICH.RICHGeoConstants;
import org.jlab.detector.geom.RICH.RICHGeoFactory;
import java.io.FileReader;
import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;

import org.jlab.detector.calib.utils.ConstantsManager;
import org.jlab.detector.geant4.v2.RICHGeant4Factory;
import org.jlab.detector.volume.G4Stl;
import org.jlab.detector.volume.G4Box;

import eu.mihosoft.vrl.v3d.Vector3d;
import eu.mihosoft.vrl.v3d.Vertex;
import eu.mihosoft.vrl.v3d.Polygon;   
import eu.mihosoft.vrl.v3d.CSG;   
import org.jlab.geometry.prim.Line3d;   

import org.jlab.geom.prim.Vector3D;
import org.jlab.geom.prim.Line3D;
import org.jlab.geom.prim.Plane3D;
import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Sphere3D;   
import org.jlab.geom.prim.Triangle3D;   
import org.jlab.geom.prim.Face3D;   
import org.jlab.geom.prim.Shape3D;   

import org.jlab.utils.groups.IndexedTable;

/**
 *
 * @author mcontalb
 */
public class RICHTool{

    public final static RICHGeoConstants geocost = new RICHGeoConstants();
    public RICHRecParameters recpar = new RICHRecParameters();

    private final static int NLAY   =  geocost.NLAY;
    private final static int NPMT   =  geocost.NPMT;
    private final static int NPIX   =  geocost.NPIX;
    private final static int NCOMPO =  geocost.NCOMPO;
    private int              ncalls =  0;
    private int              NCMAX  =  10;

    private double pmt_timeoff[][] = new double[NPMT][NPIX];
    private double pmt_timewalk[][] = new double[NPMT][4];
    private double pixel_gain[][] = new double[NPMT][NPIX];
    private double pixel_eff[][] = new double[NPMT][NPIX];
    private int pixel_flag[][] = new int [NPMT][NPIX];                   // 0 = dead, 1 = ok, 2= hot
    private int pixel_ntime[][] = new int [NPMT][NPIX];                  // 0 = dead, 1 = ok, 2= hot
    private double pixel_mtime[][] = new double [NPMT][NPIX];            // 0 = dead, 1 = ok, 2= hot
    private double pixel_stime[][] = new double [NPMT][NPIX];            // 0 = dead, 1 = ok, 2= hot

    private double aero_chele_dir[][][] = new double[4][31][225];
    private double aero_chele_lat[][][] = new double[4][31][225];
    private double aero_chele_spe[][][] = new double[4][31][225];
    private double aero_schele_dir[][][] = new double[4][31][225];
    private double aero_schele_lat[][][] = new double[4][31][225];
    private double aero_schele_spe[][][] = new double[4][31][225];

    private Vector3D layer_misa_angle[][] = new Vector3D[NLAY+1][NCOMPO+1];
    private Vector3D layer_misa_shift[][] = new Vector3D[NLAY+1][NCOMPO+1];
    private RICHFrame rich_frame = new RICHFrame();
    private RICHFrame survey_frame = new RICHFrame();

    public RICHGeoFactory rgeo = null;

    //private List<RICHLayer> opticlayers = new ArrayList<RICHLayer>();

    private final static int NTIME = 10;
    private long RICH_START_TIME = (long) 0;
    private long RICH_LAST_TIME = (long) 0;
    private double richprocess_time[] = new double[NTIME];
    private int richprocess_ntimes[] = new int[NTIME];



    //------------------------------
    public RICHTool() {
    //------------------------------
    }


    //------------------------------
    public RICHTool(RICHGeoFactory ebgeo, int ncalls){
    //------------------------------

        init_ProcessTime();

        this.rgeo = ebgeo;
        this.ncalls = ncalls;

    }


    //------------------------------
    public void load_CCDB(ConstantsManager manager, int run){
    //------------------------------

    //start processing time
    int debugMode = 0;


    if(RICHRecConstants.READ_FROM_FILES==0){

            if(debugMode>0){
                System.out.format("------------------------------------------------------------- \n");
                System.out.format("RICHRecFactory: Load calibration parameters from CCDB for run %6d \n", run);
                System.out.format("------------------------------------------------------------- \n");
            }
            init_RecParametersCCDB( manager.getConstants(run, "/calibration/rich/parameterss") );
            init_CalParametersCCDB( manager.getConstants(run, "/calibration/rich/time_walk"),
                                    manager.getConstants(run, "/calibration/rich/time_offset"),
                                    manager.getConstants(run, "/calibration/rich/electro"),
                                    manager.getConstants(run, "/calibration/rich/pixels") );
        }else{

            if(debugMode>0)System.out.format("RICHRecFactory: Load calibration parameters from TxT\n");
            init_CalParametersTxT(2);
        }

    }


    //------------------------------
    public RICHRecParameters get_RecParameters() {return recpar;}
    //------------------------------


    //------------------------------
    public void init_CalParametersCCDB(IndexedTable timewalkConstants, IndexedTable timeoffConstants, IndexedTable cheleConstants, IndexedTable pixelConstants){
    //------------------------------

        int debugMode = 0;

        /*
        * TIME_OFFSETs
        */

        for(int ipmt=0; ipmt<NPMT; ipmt++){
            for(int ich=0; ich<NPIX; ich++){
                pmt_timeoff[ipmt][ich] = (float) timeoffConstants.getDoubleValue("offset", 4, ipmt+1, ich+1);
            }
            if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX){
                if(ipmt<10 || ipmt>380)System.out.format("CCDB RICH TOFF    ipmt %4d  %8.3f (ch1)  %8.3f (ch2)  %8.3f (ch63)  %8.3f (ch64) \n", ipmt+1,
                   pmt_timeoff[ipmt][0], pmt_timeoff[ipmt][1], pmt_timeoff[ipmt][62], pmt_timeoff[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB RICH TOFF     ....... \n");
                if(ipmt==390)System.out.format("  \n");
            }
        }

        /*
        *  TIME_WALKs
        */

        // TODO: time_walk bank definition is wrong
        for(int ipmt=0; ipmt<NPMT; ipmt++){
            pmt_timewalk[ipmt][0] = (float) timewalkConstants.getDoubleValue("D0", 4, ipmt+1, 0);
            pmt_timewalk[ipmt][1] = (float) timewalkConstants.getDoubleValue("m1", 4, ipmt+1, 0);
            pmt_timewalk[ipmt][2] = (float) Math.abs(timewalkConstants.getDoubleValue("m2", 4, ipmt+1, 0))*(-1.0);
            pmt_timewalk[ipmt][3] = (float) timewalkConstants.getDoubleValue("T0", 4, ipmt+1, 0);
            if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX){
                if(ipmt<10 || ipmt>380)System.out.format("CCDB RICH TWALK   ipmt %4d  D0 = %8.3f  T0 = %8.3f  m1 = %8.4f  m2 = %8.4f\n", ipmt+1,
                         pmt_timewalk[ipmt][0], pmt_timewalk[ipmt][1] , pmt_timewalk[ipmt][2], pmt_timewalk[ipmt][3]);
                if(ipmt==10)System.out.format("CCDB RICH TWALK    ....... \n");
                if(ipmt==390)System.out.format("  \n");
            }
        }

        /*
        * AEROGEL CALIBRATED OPTCIS (USING ELECTRON CONTROL SAMPLE)
        */

        int ndo[] = {16,22,31,31};
        double mrad = RICHRecConstants.MRAD;
        for (int ila=0; ila<4; ila++){
            for (int ico=0; ico<ndo[ila]*225; ico++){
                int itil = (int) ico/225;
                int iqua  = (int) ico - itil*225;

                aero_chele_dir[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("ch_dir", 4,201+ila,ico+1);
                aero_chele_lat[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("ch_lat", 4,201+ila,ico+1);
                aero_chele_spe[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("ch_spe", 4,201+ila,ico+1);

                aero_schele_dir[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("s_dir", 4,201+ila,ico+1);
                aero_schele_lat[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("s_lat", 4,201+ila,ico+1);
                aero_schele_spe[ila][itil][iqua] = (float) cheleConstants.getDoubleValue("s_spe", 4,201+ila,ico+1);

                if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX){
                    if( (itil<2 || itil>ndo[ila]-3) && (iqua==0 || iqua==224)) {
                        System.out.format("CCDB RICH CHELE   ila %4d  itile %3d  iq %4d dir = %7.2f  %7.2f  lat = %7.2f  %7.2f  spe = %7.2f  %7.2f \n", 201+ila, itil+1, iqua+1,
                        aero_chele_dir[ila][itil][iqua]*mrad, aero_schele_dir[ila][itil][iqua]*mrad,
                        aero_chele_lat[ila][itil][iqua]*mrad, aero_schele_lat[ila][itil][iqua]*mrad,
                        aero_chele_spe[ila][itil][iqua]*mrad, aero_schele_spe[ila][itil][iqua]*mrad);
                    }
                    if(ila==3 && ico==ndo[ila]*225-1)System.out.format("  \n");
                }
            }
        }

        /*
        * PIXELS
        */

        for(int ipmt=0; ipmt<NPMT; ipmt++){
            for(int ich=0; ich<NPIX; ich++){
                pixel_gain[ipmt][ich] = (float) pixelConstants.getDoubleValue("gain", 4, ipmt+1, ich+1);
                pixel_eff[ipmt][ich] = (float) pixelConstants.getDoubleValue("efficiency", 4, ipmt+1, ich+1);
                pixel_flag[ipmt][ich] = (int) pixelConstants.getIntValue("status", 4, ipmt+1, ich+1);
                
                pixel_ntime[ipmt][ich] = (int) pixelConstants.getIntValue("N_t", 4, ipmt+1, ich+1);
                pixel_mtime[ipmt][ich] = (float) pixelConstants.getDoubleValue("mean_t", 4, ipmt+1, ich+1);
                pixel_stime[ipmt][ich] = (float) pixelConstants.getDoubleValue("sigma_t", 4, ipmt+1, ich+1);
            }
            if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX){
                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL GAIN    ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt+1,
                   pixel_gain[ipmt][0], pixel_gain[ipmt][1], pixel_gain[ipmt][62], pixel_gain[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL GAIN     ....... \n");

                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL EFF     ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt+1,
                   pixel_eff[ipmt][0], pixel_eff[ipmt][1], pixel_eff[ipmt][62], pixel_eff[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL EFF      ....... \n");

                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL STATUS  ipmt %4d  %8d (ch1)  %8d (ch2)  %8d (ch63)  %8d (ch64) \n", ipmt+1,
                   pixel_flag[ipmt][0], pixel_flag[ipmt][1], pixel_flag[ipmt][62], pixel_flag[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL STATUS   ....... \n");

                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL NTIME   ipmt %4d  %8d (ch1)  %8d (ch2)  %8d (ch63)  %8d (ch64) \n", ipmt+1,
                   pixel_ntime[ipmt][0], pixel_ntime[ipmt][1], pixel_ntime[ipmt][62], pixel_ntime[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL NTIME    ....... \n");

                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL MTIME   ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt+1,
                   pixel_mtime[ipmt][0], pixel_mtime[ipmt][1], pixel_mtime[ipmt][62], pixel_mtime[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL MTIME    ....... \n");

                if(ipmt<2 || ipmt>388)System.out.format("CCDB PIXEL STIME   ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt+1,
                   pixel_stime[ipmt][0], pixel_stime[ipmt][1], pixel_stime[ipmt][62], pixel_stime[ipmt][63]);
                if(ipmt==10)System.out.format("CCDB PIXEL STIME    ....... \n");

                if(ipmt==390)System.out.format("  \n");
            }
        }

    }


    //------------------------------
    public void init_RecParametersCCDB(IndexedTable paraConstants) {
    //------------------------------

        int debugMode = 0;

        /*
        * RECONSTRUCTION PARAMETERS
        */

        recpar.FORCE_DC_MATCH              =  paraConstants.getIntValue("flag2", 4, 0, 0);

        recpar.DO_ANALYTIC                 =  paraConstants.getIntValue("flag6", 4, 0, 0);
        recpar.THROW_ELECTRONS             =  paraConstants.getIntValue("flag7", 4, 0, 0);
        recpar.THROW_PIONS                 =  paraConstants.getIntValue("flag8", 4, 0, 0);
        recpar.THROW_KAONS                 =  paraConstants.getIntValue("flag9", 4, 0, 0);
        recpar.THROW_PROTONS               =  paraConstants.getIntValue("flag10", 4, 0, 0);
        recpar.THROW_PHOTON_NUMBER         =  paraConstants.getIntValue("flag11", 4, 0, 0);
        recpar.TRACE_PHOTONS               =  paraConstants.getIntValue("flag12", 4, 0, 0);

        recpar.REDO_RICH_RECO              =  paraConstants.getIntValue("flag13", 4, 0, 0);
        recpar.DO_MIRROR_HADS              =  paraConstants.getIntValue("flag14", 4, 0, 0);  
        recpar.DO_CURVED_AERO              =  paraConstants.getIntValue("flag15", 4, 0, 0);  

        recpar.USE_ELECTRON_ANGLES         =  paraConstants.getIntValue("flag16", 4, 0, 0);
        recpar.USE_PIXEL_PROPERTIES        =  paraConstants.getIntValue("flag17", 4, 0, 0);
        recpar.SAVE_THROWS                 =  paraConstants.getIntValue("flag18", 4, 0, 0);
        recpar.QUADRANT_NUMBER             =  paraConstants.getIntValue("flag19", 4, 0, 0);

        recpar.GOODHIT_FRAC                =  paraConstants.getDoubleValue("par1", 4, 0, 0);
        recpar.RICH_DCMATCH_CUT            =  paraConstants.getDoubleValue("par2", 4, 0, 0);
        recpar.RICH_HITMATCH_RMS           =  paraConstants.getDoubleValue("par3", 4, 0, 0);
        recpar.RICH_DIRECT_RMS             =  paraConstants.getDoubleValue("par4", 4, 0, 0) / 1000.;
        recpar.SHOW_PROGRESS_INTERVAL      =  paraConstants.getDoubleValue("par5", 4, 0, 0);
        recpar.THROW_ASSOCIATION_CUT       =  paraConstants.getDoubleValue("par6", 4, 0, 0);

        recpar.RICH_REC_DEBUG              =  paraConstants.getDoubleValue("par7", 4, 0, 0);
        recpar.RICH_TIME_RMS               =  paraConstants.getDoubleValue("par8", 4, 0, 0);
        
        //TODO: check
        //recpar.RICH_REC_DEBUG              =  1.0;
        //recpar.QUADRANT_NUMBER             =  5;
        //recpar.USE_ELECTRON_ANGLES         =  1;
        //recpar.USE_PIXEL_PROPERTIES        =  1;

        if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX){

            System.out.format(" \n");
            System.out.format("CCDB RICH PARA    FORCE_DC_MATCH               %9d \n", recpar.FORCE_DC_MATCH); 

            System.out.format("CCDB RICH PARA    DO_ANALYTIC                  %9d \n", recpar.DO_ANALYTIC); 
            System.out.format("CCDB RICH PARA    THROW_ELECTRONS              %9d \n", recpar.THROW_ELECTRONS); 
            System.out.format("CCDB RICH PARA    THROW_PIONS                  %9d \n", recpar.THROW_PIONS); 
            System.out.format("CCDB RICH PARA    THROW_KAONS                  %9d \n", recpar.THROW_KAONS); 
            System.out.format("CCDB RICH PARA    THROW_PROTONS                %9d \n", recpar.THROW_PROTONS); 
            System.out.format("CCDB RICH PARA    THROW_PHOTON_NUMBER          %9d \n", recpar.THROW_PHOTON_NUMBER); 
            System.out.format("CCDB RICH PARA    TRACE_PHOTONS                %9d \n", recpar.TRACE_PHOTONS); 

            System.out.format("CCDB RICH PARA    REDO_RICH_RECO               %9d \n", recpar.REDO_RICH_RECO); 
            System.out.format("CCDB RICH PARA    DO_MIRROR_HADS               %9d \n", recpar.DO_MIRROR_HADS); 
            System.out.format("CCDB RICH PARA    DO_CURVED_AERO               %9d \n", recpar.DO_CURVED_AERO); 

            System.out.format("CCDB RICH PARA    USE_ELECTRON_ANGLES          %9d \n", recpar.USE_ELECTRON_ANGLES); 
            System.out.format("CCDB RICH PARA    USE_PIXEL_PROPERTIES         %9d \n", recpar.USE_PIXEL_PROPERTIES); 
            System.out.format("CCDB RICH PARA    SAVE_THROWS                  %9d \n", recpar.SAVE_THROWS);
            System.out.format("CCDB RICH PARA    QUADRANT_NUMBER              %9d \n \n", recpar.QUADRANT_NUMBER);

            System.out.format("CCDB RICH PARA    GOODHIT_FRAC                 %9.4f \n", recpar.GOODHIT_FRAC); 
            System.out.format("CCDB RICH PARA    RICH_DCMATCH_CUT             %9.4f \n", recpar.RICH_DCMATCH_CUT); 
            System.out.format("CCDB RICH PARA    RICH_HITMATCH_RMS            %9.4f \n", recpar.RICH_HITMATCH_RMS); 
            System.out.format("CCDB RICH PARA    RICH_DIRECT_RMS              %9.4f \n", recpar.RICH_DIRECT_RMS); 
            System.out.format("CCDB RICH PARA    SHOW_PROGRESS_INTERVAL       %9.4f \n", recpar.SHOW_PROGRESS_INTERVAL); 
            System.out.format("CCDB RICH PARA    THROW_ASSOCIATION_CUT        %9.4f \n", recpar.THROW_ASSOCIATION_CUT); 

            System.out.format("CCDB RICH PARA    RICH_REC_DEBUG               %9.4f \n", recpar.RICH_REC_DEBUG); 
            System.out.format("CCDB RICH PARA    RICH_TIME_RMS                %9.4f \n", recpar.RICH_TIME_RMS); 
            System.out.format(" \n");

        }

    }


    //------------------------------
    public void init_CalParametersTxT(int ifile){
    //------------------------------
    // To be moved to CCDB

        int debugMode = 0;

       if(ifile==2){
           /**
            * TIME_OFFSETs
            */
            String off_filename = new String("CALIB_DATA/MIRA/richTimeOffsets.out");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(off_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {

                    String[] array = currentLine.split(" ");
                    int ipmt = Integer.parseInt(array[0]);
                    int ich  = Integer.parseInt(array[1]);
                    float off = Float.parseFloat(array[4]);
                    pmt_timeoff[ipmt-1][ich-1] = off;

                    if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)if(ich==1 || ich==64)
                              System.out.format("TXT RICH TOFF   pmt %4d (ich=%3d: %8.2f) \n", ipmt, ich, pmt_timeoff[ipmt-1][ich-1]);

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", off_filename);
                e.printStackTrace();

            }


            /*
            *  TIME_WALKs
            */
            String walk_filename = new String("CALIB_DATA/MIRA/richTimeWalks.out");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(walk_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {

                    String[] array = currentLine.split(" ");
                    int ipmt = Integer.parseInt(array[0]);

                    if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)System.out.format("TXT WALK   pmt %d", ipmt);
                    for (int ich=1; ich<5; ich++){
                        float walk = Float.parseFloat(array[1+(ich-1)*2]);
                        if(ich==4 && walk<-1000)walk= (float)-0.100;
                        pmt_timewalk[ipmt-1][ich-1] = walk;
                        if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)System.out.format(" (%d, %8.4f) ", ich, pmt_timewalk[ipmt-1][ich-1]);
                    }
                    if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)System.out.format("\n");

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", walk_filename);
                e.printStackTrace();

            }
       }


        if(ifile==4){

           /*
            * AEROGEL CALIBRATED OPTICS
            */

            String chele_filename = new String("CALIB_DATA/aerogel_chele.txt");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(chele_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {

                    String[] array = currentLine.split(" ");
                    int idlay = Integer.parseInt(array[1]);
                    int iaer  = Integer.parseInt(array[2]);
                    int iqua  = Integer.parseInt(array[3]);

                    if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)System.out.format("Read chele for AERO lay %3d  compo %3d quadrant  %3d", idlay, iaer, iqua);

                    int ndir     = Integer.parseInt(array[4]);
                    float chdir  = Float.parseFloat(array[5]);
                    float sdir   = Float.parseFloat(array[6]);

                    int nlat     = Integer.parseInt(array[7]);
                    float chlat  = Float.parseFloat(array[8]);
                    float slat   = Float.parseFloat(array[9]);

                    int nspe     = Integer.parseInt(array[10]);
                    float chspe  = Float.parseFloat(array[11]);
                    float sspe   = Float.parseFloat(array[12]);

                    aero_chele_dir[idlay-201][iaer-1][iqua] = chdir;
                    aero_chele_lat[idlay-201][iaer-1][iqua] = chlat;
                    aero_chele_spe[idlay-201][iaer-1][iqua] = chspe;

                    aero_schele_dir[idlay-201][iaer-1][iqua] = sdir;
                    aero_schele_lat[idlay-201][iaer-1][iqua] = slat;
                    aero_schele_spe[idlay-201][iaer-1][iqua] = sspe;

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", chele_filename);
                e.printStackTrace();
            }

            if((debugMode>=1 || recpar.RICH_REC_DEBUG>=1) && ncalls<NCMAX)System.out.format("initConstants: DONE \n");

        }

    }


    //------------------------------
    public boolean convert_indexes(int lla, int cco, int[] ind){
    //------------------------------

        int[] lateral_compo = {11,5,6,9,10,7,8};

        /*
        *  Aerogel
        */
	if(lla==0 && cco==0){ind[0]=lla; ind[1]=cco; return true;}
	if(lla>=201 && lla<=204 && cco==0){ind[0]=lla-200; ind[1]=cco; return true;}
        if(lla==301 && cco>0 && cco<=7) {ind[0]=lateral_compo[cco-1]; ind[1]=0; return true;}
        if(lla==302){
            if(cco==0){ind[0]=12; ind[1]=0; return true;}
            if(cco>0 && cco<11){ind[0]=12; ind[1]=cco; return true;}
        }
        if(lla==401 && cco==0){ind[0]=13; ind[1]=cco; return true;}

        return false;
    }


    //------------------------------
    public String toString(Vector3d vec, int qua) {
    //------------------------------
        if(qua==2)return String.format("%8.2f %8.2f %8.2f", vec.x, vec.y, vec.z);
        if(qua==3)return String.format("%8.3f %8.3f %8.3f", vec.x, vec.y, vec.z);
        if(qua==4)return String.format("%8.4f %8.4f %8.4f", vec.x, vec.y, vec.z);
        return String.format("%8.1f %8.1f %8.1f", vec.x, vec.y, vec.z);
 
    }

    //------------------------------
    public String toString(Vector3d vec) {
    //------------------------------
        return String.format("%8.3f %8.3f %8.3f", vec.x, vec.y, vec.z);
    }


    //------------------------------
    public String toString(Vector3D vec) {
    //------------------------------
        return String.format("%8.3f %8.3f %8.3f", vec.x(), vec.y(), vec.z());
    }

    //------------------------------
    public String toString(Point3D vec) {
    //------------------------------
        return String.format("%7.2f %7.2f %7.2f", vec.x(), vec.y(), vec.z());
    }


    //------------------------------
    //public Triangle3D toTriangle3D(Face3D face){ return new Triangle3D(face.point(0), face.point(1), face.point(2)); }
    //------------------------------

    //------------------------------
    //public ArrayList<Triangle3D> toTriangle3D(List<Polygon> pols){
    //------------------------------
    /*
        ArrayList<Triangle3D> trias = new ArrayList<Triangle3D>();

        for (Polygon pol: pols){
            for (int iv=2; iv<pol.vertices.size(); iv++){
                Triangle3D tri = new Triangle3D(toPoint3D(pol.vertices.get(0)), toPoint3D(pol.vertices.get(iv-1)), toPoint3D(pol.vertices.get(iv)));
                trias.add(tri);
            }
        }
        
        return trias;
    }*/


    //------------------------------
    public double get_sChElectron(int ila, int ico, int iqua, int irefle) {
    //------------------------------

        if(recpar.USE_ELECTRON_ANGLES==1){
            if(irefle==0){
                if(aero_schele_dir[ila][ico][iqua]>0){ 
                    return aero_schele_dir[ila][ico][iqua];
                }else{
                    if(aero_schele_lat[ila][ico][iqua]>0){ 
                        return aero_schele_lat[ila][ico][iqua];
                    }else{
                        return aero_schele_spe[ila][ico][iqua];
                    }
                }
            }
            if(irefle==1){
                if(aero_schele_lat[ila][ico][iqua]>0){ 
                    return aero_schele_lat[ila][ico][iqua];
                }else{
                    if(aero_schele_dir[ila][ico][iqua]>0){ 
                        return aero_schele_dir[ila][ico][iqua];
                    }else{
                        return aero_schele_spe[ila][ico][iqua];
                    }
                }
            }
            if(irefle==2){
                if(aero_schele_spe[ila][ico][iqua]>0){ 
                    return aero_schele_spe[ila][ico][iqua];
                }else{
                    if(aero_schele_dir[ila][ico][iqua]>0){ 
                        return aero_schele_dir[ila][ico][iqua];
                    }else{
                        return aero_schele_lat[ila][ico][iqua];
                    }
                }
            }
        }
        return 0.0;
    }
 

    //------------------------------
    public double get_PixelGain(int ipmt, int ich) { return pixel_gain[ipmt][ich]; }
    //------------------------------

    //------------------------------
    public double get_PixelEff(int ipmt, int ich) { return pixel_eff[ipmt][ich]; }
    //------------------------------

    //------------------------------
    public int get_PixelFlag(int ipmt, int ich) { return pixel_flag[ipmt][ich]; }
    //------------------------------

    //------------------------------
    public double get_PixelMtime(int ipmt, int ich) { return pixel_mtime[ipmt][ich]; }
    //------------------------------

    //------------------------------
    public double get_PixelStime(int ipmt, int ich) { return pixel_stime[ipmt][ich]; }
    //------------------------------

    //------------------------------
    public double get_ChElectron(int ila, int ico, int iqua, int irefle) {
    //------------------------------
 
        if(recpar.USE_ELECTRON_ANGLES==1){
            if(irefle==0){
                if(aero_chele_dir[ila][ico][iqua]>0){ 
                    return aero_chele_dir[ila][ico][iqua];
                }else{
                    if(aero_chele_lat[ila][ico][iqua]>0){ 
                        return aero_chele_lat[ila][ico][iqua];
                    }else{
                        return aero_chele_spe[ila][ico][iqua];
                    }
                }
            }
            if(irefle==1){
                if(aero_chele_lat[ila][ico][iqua]>0){ 
                    return aero_chele_lat[ila][ico][iqua];
                }else{
                    if(aero_chele_dir[ila][ico][iqua]>0){ 
                        return aero_chele_dir[ila][ico][iqua];
                    }else{
                        return aero_chele_spe[ila][ico][iqua];
                    }
                }
            }
            if(irefle==2){
                if(aero_chele_spe[ila][ico][iqua]>0){ 
                    return aero_chele_spe[ila][ico][iqua];
                }else{
                    if(aero_chele_dir[ila][ico][iqua]>0){ 
                        return aero_chele_dir[ila][ico][iqua];
                    }else{
                        return aero_chele_lat[ila][ico][iqua];
                    }
                }
            }
        }
        return 0.0;

    }


    //------------------------------
    public double getPMTtimeoff(int ipmt, int ich){
    //------------------------------
 
        return -1*pmt_timeoff[ipmt-1][ich-1];

    }

    //------------------------------
    public double getPMTtimewalk(int ipmt, int ich){
    //------------------------------
 
        return pmt_timewalk[ipmt-1][ich-1];

    }


    //------------------------------
    public int get_LayerNumber(String slay){
    //------------------------------
        int debugMode = 0;
        for (int ila=0; ila<NLAY; ila++){
            if(rgeo.get_Layer(ila).get_Name().equals(slay)) {
                if(debugMode>=1)System.out.format(" Find layer %s --> %4d \n",slay,ila);
                return ila;
            }
        }  
        return -1;
    }


    //------------------------------
    public RICHLayer get_Layer(String slay){
    //------------------------------
        for (int ila=0; ila<NLAY; ila++){
            if(rgeo.get_Layer(ila).get_Name().equals(slay)) {
                return rgeo.get_Layer(ila);
            }
        }  
        return null;
    }


    //------------------------------
    public RICHLayer get_Layer(int ilay){ 
    //------------------------------
        if(ilay>-1 && ilay<NLAY) return rgeo.get_Layer(ilay);
        return null; 
    }


    //------------------------------
    public RICHComponent get_Component(int ilay, int ico){ 
    //------------------------------
        return rgeo.get_Layer(ilay).get(ico);
    }


    //------------------------------
    public Vector3D toVector3D(Vector3d vin) {
    //------------------------------
        Vector3D vout = new Vector3D(vin.x, vin.y, vin.z); 
	return vout;
    }


    //------------------------------
    public Vector3D toVector3D(Point3D pin) {
    //------------------------------
        Vector3D vout = new Vector3D(pin.x(), pin.y(), pin.z()); 
	return vout;
    }

    //------------------------------
    public Vector3d toVector3d(Vertex ver) {return  new Vector3d(ver.pos.x, ver.pos.y, ver.pos.z); }
    //------------------------------

    //------------------------------
    public Vector3d toVector3d(Vector3D vin) {
    //------------------------------
        Vector3d vout = new Vector3d(vin.x(), vin.y(), vin.z()); 
	return vout;
    }

     //------------------------------
     public Vector3d toVector3d(Point3D pin) {
     //------------------------------
        Vector3d vout = new Vector3d(pin.x(), pin.y(), pin.z()); 
	return vout;
     }

     //------------------------------
     public Point3D toPoint3D(Vertex vin) {
     //------------------------------
        Point3D pout = new Point3D(vin.pos.x, vin.pos.y, vin.pos.z); 
	return pout;
     }

     //------------------------------
     public Point3D toPoint3D(Vector3D vin) {
     //------------------------------
        Point3D pout = new Point3D(vin.x(), vin.y(), vin.z()); 
	return pout;
     }


     //------------------------------
     public Point3D toPoint3D(Vector3d vin) {
     //------------------------------
        if(vin==null) return null;
        Point3D pout = new Point3D(vin.x, vin.y, vin.z); 
	return pout;
     }


     //------------------------------
     public Line3d toLine3d(Line3D lin) {
     //------------------------------
        Line3d lout = new Line3d(toVector3d(lin.origin()), toVector3d(lin.end()));
	return lout;
     }


     //------------------------------
     public Line3D toLine3D(Line3d lin) {
     //------------------------------
        Line3D lout = new Line3D(toPoint3D(lin.origin()), toPoint3D(lin.end()));
        return lout;
     }


    // ----------------
    //public Vector3d find_intersection_UpperHalf_RICH(Line3D ray){
    // ----------------

     /*   int debugMode = 0;
        RICHIntersection inter = get_Layer("mirror_sphere").find_Entrance(ray, -1);

        if(inter!=null){
            if(debugMode>=1)  System.out.format("find_intersection with SPHERICAL (%d, %d): %s\n",
                 inter.get_layer(), inter.get_component(), inter.get_pos().toStringBrief(2));
            return toVector3d(inter.get_pos());
        }else{
            if(debugMode>=1)  System.out.format("find NO intersection with SPHERICAL \n");
        }

        return null;

    }*/

    // ----------------
    //public Vector3d find_intersection_MAPMT(Line3D ray){
    // ----------------
    /*
        int debugMode = 0;

        if(debugMode>=1) System.out.format(" Ray %s \n",ray.origin().toStringBrief(2));
        RICHIntersection inter = get_Layer("mapmts").find_Entrance(ray, -1);

        if(inter!=null){
            if(debugMode>=1)  System.out.format("find_intersection with MAPMT (%d, %d): %s\n",
                 inter.get_layer(), inter.get_component(), inter.get_pos().toStringBrief(2));
            return toVector3d(inter.get_pos());
        }

        return null;
    }*/


    // ----------------
    public Vector3D Reflection(Vector3D vector1, Vector3D normal) {
    // ----------------

        int debugMode = 0;
        Vector3D vin = vector1.asUnit();
        Vector3D vnorm = normal.asUnit();

        double cosI  =  vin.dot(vnorm); 
        if(debugMode>=1)System.out.format("Vector in %s  vnorm %s cosI %7.3f \n ",vin.toStringBrief(3),vnorm.toStringBrief(3),cosI);
        if (cosI > 0) {
            if(debugMode>=1)System.out.format("ATT: Mirror normal parallel to impinging ray %7.3f \n",cosI);
            vnorm.scale(-1.0);
        }

        double refle = 2*(vin.dot(vnorm));
        Vector3D vout = vin.sub(vnorm.multiply(refle));

        if(debugMode>=1){
            System.out.format("Mirror normal %s\n",normal.toStringBrief(3));
            System.out.format("Reflected versor %s\n", vout.asUnit().toStringBrief(3));
        }

        return vout.asUnit();
    }

    // ----------------
    public Vector3D Transmission2(Vector3D vector1, Vector3D normal, double n_1, double n_2) {
    // ----------------

        int debugMode = 0;
        double rn = n_1 / n_2;

        Vector3D vin = vector1.asUnit();
        Vector3D vnorm = normal.asUnit();

        double cosI  =  vin.dot(vnorm); 
        if(debugMode>=1)System.out.format("Vector in %s  vnorm %s cosI %7.3f \n ",vin.toStringBrief(3),vnorm.toStringBrief(3),cosI);
        if (cosI < 0) {
            if(debugMode>=1)System.out.format("ATT: Mirror normal parallel to impinging ray %7.3f \n",cosI);
            vnorm.scale(-1.0);
        }
        if(debugMode>=1)System.out.format("Vector in %s  vnorm %s cosI %7.3f \n ",vin.toStringBrief(3),vnorm.toStringBrief(3),cosI);

        Vector3D vrot = (vnorm.cross(vin)).asUnit();
 
        double angi = Math.acos(vin.dot(vnorm)) ;
        double ango = Math.asin( rn * Math.sin(angi));

        Quaternion q = new Quaternion(ango, toVector3d(vrot));

        Vector3D vout = toVector3D(q.rotate(toVector3d(vnorm)));

        if(debugMode>=1){
            System.out.format(" vin   %s \n", vin.toStringBrief(3));
            System.out.format(" vnorm %s \n", vnorm.toStringBrief(3)); 
            System.out.format(" angles %7.3f %7.3f \n",angi*57.3, ango*57.3);
            System.out.format(" vout  %s \n", vout.toStringBrief(3)); 
        }

        return vout;

    }
 
    // ----------------
    public RICHRay OpticalRotation(RICHRay rayin, RICHIntersection intersection) {
    // ----------------

        int debugMode = 0;
        Point3D vori = rayin.origin();
        Vector3D inVersor = rayin.direction().asUnit();
        Vector3D newVersor = new Vector3D(0.0, 0.0, 0.0);
        RICHRay rayout = null;
        int type = 0;
 
        if(debugMode>=1)System.out.format("Ray for %3d %3d \n",intersection.get_layer(), intersection.get_component());
        //RICHComponent component = rgeo.get_Layer(intersection.get_layer()).get(intersection.get_component());
        RICHLayer layer = rgeo.get_Layer(intersection.get_layer());

        if(layer.is_optical()==true){
                
            if(debugMode>=1)System.out.format("Ray rotation at Optical compo %3d %3d  xyz %s \n", intersection.get_layer(), intersection.get_component(), vori.toStringBrief(2));
            Vector3D vnorm = intersection.get_normal();
            if(vnorm != null ){
                if(layer.is_mirror()==true){
             
                    newVersor = Reflection(inVersor, vnorm);
                    type=10000+intersection.get_layer()*100+intersection.get_component()+1;
                    if(debugMode>=1)System.out.format(" Reflection at mirror surface norm %s \n", vnorm.toStringBrief(3));

                }else{

                    newVersor = Transmission2(inVersor, vnorm, intersection.get_nin(), intersection.get_nout());
                    type=20000+intersection.get_layer()*100+intersection.get_component()+1;
                    if(debugMode>=1){
                        System.out.format(" Refraction at surface boundary norm %s \n", vnorm.toStringBrief(3));
                        System.out.format(" norm in %s %7.4f \n",vnorm.toStringBrief(3), vnorm.costheta());
                        System.out.format(" vers in %s %7.4f \n",inVersor.toStringBrief(3), inVersor.costheta());
                        System.out.format(" vers ou %s %7.4f \n",newVersor.toStringBrief(3), newVersor.costheta());
                    }
                }
            }

            if(debugMode>=1)System.out.format(" Versor in %s   --> out %s \n",inVersor.toStringBrief(3), newVersor.toStringBrief(3)); 
        }

        rayout = new RICHRay(vori, newVersor.multiply(200));
        rayout.set_type(type);
        return rayout;

    }

    // ----------------
    public ArrayList<RICHRay> RayTrace(Vector3d emission, int orilay, int orico, Vector3d vlab) {
    // ---------------- 

        int debugMode = 0;

        RICHLayer layer = get_Layer(orilay);
        if(debugMode>=1)System.out.format("Raytrace gets refractive index from CCDB database %8.5f \n",layer.get(orico).get_index());
        return RayTrace(emission, orilay, orico, vlab, layer.get(orico).get_index());

    }

    // ----------------
    public ArrayList<RICHRay> RayTrace(Vector3d emission, int orilay, int orico, Vector3d vlab, double naero) {
    // ---------------- 
    // return the hit position on the PMT plane of a photon emitted at emission with direction vlab

        int debugMode = 0;
        ArrayList<RICHRay> raytracks = new ArrayList<RICHRay>();

        Point3D emi = toPoint3D(emission);
        Vector3D vdir = toVector3D(vlab);

        RICHRay lastray = new RICHRay(emi, vdir.multiply(200));
        if(debugMode>=1) {
            System.out.format(" --------------------------- \n");
            System.out.format("Raytrace photon ori %s  olay %3d  oco %3d  dir %s \n",emi.toStringBrief(2),orilay,orico,vdir.toStringBrief(3)); 
            System.out.format(" --------------------------- \n");
        }

        RICHLayer layer = get_Layer(orilay);
        if(layer==null)return null;

        RICHIntersection first_intersection = null;
        if(recpar.DO_CURVED_AERO==1){
            first_intersection = layer.find_ExitCurved(lastray.asLine3D(), orico);
        }else{
            first_intersection = layer.find_Exit(lastray.asLine3D(), orico);
        }
        if(first_intersection==null)return null;   

        if(debugMode>=1){
            System.out.format(" first inter : ");
            first_intersection.showIntersection();
        }

        Point3D new_pos = first_intersection.get_pos();
        RICHRay oriray = new RICHRay(emi, new_pos);

        /* rewrite the refractive index to be consistent with photon theta
           only valid for initial aerogel
           the rest of components take ref index from CCDB database 
        */
        //oriray.set_refind(layer.get(orico).get_index());
        first_intersection.set_nin((float) naero);
        oriray.set_refind(naero);
        raytracks.add(oriray);

        RICHRay rayin = new RICHRay(new_pos, oriray.direction().multiply(200));
        lastray = OpticalRotation(rayin, first_intersection);
        lastray.set_refind(geocost.RICH_AIR_INDEX);
        RICHIntersection last_intersection = first_intersection;

        if(debugMode>=1){
            System.out.format(" add first ray : ");
            oriray.showRay();
            System.out.format(" get rotated ray : ");
            lastray.showRay();
        }

        int jj = 1;
        int front_nrefl = 0;
        boolean detected = false;
        boolean lost = false;
        while( detected == false && lost == false && raytracks.size()<10){

            Point3D last_ori  = lastray.origin();
            Point3D new_hit = null;
            RICHIntersection new_intersection = null;
            if(debugMode>=1)System.out.format(" ray-tracking step %d \n",jj);

            if(last_intersection.get_layer()<4){
  
                // planar mirrors
                RICHIntersection test_intersection = get_Layer("mirror_bottom").find_Entrance(lastray.asLine3D(), -1);
                if(test_intersection==null)test_intersection = get_Layer("mirror_left_L1").find_Entrance(lastray.asLine3D(), -1);
                if(test_intersection==null)test_intersection = get_Layer("mirror_right_L1").find_Entrance(lastray.asLine3D(), -1);
                if(test_intersection==null)test_intersection = get_Layer("mirror_left_L2").find_Entrance(lastray.asLine3D(), -1);
                if(test_intersection==null)test_intersection = get_Layer("mirror_right_L2").find_Entrance(lastray.asLine3D(), -1);
                if(test_intersection!=null){
                    if(debugMode>=1){
                        System.out.format(" test planar (z %7.2f, step %7.2f) : ",last_ori.z(), test_intersection.get_pos().distance(last_ori));
                        test_intersection.showIntersection();
                    }
                    //if(test_intersection.get_pos().distance(last_ori)>RICHConstants.PHOTON_DISTMIN_TRACING)new_intersection = test_intersection;
                    new_intersection = test_intersection;
                }else{
                    if(debugMode>=1)System.out.format(" no lateral mirror intersection \n");
                }

                // shperical mirrors
                if(lastray.direction().costheta()>0){
                    test_intersection = get_Layer("mirror_sphere").find_EntranceCurved(lastray.asLine3D(), -1);
                    
                    if(test_intersection!=null){
                        if(debugMode>=1){
                            System.out.format(" test sphere (z %7.2f, step %7.2f) : ",last_ori.z(), test_intersection.get_pos().distance(last_ori));
                            test_intersection.showIntersection();
                        }
                        //if(test_intersection.get_pos().distance(last_ori)>RICHConstants.PHOTON_DISTMIN_TRACING){
                            if(new_intersection==null || (new_intersection!=null && test_intersection.get_pos().z()<new_intersection.get_pos().z())) {
                                new_intersection = test_intersection;
                            }
                        //}
                    }else{
                        if(debugMode>=1)System.out.format(" no sphere intersection \n");
                    }

                    RICHIntersection pmt_inter = get_Layer("mapmts").find_Entrance(lastray.asLine3D(), -1);
                    if(pmt_inter!=null) {
                        Point3D test_hit = pmt_inter.get_pos(); 
                        //if(test_hit.distance(last_ori)>RICHConstants.PHOTON_DISTMIN_TRACING){
                            new_hit=test_hit;
                            if(debugMode>=1)System.out.format(" test PMT : Hit %s \n",new_hit.toStringBrief(2));
                        //}else{
                            //if(debugMode>=1)System.out.format(" too far PMT plane intersection \n");
                        //}
                    }else{
                        if(debugMode>=1)System.out.format(" no PMT plane intersection \n");
                    }
                }else{
                    test_intersection = get_Layer("mirror_front_B1").find_Entrance(lastray.asLine3D(), -1);
                    if(test_intersection==null)test_intersection = get_Layer("mirror_front_B2").find_Entrance(lastray.asLine3D(), -1);
                    if(test_intersection!=null){
                        if(debugMode>=1){
                            System.out.format(" test front (z %7.2f, step %7.2f) : ",last_ori.z(), test_intersection.get_pos().distance(last_ori));
                            test_intersection.showIntersection();
                        }
                        //if(test_intersection.get_pos().distance(last_ori)>RICHConstants.PHOTON_DISTMIN_TRACING)new_intersection = test_intersection; 
                        new_intersection = test_intersection;
                        front_nrefl++;
                    }else{
                        if(debugMode>=1)System.out.format(" no front mirror intersection \n");
                    }
                }

            }

            if(new_hit!=null){
                if(new_intersection==null || new_hit.distance(last_ori) <= new_intersection.get_pos().distance(last_ori)) {
                    detected=true;
                    if(debugMode>=1) System.out.format(" found PMT hit %s  dist %6.2f \n", new_hit.toStringBrief(2), new_hit.distance(last_ori));
                }
            }
            if(front_nrefl>1){
                lost = true; 
                new_hit=new_intersection.get_pos();
                if(debugMode>=1) System.out.format(" double front reflection: stop at front %s \n",toString(new_hit));
            }
            if(new_hit==null && new_intersection==null){
                lost = true; 
                Point3D point = new Point3D(0.0, 0.0, 0.0);;
                new_hit = new Point3D(lastray.end());
                Plane3D plane = rgeo.toTriangle3D(get_Layer(get_LayerNumber("mapmts")).get_Face(0)).plane();
                if(plane.intersection(lastray.asLine3D(), point)==1){ 
                    double vers = lastray.direction().costheta();
                    double Delta_z = point.z()-lastray.origin().z();
                    if(debugMode>=1) System.out.format(" forced stop at PMT plane: Delta_z %7.3f vers %7.3f \n",Delta_z, vers);
                    if(Delta_z*vers>0){
                        new_hit=point;
                        if(debugMode>=1) System.out.format(" take PMT plane hit %s \n", new_hit.toStringBrief(2));
                    }else{
                        if(debugMode>=1) System.out.format(" no Delta_z on PMT plane: take last ray end %s \n", new_hit.toStringBrief(2));
                    }
                }else{
                    if(debugMode>=1) System.out.format(" no hit on PMT plane: take last ray end %s \n", new_hit.toStringBrief(2));
                }
            }

            if(lost || detected){
                if(debugMode>=1 && lost) System.out.format("LOST! stop ray-tracing \n");
                if(debugMode>=1 && detected) System.out.format("DETECTED! stop ray-tracing \n");

                RICHRay newray = new RICHRay(last_ori, new_hit);
                newray.set_type(lastray.get_type());
                newray.set_refind((float) geocost.RICH_AIR_INDEX);
                if(detected)newray.set_detected();
                raytracks.add(newray);
                if(debugMode>=1){
                    System.out.format(" --> Add last ray (%7.4f) : ", geocost.RICH_AIR_INDEX);
                    newray.showRay();
                }

            }else{

                RICHRay newray = new RICHRay(last_ori, new_intersection.get_pos());
                newray.set_refind(new_intersection.get_nin());
                newray.set_type(lastray.get_type());
                raytracks.add(newray);

                // new ray starting at intersection, to be rotated
                rayin = new RICHRay(new_intersection.get_pos(), newray.direction().multiply(200));
                lastray = OpticalRotation(rayin, new_intersection);
                lastray.set_refind(new_intersection.get_nout());

                if(debugMode>=1){
                    System.out.format(" -->  Add new ray (%7.4f) : ",new_intersection.get_nin());
                    newray.showRay();
                    System.out.format(" -->  Get rotated ray (%7.4f) : ",new_intersection.get_nout());
                    lastray.showRay();
                }

            }
            jj++;

        }

        if(debugMode>=1) System.out.format(" --------------------------- \n");
        //if(detected==true)return raytracks;
        return raytracks;
        //return null;
   }


    // ----------------
    public void init_ProcessTime(){
    // ----------------

       int debugMode = 0;

       for(int i=0; i<NTIME; i++){richprocess_time[i] = 0.0; richprocess_ntimes[i]=0;}

       RICH_START_TIME = System.nanoTime();
       RICH_LAST_TIME = RICH_START_TIME;

       if(debugMode==1)System.out.format("RICH_START_TIME %d \n",RICH_START_TIME);
    }

    // ----------------
    public void save_ProcessTime(int iphase){
    // ----------------

        int debugMode = 0;

        if(iphase>-1 && iphase<NTIME){

            long nanotime = System.nanoTime()-RICH_START_TIME;
            double dtime = nanotime * 1.0e-6;

            richprocess_time[iphase] += dtime;
            richprocess_ntimes[iphase] += 1;

            if(debugMode==1)System.out.format("Phase %3d: Save time %3d  %10.4f \n", iphase, richprocess_ntimes[iphase], dtime);

        }

        double interval = (System.nanoTime()-RICH_LAST_TIME)*1e-9;   //seconds
        if(iphase==0 && interval > recpar.SHOW_PROGRESS_INTERVAL) {

            RICH_LAST_TIME = System.nanoTime();
            dump_ProcessTime();
        }
    }

    // ----------------
    public void dump_ProcessTime(){
    // ----------------

        String str[] = {" RAW-RICH " ," DC-RICH  ", " HADRONS  ", " ANALYTIC ", " TRACED   ", " WRITE    ", " CLOSE    "};

        for(int i=0; i<NTIME; i++){
            double time = 0.0;
            if(richprocess_ntimes[i]>0){
                int found=-1;
                for(int j=i-1; j>-1; j--){
                    if(richprocess_ntimes[j]>0){found=j; break;}
                }
                if(found>-1){
                    time = (richprocess_time[i]/richprocess_ntimes[i]-richprocess_time[found]/richprocess_ntimes[found]);
                }else{
                    time = richprocess_time[i]/richprocess_ntimes[i];
                }
                System.out.format(" PHASE %3d: %s  average over %6d  time %10.4f ms \n", i, str[i], richprocess_ntimes[i], time);
            }
        }

        for(int i=NTIME-1; i>-1; i--){
            double time = 0.0;
            if(richprocess_ntimes[i]>0){
                time = richprocess_time[i]/richprocess_ntimes[i];
                System.out.format(" PHASE %3d:  TOTAL      average over %6d  time %10.4f ms \n", NTIME, richprocess_ntimes[i], time);
                break;
            }
        }
    }

}
