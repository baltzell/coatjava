package org.jlab.rec.rich;

import org.jlab.detector.geom.RICH.RICHGeoConstants;
import org.jlab.detector.geom.RICH.RICHGeoFactory;
import org.jlab.detector.calib.utils.ConstantsManager;
import org.jlab.utils.groups.IndexedTable;

import org.jlab.geom.prim.Vector3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import java.io.FileReader;
import java.io.BufferedReader;

/**
 *
 * @author mcontalb
 */
public class RICHCalibration{

    private final static RICHGeoConstants geocost = new RICHGeoConstants();

    private final static int NSEC   =  geocost.NSEC;
    private final static int NLAY   =  geocost.NLAY;
    private final static int NPMT   =  geocost.NPMT;
    private final static int NPIX   =  geocost.NPIX;
    private final static int NCOMPO =  geocost.NCOMPO;
    private final static int NWALK  =  geocost.NWALK;

    private static IndexedTable richTable;
    private ArrayList<IndexedTable> timewalkTables = new ArrayList<IndexedTable>();
    private ArrayList<IndexedTable> timeoffTables  = new ArrayList<IndexedTable>();
    private ArrayList<IndexedTable> anglerefTables = new ArrayList<IndexedTable>();
    private ArrayList<IndexedTable> pixelTables    = new ArrayList<IndexedTable>();

    private double D0[] = new double[NPMT];
    private double T0[] = new double[NPMT];
    private double m1[] = new double[NPMT];
    private double m2[] = new double[NPMT];

    private RICHParameters richpar;

    // just for TxT file
    private double pmt_timeoff[][] = new double[NPMT][NPIX];
    private double pmt_timewalk[][] = new double[NPMT][4];

    private double aero_chele_dir[][][] = new double[4][31][225];
    private double aero_chele_lat[][][] = new double[4][31][225];
    private double aero_chele_spe[][][] = new double[4][31][225];
    private double aero_schele_dir[][][] = new double[4][31][225];
    private double aero_schele_lat[][][] = new double[4][31][225];
    private double aero_schele_spe[][][] = new double[4][31][225];


    //------------------------------
    public RICHCalibration() {
    //------------------------------
    }


    //------------------------------
    public void load_CCDB(ConstantsManager manager, int run, int ncalls, RICHGeoFactory richgeo, RICHParameters richpar){
    //------------------------------

        int debugMode = 0;

        this.richpar = richpar;
        this.richTable = richgeo.get_richTable();

        for(int irich=1; irich<=richgeo.nRICHes(); irich++){

            int isec = find_RICHSector(irich);
            if(isec==0) continue;

            if(irich==1){
                // first RICH module
                init_CalConstantsCCDB( manager.getConstants(run, "/calibration/rich/time_walk"),
                                       manager.getConstants(run, "/calibration/rich/time_offset"),
                                       manager.getConstants(run, "/calibration/rich/electro"),
                                       manager.getConstants(run, "/calibration/rich/pixels"), irich );
            }else{
                // second RICH module
                init_CalConstantsCCDB( manager.getConstants(run, "/calibration/rich/time_walk"),
                                       manager.getConstants(run, "/calibration/rich/time_offset"),
                                       manager.getConstants(run, "/calibration/rich/electro"),
                                       manager.getConstants(run, "/calibration/rich/pixels"), irich );
            }

            if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST) {
                System.out.format("------------------------------------------------------------- \n");
                System.out.format("RICH: Load calib constants from CCDB for RICH %4d  sector %4d  run %6d \n", irich, isec, run);
                System.out.format("------------------------------------------------------------- \n");

                dump_CalConstants(irich);
            }

            if(RICHConstants.TIMECAL_FROM_FILE==1 && ncalls==0){

                init_CalConstantsTxT(1, isec, ncalls);

                if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST) {
                        System.out.format("------------------------------------------------------------- \n");
                        System.out.format("RICH: Load TIME calib constants from local TxT file for RICH 4d  sector %4d  run %6d \n", irich, isec, run);
                        System.out.format("------------------------------------------------------------- \n");

                        dump_TimeConstants(irich);
                }
            }

            if(RICHConstants.AEROCAL_FROM_FILE==1){

                init_CalConstantsTxT(2, isec, ncalls);

                if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST) {
                        System.out.format("------------------------------------------------------------- \n");
                        System.out.format("RICH: Load AERO calib constants from local TxT file for RICH 4d  sector %4d  run %6d \n", irich, isec, run);
                        System.out.format("------------------------------------------------------------- \n");

                        dump_AeroConstants(irich);
                }
            }

            if(RICHConstants.PIXECAL_FROM_FILE==1){

                init_CalConstantsTxT(3, isec, ncalls);

                if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST) {
                        System.out.format("------------------------------------------------------------- \n");
                        System.out.format("RICH: Load PIXEL calib constants from local TxT file for RICH 4d  sector %4d  run %6d \n", irich, isec, run);
                        System.out.format("------------------------------------------------------------- \n");

                        dump_PixelConstants(irich);
                }
            }
        }

    }


    //------------------------------
    public void init_CalConstantsCCDB(IndexedTable timewalkConstants, IndexedTable timeoffConstants,  
                                      IndexedTable cheleConstants, IndexedTable pixelConstants, int irich){
    //------------------------------

        int debugMode = 0;

        if(irich==1){
          this.timeoffTables.add( timeoffConstants );
          this.timewalkTables.add( timewalkConstants );
          this.anglerefTables.add( cheleConstants );
          this.pixelTables.add( pixelConstants );
        }

    }


    //------------------------------
    public void init_CalConstantsTxT(int ifile, int isec, int ncalls){
    //------------------------------
    // To be moved to CCDB

        int debugMode = 0;

       if(ifile==1){
           /*
            * TIME_OFFSETs
            */
           /* String off_filename = new String("CALIB_DATA/MIRA/richTimeOffsets.out");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(off_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {

                    String[] array = currentLine.split(" ");
                    int ipmt = Integer.parseInt(array[0]);
                    int ich  = Integer.parseInt(array[1]);
                    float off = Float.parseFloat(array[4]);
                    pmt_timeoff[ipmt-1][ich-1] = off;

                    if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST)if(ich==1 || ich==64)
                              System.out.format("TXT RICH TOFF   pmt %4d (ich=%3d: %8.2f) \n", ipmt, ich, pmt_timeoff[ipmt-1][ich-1]);

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", off_filename);
                e.printStackTrace();

            }*/


            /*
            *  TIME_WALKs
            */
            String walk_filename = new String("calibration/rich/richModule1/time_walk.txt");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(walk_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {

                    if(currentLine.substring(0,1).equals("#")) continue;
                    if(currentLine.substring(0,1).equals(" ")) continue;

                    String[] array = currentLine.split(" ");
                    //int isec = Integer.parseInt(array[0]);
                    int ipmt = Integer.parseInt(array[1]);
                    if(ipmt<1 || ipmt>391) System.err.format("Exception occurred trying to pmt from '%s' \n", walk_filename);

                    D0[ipmt-1] = Double.parseDouble(array[3]);
                    T0[ipmt-1] = Double.parseDouble(array[4]);
                    m1[ipmt-1] = Double.parseDouble(array[5]);
                    m2[ipmt-1] = Double.parseDouble(array[6]);

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", walk_filename);
                e.printStackTrace();

            }
       }


        if(ifile==2){

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

                    if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST)System.out.format("Read chele for AERO lay %3d  compo %3d quadrant  %3d", idlay, iaer, iqua);

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

            if((debugMode>=1 || richpar.DEBUG_CALCOST>=1) && ncalls<richpar.DEBUG_CALCOST)System.out.format("initConstants: DONE \n");

        }

    }

    //------------------------------
    public void dump_CalConstants(int irich) { 
    //------------------------------


        dump_TimeConstants(irich);
        dump_AeroConstants(irich);
        dump_PixelConstants(irich);

    }


    //------------------------------
    public void dump_TimeConstants(int irich) { 
    //------------------------------

        int isec = find_RICHSector(irich);
        if(isec==0) return;

        for(int ipmt=1; ipmt<=NPMT; ipmt++){

            if(ipmt<=10 || ipmt>=382)System.out.format("CCDB RICH TOFF    ipmt %4d  %8.3f (ch1)  %8.3f (ch2)  %8.3f (ch63)  %8.3f (ch64) \n", ipmt,
               -get_PixelTimeOff(isec, ipmt, 1), -get_PixelTimeOff(isec, ipmt, 2), -get_PixelTimeOff(isec, ipmt, 63), -get_PixelTimeOff(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB RICH TOFF     ....... \n");
            if(ipmt==391)System.out.format("  \n");
        }

        for(int ipmt=1; ipmt<=NPMT; ipmt++){
            if(ipmt<=10 || ipmt>=382)System.out.format("CCDB RICH TWALK   ipmt %4d  D0 = %8.3f  T0 = %8.3f  m1 = %8.4f  m2 = %8.4f\n", ipmt,
                     timewalkTables.get(irich-1).getDoubleValue("D0", isec, ipmt, 0), timewalkTables.get(irich-1).getDoubleValue("m1", isec, ipmt, 0),
                     timewalkTables.get(irich-1).getDoubleValue("m2", isec, ipmt, 0), timewalkTables.get(irich-1).getDoubleValue("T0", isec, ipmt, 0));
                     

            if(ipmt==10)System.out.format("CCDB RICH TWALK    ....... \n");
            if(ipmt==391)System.out.format("  \n");
        }

    }


    //------------------------------
    public void dump_PixelConstants(int irich) { 
    //------------------------------

        int isec = find_RICHSector(irich);
        if(isec==0) return;

        for(int ipmt=1; ipmt<=NPMT; ipmt++){

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL GAIN    ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt,
               get_PixelGain(isec, ipmt, 1), get_PixelGain(isec, ipmt, 2), get_PixelGain(isec, ipmt, 63), get_PixelGain(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL GAIN     ....... \n");

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL EFF     ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt,
               get_PixelEff(isec, ipmt, 1), get_PixelEff(isec, ipmt, 2), get_PixelEff(isec, ipmt, 63), get_PixelEff(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL EFF      ....... \n");

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL STATUS  ipmt %4d  %8d (ch1)  %8d (ch2)  %8d (ch63)  %8d (ch64) \n", ipmt,
               get_PixelStatus(isec, ipmt, 1), get_PixelStatus(isec, ipmt, 2), get_PixelStatus(isec, ipmt, 63), get_PixelStatus(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL STATUS   ....... \n");

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL NTIME   ipmt %4d  %8d (ch1)  %8d (ch2)  %8d (ch63)  %8d (ch64) \n", ipmt,
               get_PixelNTime(isec, ipmt, 1), get_PixelNTime(isec, ipmt, 2), get_PixelNTime(isec, ipmt, 63), get_PixelNTime(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL NTIME    ....... \n");

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL MTIME   ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt,
               get_PixelMTime(isec, ipmt, 1), get_PixelMTime(isec, ipmt, 2), get_PixelMTime(isec, ipmt, 63), get_PixelMTime(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL MTIME    ....... \n");

            if(ipmt<=2 || ipmt>=390)System.out.format("CCDB PIXEL STIME   ipmt %4d  %8.2f (ch1)  %8.2f (ch2)  %8.2f (ch63)  %8.2f (ch64) \n", ipmt,
               get_PixelSTime(isec, ipmt, 1), get_PixelSTime(isec, ipmt, 2), get_PixelSTime(isec, ipmt, 63), get_PixelSTime(isec, ipmt, 64));
            if(ipmt==10)System.out.format("CCDB PIXEL STIME    ....... \n");

        }
        System.out.format("  \n");


        for(int ipmt=1; ipmt<=NPMT; ipmt++){
            for(int ich=1; ich<=NPIX; ich++){
                if(get_PixelStatus(isec, ipmt, ich)==2)
                    System.out.format("CCDB DEAD PIXEL  Sec %4d  PMT %4d  Anode %6d  Status %6d \n",isec, ipmt, ich, get_PixelStatus(isec, ipmt, ich));  
            }
        }

        System.out.format("  \n");
        for(int ipmt=1; ipmt<=NPMT; ipmt++){
            for(int ich=1; ich<=NPIX; ich++){
                if(get_PixelStatus(isec, ipmt, ich)==5)
                    System.out.format("CCDB HOT  PIXEL  Sec %4d  PMT %4d  Anode %6d  Status %6d \n",isec, ipmt, ich, get_PixelStatus(isec, ipmt, ich));  
            }
        }

    }



    //------------------------------
    public void dump_AeroConstants(int irich) { 
    //------------------------------

        double mrad = RICHConstants.MRAD;
        int isec = find_RICHSector(irich);
        if(isec==0) return;

        for (int ila=0; ila<4; ila++){
            for(int ico=0; ico<geocost.NAERCO[ila]; ico++){
                if(ico<2 || ico>geocost.NAERCO[ila]-3){
                    for (int iqua=0; iqua<225; iqua+=224){
                    
                        System.out.format("CCDB RICH CHELE   ila %4d  itile %3d  iq %4d dir = %7.2f  %7.2f  lat = %7.2f  %7.2f  spe = %7.2f  %7.2f \n", 
                        201+ila, ico+1, iqua+1,
                        get_ChElectron(isec, ila, ico, iqua, 0)*mrad, get_SChElectron(isec, ila, ico, iqua, 0)*mrad,
                        get_ChElectron(isec, ila, ico, iqua, 1)*mrad, get_SChElectron(isec, ila, ico, iqua, 1)*mrad,
                        get_ChElectron(isec, ila, ico, iqua, 2)*mrad, get_SChElectron(isec, ila, ico, iqua, 2)*mrad);
                    }
                }
            }
        }
        System.out.format("  \n");
    }



    //------------------------------
    public double get_SChElectron(int isec, int ila, int ico, int iqua, int irefle) {
    //------------------------------


        if(ico<0 || ico>=geocost.NAERCO[ila]) return 0.0;
        int irich = find_RICHModule(isec);
        if(irich==0) return 0.0;
 
        int irow = ico*225+iqua+1;
   
        if(irefle==0){
            if(anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("s_lat", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("s_lat", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("s_spe", isec, 201+ila, irow);
                }
            }
        }
        if(irefle==1){
            if(anglerefTables.get(irich-1).getDoubleValue("s_lat", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("s_lat", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("s_spe", isec, 201+ila, irow);
                }
            }
        }
        if(irefle==2){
            if(anglerefTables.get(irich-1).getDoubleValue("s_spe", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("s_spe", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("s_dir", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("s_lat", isec, 201+ila, irow);
                }
            }
        }
        return 0.0;
    }


    //------------------------------
    public double get_PixelGain(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
        return pixelTables.get(irich-1).getDoubleValue("gain", isec, ipmt, ich);
    }


    //------------------------------
    public double get_PixelEff(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
        return pixelTables.get(irich-1).getDoubleValue("efficiency", isec, ipmt, ich);
    }


    //------------------------------
    public int get_PixelStatus(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0;
        return pixelTables.get(irich-1).getIntValue("status", isec, ipmt, ich);
    }


    //------------------------------
    public double get_PixelMTime(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
        return pixelTables.get(irich-1).getDoubleValue("mean_t", isec, ipmt, ich);
    }


    //------------------------------
    public double get_PixelSTime(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
        return pixelTables.get(irich-1).getDoubleValue("sigma_t", isec, ipmt, ich);
    }


    //------------------------------
    public int get_PixelNTime(int isec, int ipmt, int ich) { 
    //------------------------------
        int irich = find_RICHModule(isec);
        if(irich==0)return 0;
        return pixelTables.get(irich-1).getIntValue("N_t", isec, ipmt, ich);
    }


    //------------------------------
    public double get_ChElectron(int isec, int ila, int ico, int iqua, int irefle) {
    //------------------------------


        if(ico<0 || ico>=geocost.NAERCO[ila]) return 0.0;
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
 
        int irow = ico*225+iqua+1;
   
        if(irefle==0){
            if(anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("ch_lat", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("ch_lat", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("ch_spe", isec, 201+ila, irow);
                }
            }
        }
        if(irefle==1){
            if(anglerefTables.get(irich-1).getDoubleValue("ch_lat", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("ch_lat", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("ch_spe", isec, 201+ila, irow);
                }
            }
        }
        if(irefle==2){
            if(anglerefTables.get(irich-1).getDoubleValue("ch_spe", isec, 201+ila, irow)>0){
                return anglerefTables.get(irich-1).getDoubleValue("ch_spe", isec, 201+ila, irow);
            }else{
                if(anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow)>0){
                    return anglerefTables.get(irich-1).getDoubleValue("ch_dir", isec, 201+ila, irow);
                }else{
                    return anglerefTables.get(irich-1).getDoubleValue("ch_lat", isec, 201+ila, irow);
                }
            }
        }
        return 0.0;
    }


    //------------------------------
    public double get_PixelTimeOff(int isec, int ipmt, int ich){
    //------------------------------
 
        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;

        return -1*timeoffTables.get(irich-1).getDoubleValue("offset", isec, ipmt, ich);

    }


    //------------------------------
    public double get_PixelTimeWalk(int isec, int ipmt, int duration){
    //------------------------------

        int irich = find_RICHModule(isec);
        if(irich==0)return 0.0;
 
        double twalk_corr = 0;
        double D0 = timewalkTables.get(irich-1).getDoubleValue("D0", isec, ipmt, 0);
        double T0 = timewalkTables.get(irich-1).getDoubleValue("m1", isec, ipmt, 0);
        double m1 = timewalkTables.get(irich-1).getDoubleValue("m2", isec, ipmt, 0);
        double m2 = timewalkTables.get(irich-1).getDoubleValue("T0", isec, ipmt, 0);

        double f1 = m1 * duration + T0;
        double f1T = m1 * D0 + T0;

        double f2 = m2 * (duration - D0) + f1T;
        twalk_corr = f1;
        if (duration > D0) twalk_corr = f2;

        return twalk_corr;
    }


    //------------------------------
    public int find_RICHModule(int isec){
    //------------------------------

        if( richTable.hasEntry(isec,0,0)){
            return richTable.getIntValue("module", isec, 0, 0);
        }
        return 0;
    }


    //------------------------------
    public int find_RICHSector(int irich){
    //------------------------------
        int debugMode = 0;

        for (int isec=1; isec<=RICHGeoConstants.NSEC; isec++){
            if(richTable.hasEntry(isec,0,0)){
                if(debugMode>=1)System.out.format(" trovo %4d <--> %4d \n",irich, richTable.getIntValue("module", isec, 0, 0));
                if(richTable.getIntValue("module", isec, 0, 0) == irich)  return isec;
            }
        }
        return 0;
    }

}
