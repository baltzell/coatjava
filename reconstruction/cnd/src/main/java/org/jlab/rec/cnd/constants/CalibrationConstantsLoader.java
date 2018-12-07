package org.jlab.rec.cnd.constants;

import java.util.List;
import org.jlab.utils.groups.IndexedTable;



/**
 * 
 * @author ziegler
 *
 */
public class CalibrationConstantsLoader {

	public CalibrationConstantsLoader() {
		// TODO Auto-generated constructor stub
	}
	public static boolean CSTLOADED = false;

	// Instantiating the constants arrays
	public static double[][] UTURNELOSS 		= new double[24][3];
	public static double[][] UTURNTLOSS 		= new double[24][3];
	public static double[][] TIMEOFFSETSLR 		= new double[24][3];	
	public static double[][][] TDCTOTIMESLOPE	= new double[24][3][2];
	public static double[][][] TDCTOTIMEOFFSET	= new double[24][3][2];	
	public static double[][] TIMEOFFSETSECT 	= new double[24][3];
	public static double[][][] EFFVEL 		= new double[24][3][2];
	public static double[][][] ATNLEN               = new double[24][3][2];
	public static double[][][] MIPDIRECT 		= new double[24][3][2];
	public static double[][][] MIPINDIRECT		= new double[24][3][2];
	public static int[][][] Status_LR 		= new int[24][3][2];
	public static double JITTER_PERIOD              = 0;
	public static int JITTER_PHASE                  = 0;
	public static int JITTER_CYCLES                 = 0;
	public static double[] LENGTH                   = new double[3];
	public static double[] ZOFFSET                  = new double[3];
	public static double[] THICKNESS                = new double[1];
	public static double[] INNERRADIUS              = new double[1];
	//Calibration and geometry parameters from DB    

	public static boolean arEnergyibConstantsLoaded = false;
	public static synchronized void Load(List<IndexedTable> tabJs) {
            if(CSTLOADED)
                return;
            System.out.println(" LOADING CONSTANTS ");

            // load table reads entire table and makes an array of variables for each column in the table.
            //tabJs(0): ("/calibration/cnd/UturnEloss");
            //tabJs(1): ("/calibration/cnd/UturnTloss");
            //tabJs(2): ("/calibration/cnd/TimeOffsets_LR");
            //tabJs(3): ("/calibration/cnd/TDC_conv");
            //tabJs(4): ("/calibration/cnd/TimeOffsets_layer");
            //tabJs(5): ("/calibration/cnd/EffV");
            //tabJs(6): ("/calibration/cnd/Attenuation");
            //tabJs(7): ("/calibration/cnd/Status_LR");
            //tabJs(8): ("/calibration/cnd/Energy");
            //tabJs(9): ("/calibration/cnd/time_jitter");
            //tabJs(10): ("/geometry/cnd/layer");
            //tabJs(11): ("/geometry/cnd/cnd");

            for (int iSec = 1; iSec <=24; iSec++) {
                for(int iLay = 1; iLay <=3; iLay++) {
                    UTURNELOSS[iSec-1][iLay-1] = tabJs.get(0).getDoubleValue("uturn_eloss", iSec, iLay, 0);
                    UTURNTLOSS[iSec-1][iLay-1] = tabJs.get(1).getDoubleValue("uturn_tloss", iSec, iLay, 0);
                    TIMEOFFSETSLR[iSec-1][iLay-1] = tabJs.get(2).getDoubleValue("time_offset_LR", iSec, iLay, 0);
                    TDCTOTIMESLOPE[iSec-1][iLay-1][0]  = tabJs.get(3).getDoubleValue("slope_L", iSec, iLay, 0);
                    TDCTOTIMEOFFSET[iSec-1][iLay-1][0] = tabJs.get(3).getDoubleValue("offset_L", iSec, iLay, 0);
                    TDCTOTIMESLOPE[iSec-1][iLay-1][1]  = tabJs.get(3).getDoubleValue("slope_R", iSec, iLay, 0);
                    TDCTOTIMEOFFSET[iSec-1][iLay-1][1] = tabJs.get(3).getDoubleValue("offset_R", iSec, iLay, 0);
                    TIMEOFFSETSECT[iSec-1][iLay-1] = tabJs.get(4).getDoubleValue("time_offset_layer", iSec, iLay, 0);
                    EFFVEL[iSec-1][iLay-1][0] = tabJs.get(5).getDoubleValue("veff_L", iSec, iLay, 0);
                    EFFVEL[iSec-1][iLay-1][1] = tabJs.get(5).getDoubleValue("veff_R", iSec, iLay, 0);
                    ATNLEN[iSec-1][iLay-1][0] = tabJs.get(6).getDoubleValue("attlen_L", iSec, iLay, 0);
                    ATNLEN[iSec-1][iLay-1][1] = tabJs.get(6).getDoubleValue("attlen_R", iSec, iLay, 0);
                    Status_LR[iSec-1][iLay-1][0] = tabJs.get(7).getIntValue("/calibration/cnd/Status_LR/status_L", iSec, iLay, 0);
                    Status_LR[iSec-1][iLay-1][1] = tabJs.get(7).getIntValue("/calibration/cnd/Status_LR/status_R", iSec, iLay, 0);
                    MIPDIRECT[iSec-1][iLay-1][0]    = tabJs.get(8).getDoubleValue("/calibration/cnd/Energy/mip_dir_L", iSec, iLay, 0);
                    MIPINDIRECT[iSec-1][iLay-1][0]  = tabJs.get(8).getDoubleValue("/calibration/cnd/Energy/mip_indir_L", iSec, iLay, 0);
                    MIPDIRECT[iSec-1][iLay-1][1]    = tabJs.get(8).getDoubleValue("/calibration/cnd/Energy/mip_dir_R", iSec, iLay, 0);
                    MIPINDIRECT[iSec-1][iLay-1][1]  = tabJs.get(8).getDoubleValue("/calibration/cnd/Energy/mip_indir_R", iSec, iLay, 0);

                }
            }
            JITTER_PERIOD = tabJs.get(9).getDoubleValue("period", 0, 0, 0);
            JITTER_PHASE  = tabJs.get(9).getIntValue("phase",  0, 0, 0);
            JITTER_CYCLES = tabJs.get(9).getIntValue("cycles", 0, 0, 0);

            //for(int iLay = 1; iLay <=3; iLay++) {
            //    LENGTH[iLay-1]  = tabJs.get(10).getDoubleValue("Length", 0, iLay, 0);
            //    ZOFFSET[iLay-1] = tabJs.get(10).getDoubleValue("UpstreamZOffset", 0, iLay, 0);// not right structure for common tools
            //}
            LENGTH = new double[]{665.72, 700.0, 734.28};
            //INNERRADIUS[0] = tabJs.get(11).getDoubleValue("InnerRadius", 0, 0, 0);            
            //THICKNESS[0] = tabJs.get(11).getDoubleValue("Thickness", 0, 0, 0);  // not right structure for common tools
            INNERRADIUS[0] = 290.0;            
            THICKNESS[0] = 30.0;  
//
            CSTLOADED = true;
            System.out.println("SUCCESSFULLY LOADED CND CALIBRATION CONSTANTS....");

	} 


}
