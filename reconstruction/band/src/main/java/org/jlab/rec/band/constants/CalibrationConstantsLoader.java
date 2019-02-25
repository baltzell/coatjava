package org.jlab.rec.band.constants;

import org.jlab.detector.calib.utils.DatabaseConstantProvider;

import java.util.HashMap;
import java.util.Map;
import java.lang.Integer;
import java.lang.Double;

/**
 * 
 * @author Efrain Segarra
 * This class loads constants from CCDB
 */

public class CalibrationConstantsLoader {

	public CalibrationConstantsLoader() {
		// TODO Auto-generated constructor stub
	}
	public static boolean CSTLOADED = false;

		// Maps for constants from database
	public static Map<Integer, Double> TDC_T_OFFSET = new HashMap<Integer, Double>();
	public static Map<Integer, Double> FADC_T_OFFSET = new HashMap<Integer, Double>();
	public static Map<Integer, Double> TDC_VEFF = new HashMap<Integer,Double>();
	public static Map<Integer, Double> FADC_VEFF = new HashMap<Integer, Double>();
	public static Map<Integer, Double> FADC_MT_P2P_OFFSET = new HashMap<Integer,Double>();
	public static Map<Integer, Double> FADC_MT_P2P_RES = new HashMap<Integer,Double>();
	public static Map<Integer, Double> FADC_MT_L2L_OFFSET = new HashMap<Integer,Double>();
	public static Map<Integer, Double> FADC_MT_L2L_RES = new HashMap<Integer,Double>();
	public static double JITTER_PERIOD = 0;
	public static int JITTER_PHASE = 0;
	public static int JITTER_CYCLES = 0;


	static DatabaseConstantProvider dbprovider = null;

	public static synchronized void Load(int runno, String var) {

		System.out.println("*Loading calibration constants*");

		dbprovider = new DatabaseConstantProvider(runno, var); // reset using the new variation

		// load table reads entire table and makes an array of variables for each column in the table.
		dbprovider.loadTable("/calibration/band/lr_offsets");
		dbprovider.loadTable("/calibration/band/effective_velocity");
		dbprovider.loadTable("/calibration/band/paddle_offsets");
		dbprovider.loadTable("/calibration/band/layer_offsets");

		//disconncect from database. Important to do this after loading tables.
		dbprovider.disconnect(); 

		dbprovider.show();


		// Time offsets
		for(int i =0; i< dbprovider.length("/calibration/band/lr_offsets/sector"); i++) {
				// Get sector, layer, component
			int sector 		= dbprovider.getInteger("/calibration/band/lr_offsets/sector", 		i);	    
			int layer 		= dbprovider.getInteger("/calibration/band/lr_offsets/layer", 		i);
			int component 	= dbprovider.getInteger("/calibration/band/lr_offsets/component", 	i);
				// Get the actual offsets
			double tdc_off 	= dbprovider.getDouble("/calibration/band/lr_offsets/tdc_off", 		i);
			double fadc_off = dbprovider.getDouble("/calibration/band/lr_offsets/fadc_off", 	i);
				// Put in the maps
			int key = sector*100+layer*10+component;
			TDC_T_OFFSET.put( 	Integer.valueOf(key), 	Double.valueOf(tdc_off) );
			FADC_T_OFFSET.put( 	Integer.valueOf(key), 	Double.valueOf(fadc_off) );
		}

		// Speed of lights
		for(int i =0; i< dbprovider.length("/calibration/band/effective_velocity/sector"); i++) {
				// Get sector, layer, component
			int sector 		= dbprovider.getInteger("/calibration/band/effective_velocity/sector",		i);	    
			int layer 		= dbprovider.getInteger("/calibration/band/effective_velocity/layer",		i);
			int component 	= dbprovider.getInteger("/calibration/band/effective_velocity/component",	i);
				// Get the velocities
			double veff_tdc		= dbprovider.getDouble("/calibration/band/effective_velocity/veff_tdc", 	i);
			double veff_fadc	= dbprovider.getDouble("/calibration/band/effective_velocity/veff_fadc",	i);
				// Put in the maps
			int key = sector*100+layer*10+component;
			TDC_VEFF.put(	Integer.valueOf(key),		Double.valueOf(veff_tdc) );
			FADC_VEFF.put(	Integer.valueOf(key), 		Double.valueOf(veff_fadc) );
		}
		
		// TDC time jitter
		JITTER_PERIOD = dbprovider.getDouble("/calibration/band/time_jitter/period", 0);
		JITTER_PHASE  = dbprovider.getInteger("/calibration/band/time_jitter/phase", 0);
		JITTER_CYCLES = dbprovider.getInteger("/calibration/band/time_jitter/cycles", 0);

		// Paddle-to-paddle offsets
		for(int i =0; i< dbprovider.length("/calibration/band/paddle_offsets"); i++) {
			int sector 		= dbprovider.getInteger("/calibration/band/paddle_offsets/sector",		i);	    
			int layer 		= dbprovider.getInteger("/calibration/band/paddle_offsets/layer",		i);
			int component 	= dbprovider.getInteger("/calibration/band/paddle_offsets/component",	i);
			
			// Get offset and resolution for FADC
			double p2p_off_fadc	= dbprovider.getDouble("/calibration/band/paddle_offsets/offset_fadc", 	i);
			double p2p_res_fadc	= dbprovider.getDouble("/calibration/band/paddle_offsets/resolution_fadc",	i);
			
			// Put in maps
			int key = sector*100+layer*10+component;
			FADC_MT_P2P_OFFSET.put(	Integer.valueOf(key),		Double.valueOf(p2p_off_fadc) );
			FADC_MT_P2P_RES.put(	Integer.valueOf(key),		Double.valueOf(p2p_res_fadc) );
		}
		// Layer-to-layer offsets
		for(int i =0; i< dbprovider.length("/calibration/band/layer_offsets"); i++) {
			int sector 		= dbprovider.getInteger("/calibration/band/layer_offsets/sector",		i);	    
			int layer 		= dbprovider.getInteger("/calibration/band/layer_offsets/layer",		i);
			int component 	= dbprovider.getInteger("/calibration/band/layer_offsets/component",	i);
			
			// Get offset and resolution for FADC
			double l2l_off_fadc	= dbprovider.getDouble("/calibration/band/layer_offsets/offset_fadc", 	i);
			double l2l_res_fadc	= dbprovider.getDouble("/calibration/band/layer_offsets/resolution_fadc",	i);
			
			// Put in maps
			int key = sector*100+layer*10+component;
			FADC_MT_L2L_OFFSET.put(	Integer.valueOf(key),		Double.valueOf(l2l_off_fadc) );
			FADC_MT_L2L_RES.put(	Integer.valueOf(key),		Double.valueOf(l2l_res_fadc) );
		}
		
		
		CSTLOADED = true;
		System.out.println("SUCCESSFULLY LOADED band CALIBRATION CONSTANTS....");

		setDB(dbprovider);

	} 


	private static DatabaseConstantProvider DB;

	public static final DatabaseConstantProvider getDB() {
		return DB;
	}



	public static final void setDB(DatabaseConstantProvider dB) {
		DB = dB;
	}

	public static void main (String arg[]) {
		CalibrationConstantsLoader.Load(10,"default");
	}
}
