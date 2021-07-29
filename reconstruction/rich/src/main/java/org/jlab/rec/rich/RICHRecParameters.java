/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.jlab.rec.rich;

/**
 *
 * @author mcontalb
 */
public class RICHRecParameters{

    // -----------------
    public RICHRecParameters() {
    // -----------------
    }

    // Default values of RICH Reconstruction parameters to be re-loaded from CCDB or TxT
    
    public int     FORCE_DC_MATCH                         =   0;        // if 1 force the hadron track to hit the cluster

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
    public double  RICH_REC_DEBUG                         =   0.0;      // Flag to activate the printout for debug

    public static final double  RICH_BKG_PROBABILITY      =   1.e-5;    // Background probability for likelihood

}
