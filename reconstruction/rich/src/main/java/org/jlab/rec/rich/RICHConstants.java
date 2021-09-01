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
public class RICHConstants {

    // RICH Reconstruction static constants

    // -----------------
    // Static
    // -----------------

    public static final int    NPMT                       =   391;      // number of PMTs in RICH
    public static final int    NPIX                       =   64;       // number of Pixels in one PMT

    public static final int     EVENT_TIME                =   300;      // Expected Event time

    public static final int     LEADING_EDGE_POLARITY     =   1;        // MAROC polarity of the leading edge
    public static final int     TRAILING_EDGE_POLARITY    =   0;        // MAROC polarity of the trailing edge

    //public static final int     CLUSTER_TIME_WINDOW     =   20;       // Cluster acceptance time window
    public static final int     CLUSTER_TIME_WINDOW       =   500000;   // initial test with data
    public static final int     CLUSTER_MIN_SIZE          =   3;        // Cluster acceptance min size
    public static final int     CLUSTER_MIN_CHARGE        =   0;        // Cluster acceptance min energy

    public static final double  COSMIC_TRACKING_Z         =  -120.;     // Tracking station height in cosmic run
    public static final double  COSMIC_AEROGEL_Z          =  -100.;     // Aerogel  station height in cosmic run

    public static final double PHOTON_DISTMIN_TRACING     =   0.0001;   // max distance to set initial values for tracing photons
    public static final double PHOTON_DISTMIN_SPHERE      =   200.;     // max distance to approximate the spherical mirror with triangles
    public static final double RICH_MIN_CHANGLE           =   10.e-3;   // rad

    public static final double  RICH_MATCH_POLYDIST       =   1.e-3;    // Matching dist between poly and point

    public static final int    RECOPAR_FROM_FILE          =   0;        // read reconstruction parameters from local txt files
    public static final int    TIMECAL_FROM_FILE          =   0;        // read time calibation values from local txt files
    public static final int    AEROCAL_FROM_FILE          =   0;        // read aerogel calibration values from local txt files
    public static final int    PIXECAL_FROM_FILE          =   0;        // read pixel calibration values from local txt files

    // -----------------
    // Run Type
    // -----------------

    public static final int     CLAS_RUN                  = 1;       // 1 if clas runs
    public static final int     GEMC_RUN                  = 0;       // 1 if clas runs
    public static final int     COSMIC_RUN                = 0;       // 1 if cosmic runs

    // -----------------
    // Generic
    // -----------------

    public static final double MRAD = 1000.;
    public static final double RAD = 180./Math.PI;

}
