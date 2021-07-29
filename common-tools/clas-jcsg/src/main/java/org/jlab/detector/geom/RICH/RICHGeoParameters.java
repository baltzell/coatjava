package org.jlab.detector.geom.RICH;

public class RICHGeoParameters {

    // Default values of RICH Reconstruction parameters to be re-loaded from CCDB or TxT

    public int     RICH_GEO_DEBUG                         =   0;        // if 1 do debug 
    public int     DO_ALIGNMENT                           =   1;        // if 1 apply alignment
    public int     FORCE_DC_MATCH                         =   0;        // if 1 force the hadron track to hit the cluster
    public int     MISA_RICH_REF                          =   1;        // if 1 use local RICH frame (instead of Lab frame)
    public int     MISA_PMT_PIVOT                         =   1;        // if 1 use MAPMT barycenter for rotations
    public int     APPLY_SURVEY                           =   0;        // if 1 apply the survey data for misalignment

    public double  MISA_SHIFT_SCALE                       =   1.0;      // Scale factor for misalignment shifts
    public double  MISA_ANGLE_SCALE                       =   1.0;      // Scale factor for misalignment angles
    
    public int     READ_FROM_FILES                        =   0;        // if 1 read geometry parameters from file

    // ----------------
    public RICHGeoParameters() {
    // ----------------
    }
}
