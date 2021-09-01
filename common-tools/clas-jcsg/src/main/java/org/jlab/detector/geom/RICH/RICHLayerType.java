package org.jlab.detector.geom.RICH;

/*
 * @author mcontalb
 */
public enum RICHLayerType {

    AEROGEL_2CM_B1        (  0, "AER_B1", 201, 1, "front"  ),
    AEROGEL_2CM_B2        (  1, "AER_B2", 202, 1, "front"  ),
    AEROGEL_3CM_L1        (  2, "AER_L1", 203, 2, "front"  ),
    AEROGEL_3CM_L2        (  3, "AER_L2", 204, 2, "front"  ),
    MIRROR_FRONT_B1       (  4, "MIR_B1", 301, 3, "front"  ),
    MIRROR_FRONT_B2       (  5, "MIR_B2", 301, 3, "front"  ),
    MIRROR_LEFT_L1        (  6, "MIR_L1", 301, 4, "left"   ),
    MIRROR_LEFT_L2        (  7, "MIR_L2", 301, 4, "left"   ),
    MIRROR_RIGHT_R1       (  8, "MIR_R1", 301, 4, "right"  ),
    MIRROR_RIGHT_R2       (  9, "MIR_R2", 301, 4, "right"  ),
    MIRROR_BOTTOM         ( 10, "MIR_BT", 301, 4, "bottom" ),
    MIRROR_SPHERE         ( 11, "SPHER" , 302, 5, "sphere" ),
    MAPMT                 ( 12, "MAPMT" , 401, 6, "back"   ),
    UNDEFINED             ( 13, "UNDEF" ,   0, 0, "null"   );

    private final int     id;
    private final String  label;
    private final int     ccdb;
    private final int     type;
    private final String  vers;


    //------------------------------
    RICHLayerType(){
    //------------------------------
        id   = 0;
        ccdb = 0;
        type = 1;
        label = "UNDEFINED";
        vers = "front";
    }


    //------------------------------
    RICHLayerType(int id, String label, int ccdb, int type, String vers){
    //------------------------------
        this.id   = id;
        this.ccdb = ccdb;
        this.type = type;
        this.label = label;
        this.vers = vers;
    }


    //------------------------------
    public String label() { return label; }
    //------------------------------

    //------------------------------
    public int id() { return id; }
    //------------------------------

    //------------------------------
    public String vers() { return vers; }
    //------------------------------

    //------------------------------
    public int type() { return type; }
    //------------------------------

    //------------------------------
    public int ccdb() { return ccdb; }
    //------------------------------


    //------------------------------
    public static RICHLayerType get_Type(String label) {
    //------------------------------
        label = label.trim();
        for(RICHLayerType lay: RICHLayerType.values())
            if (lay.label().equalsIgnoreCase(label))
                return lay;
        return UNDEFINED;
    }


    //------------------------------
    public static RICHLayerType get_Type(int id) {
    //------------------------------

        for(RICHLayerType lay: RICHLayerType.values())
            if (lay.id() == id)
                return lay;
        return UNDEFINED;
    }

}

