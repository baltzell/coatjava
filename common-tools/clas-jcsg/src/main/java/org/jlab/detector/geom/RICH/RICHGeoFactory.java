package org.jlab.detector.geom.RICH;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.io.BufferedReader;
import java.io.FileReader;

import org.jlab.detector.geant4.v2.RICHGeant4Factory;
import org.jlab.detector.volume.G4Stl;
import org.jlab.detector.volume.G4Box;

import eu.mihosoft.vrl.v3d.Vector3d;
import eu.mihosoft.vrl.v3d.Vertex;
import eu.mihosoft.vrl.v3d.Polygon;   
import eu.mihosoft.vrl.v3d.CSG;   
import org.jlab.detector.base.DetectorLayer;
import org.jlab.detector.calib.utils.ConstantsManager;
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
public class RICHGeoFactory{

    private RICHGeant4Factory richfactory = new RICHGeant4Factory();
    private RICHPixelMap pixelmap = new RICHPixelMap();
    private RICHPixel pmtpixels  = null; 
    private RICHGeoParameters geopar = new RICHGeoParameters();

    private List<RICHLayer> opticlayers = new ArrayList<RICHLayer>();
    private final static int NLAY   = RICHGeoConstants.NLAY;
    private final static int NCOMPO = RICHGeoConstants.NCOMPO;

    private double aero_refi[][] = new double[4][31];
    private double aero_plan[][] = new double[4][31];

    private Vector3D rich_survey_angle  = new Vector3D();
    private Vector3D rich_survey_shift  = new Vector3D();
    private Vector3D layer_misa_angle[][] = new Vector3D[NLAY+1][NCOMPO+1];
    private Vector3D layer_misa_shift[][] = new Vector3D[NLAY+1][NCOMPO+1];
    private RICHFrame rich_frame = new RICHFrame();
    private RICHFrame survey_frame = new RICHFrame();

    //------------------------------
    public RICHGeoFactory() {
    //------------------------------
    }


    //------------------------------
    public RICHGeoFactory(int iEngine, ConstantsManager manager, int run){
    //------------------------------

        //PROVA
        // generate the tracking layers (iEngine=0 only Aerogel and MaPMT for trajectory, iEngine=1  all)

        if(iEngine==0){
            // add RICH tables to a different Engine
            String[] richTables = new String[]{
                    "/calibration/rich/aerogel",
                    "/calibration/rich/misalignments",
                    "/calibration/rich/parameterss"
                 };
            manager.init(Arrays.asList(richTables));
        }

        // reset alignment constants
        for (int ila=0; ila<NLAY+1; ila++){
            for (int ico=0; ico<NCOMPO+1; ico++){
                layer_misa_shift[ila][ico] = new Vector3D(0., 0., 0.);
                layer_misa_angle[ila][ico] = new Vector3D(0., 0., 0.);
            }
        }

        System.out.format("RICHGeoFactory: Load geometry constants from CCDB \n");
     
        init_GeoParametersCCDB( manager.getConstants(run, "/calibration/rich/parameterss"),
                                manager.getConstants(run, "/calibration/rich/aerogel"), 
                                manager.getConstants(run, "/calibration/rich/misalignments"));

        if(iEngine==1 && geopar.READ_FROM_FILES==1){
            // QUE attenzione per la produzione
            System.out.format("RICHGeoFactory: Load geometry constants from TXT \n");
            init_GeoParametersTxT(1);
            //init_GeoParametersTxT(3);
        }
       
        if(iEngine>0){
            // global pixel coordinat indexes
            pixelmap.init_GlobalPixelGeo();

            // RICH survey
            init_Survey();
        }

        // RICH geometry organized on layers of Shape3D area and RICH components 
        init_RICHLayers(iEngine);

    } 


    //------------------------------
    public RICHGeoParameters get_GeoParameters() {return geopar;}
    //------------------------------


    //------------------------------
    public void init_GeoParametersCCDB(IndexedTable paraConstants, IndexedTable aeroConstants, IndexedTable misaConstants){
    //------------------------------

        int debugMode = 1;

        /*
        * RECONSTRUCTION PARAMETERS
        */

        geopar.DO_ALIGNMENT                =  paraConstants.getIntValue("flag1", 4, 0, 0); 
        geopar.MISA_RICH_REF               =  paraConstants.getIntValue("flag3", 4, 0, 0);
        geopar.MISA_PMT_PIVOT              =  paraConstants.getIntValue("flag4", 4, 0, 0);
        geopar.APPLY_SURVEY                =  paraConstants.getIntValue("flag5", 4, 0, 0);

        geopar.MISA_SHIFT_SCALE            =  paraConstants.getDoubleValue("par9", 4, 0, 0);
        geopar.MISA_ANGLE_SCALE            =  paraConstants.getDoubleValue("par10", 4, 0, 0);
        
        geopar.RICH_GEO_DEBUG              =  (int) paraConstants.getDoubleValue("par7", 4, 0, 0);

        // QUE: da rimuovere nella produzione
        geopar.READ_FROM_FILES             =  1;

        if(debugMode>=1 || geopar.RICH_GEO_DEBUG>0){   

            System.out.format(" \n");
            System.out.format("CCDB RICH PARA    DO_ALIGNMENT                 %9d \n", geopar.DO_ALIGNMENT); 
            System.out.format("CCDB RICH PARA    MISA_RICH_REF                %9d \n", geopar.MISA_RICH_REF); 
            System.out.format("CCDB RICH PARA    MISA_PMT_PIVOT               %9d \n", geopar.MISA_PMT_PIVOT); 
            System.out.format("CCDB RICH PARA    APPLY_SURVEY                 %9d \n", geopar.APPLY_SURVEY); 

            System.out.format("CCDB RICH PARA    MISA_SHIFT_SCALE             %9.4f \n", geopar.MISA_SHIFT_SCALE); 
            System.out.format("CCDB RICH PARA    MISA_ANGLE_SCALE             %9.4f \n", geopar.MISA_ANGLE_SCALE); 

            System.out.format("CCDB RICH PARA    RICH_GEO_DEBUG               %9d \n", geopar.RICH_GEO_DEBUG); 
            System.out.format(" \n");

        }


        /*
        *  SINGLE COMPONENT MISALIGNMENT
        *  This comes on top of the RICH survey and global transformation
        */

        int NMISA = 24;
        int ccdb_ila[] = {0,201,202,203,204,301,301,301,301,301,301,301,302,302,302,302,302,302,302,302,302,302,302,401};
        int ccdb_ico[] = {0,  0,  0,  0,  0,  1,  2,  3,  4,  5,  6,  7,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,  0};

        //                                   BO  F1  F2  R1  R2   L1  L2   
        int tool_ila[] = {0,  1,  2,  3,  4, 11,  5,  6,  9, 10,  7,  8, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13};
        int tool_ico[] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,  0};

        double sscale = geopar.MISA_SHIFT_SCALE;
        double ascale = geopar.MISA_ANGLE_SCALE / RICHGeoConstants.MRAD;  // to convert in rad

        for (int im=0; im<NMISA; im++){

            int lla = ccdb_ila[im];
            int cco = ccdb_ico[im];
            double dx = (double) misaConstants.getDoubleValue("dx", 4, lla, cco);
            double dy = (double) misaConstants.getDoubleValue("dy", 4, lla, cco);
            double dz = (double) misaConstants.getDoubleValue("dz", 4, lla, cco);
            double thx = (double) misaConstants.getDoubleValue("dthx", 4, lla, cco);
            double thy = (double) misaConstants.getDoubleValue("dthy", 4, lla, cco);
            double thz = (double) misaConstants.getDoubleValue("dthz", 4, lla, cco);

            // the rotation is assumed to be in the component local ref system
            int ila = tool_ila[im];
            int ico = tool_ico[im];
            layer_misa_shift[ila][ico].add( new Vector3D( dx*sscale,  dy*sscale,  dz*sscale));
            layer_misa_angle[ila][ico].add( new Vector3D(thx*ascale, thy*ascale, thz*ascale));

            if(debugMode>=1 || geopar.RICH_GEO_DEBUG>0){
                //System.out.format("QUA QUA %4d %4d  %d %d  %7.2f %7.2f %7.2f  %7.2f %7.2f \n",ila,ico,lla,cco,dx,dy,dz,
                //    layer_misa_shift[ila][ico].mag(),layer_misa_angle[ila][ico].mag()*RICHGeoConstants.MRAD);
                if(layer_misa_shift[ila][ico].mag()>0 || layer_misa_angle[ila][ico].mag()>0){
                    System.out.format("CCDB RICH MISA    ila  %4d ico %3d  (%4d %3d)  shift %s  angle %s \n", lla,cco, ila,ico,
                               layer_misa_shift[ila][ico].toStringBrief(3), layer_misa_angle[ila][ico].toStringBrief(3));
                }
                if(im==23)System.out.format("  \n");
            }

        }


        /*
        * AEROGEL NOMINAL OPTCIS
        */

        int nco[] = {16,22,31,31};
        for (int ila=0; ila<4; ila++){
            for (int ico=0; ico<nco[ila]; ico++){
                aero_refi[ila][ico] = (float) aeroConstants.getDoubleValue("n400", 4,201+ila,ico+1);
                aero_plan[ila][ico] = (float) aeroConstants.getDoubleValue("planarity", 4,201+ila, ico+1);
                if(debugMode>=2 && geopar.RICH_GEO_DEBUG>0)System.out.format("CCDB RICH AERO    ila %4d  ico %3d  n = %8.5f  pla = %8.2f\n", 201+ila, ico+1, aero_refi[ila][ico], aero_plan[ila][ico]);
            }
        }

        if(debugMode>=2){
            int ndo[] = {16,22,32,32};
            for (int ila=0; ila<4; ila++){
                for (int ico=0; ico<ndo[ila]; ico++){
                    for (int iq=0; iq<225; iq++) {
                        int icompo = ico*225+iq+1;
                        System.out.format(" KK 4  %4d %6d  1 312.0 6.0  1 310.0 7.0  1 313.0 10.0 \n", 201+ila, icompo+1);
                    }
                }
            }
        }

    }


    //------------------------------
    public void init_GeoParametersTxT(int ifile){
    //------------------------------
    // To be moved to CCDB

       int debugMode = 1;

        if(ifile==1){

            /*
            * DC_OFFSETs
            */
            String dcoff_filename = new String("CALIB_DATA/DC_offsets_4013.txt");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(dcoff_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {    

                    String[] array = currentLine.split(" ");
                    int idc    = Integer.parseInt(array[0]);
                    int imatch = Integer.parseInt(array[1]);
                    int iref   = Integer.parseInt(array[2]);
                    int ipiv   = Integer.parseInt(array[3]);
                    int isur   = Integer.parseInt(array[4]);

                    float  ss  = Float.parseFloat(array[5]);
                    float  sa  = Float.parseFloat(array[6]);

                    int inp    = Integer.parseInt(array[7]);
                    
                    float  hr  = Float.parseFloat(array[8]);

                    geopar.DO_ALIGNMENT    = idc;
                    geopar.FORCE_DC_MATCH  = imatch;
                    geopar.MISA_RICH_REF   = iref;
                    geopar.MISA_PMT_PIVOT  = ipiv;
                    geopar.APPLY_SURVEY    = isur;

                    geopar.MISA_SHIFT_SCALE     = (double) ss ;
                    geopar.MISA_ANGLE_SCALE     = (double) sa;

                    if(debugMode>=1 || geopar.RICH_GEO_DEBUG>0){

                        System.out.format("TEXT PARA    DO_ALIGNMENT                 %7d \n", geopar.DO_ALIGNMENT);
                        System.out.format("TEXT PARA    FORCE_DC_MATCH               %7d \n", geopar.FORCE_DC_MATCH);
                        System.out.format("TEXT PARA    MISA_RICH_REF                %7d \n", geopar.MISA_RICH_REF);
                        System.out.format("TEXT PARA    MISA_PMT_PIVOT               %7d \n", geopar.MISA_PMT_PIVOT);
                        System.out.format("TEXT PARA    APPLY_SURVEY                 %7d \n", geopar.APPLY_SURVEY);

                        System.out.format("TEXT PARA    MISA_SHIFT_SCALE             %7.3f \n", geopar.MISA_SHIFT_SCALE);
                        System.out.format("TEXT PARA    MISA_ANGLE_SCALE             %7.3f \n", geopar.MISA_ANGLE_SCALE);

                    }

                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", dcoff_filename);
                e.printStackTrace();
            }


            double sscale = geopar.MISA_SHIFT_SCALE;
            double ascale = geopar.MISA_ANGLE_SCALE / RICHGeoConstants.MRAD;  // to convert in rad


            /*
            *  SINGLE COMPONENT MISALIGNMENT
            *  This comes on top of the RICH survey and global transformation
            */
            /*for (int ila=0; ila<NLAY+1; ila++){
                for (int ico=0; ico<NCOMPO+1; ico++){
                    layer_misa_shift[ila][ico] = new Vector3D(0., 0., 0.);
                    layer_misa_angle[ila][ico] = new Vector3D(0., 0., 0.);
                }
            }*/

            String misaco_filename = new String("CALIB_DATA/RICHlayer_misalignment.txt");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(misaco_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {    

                    String[] array = currentLine.split(" ");
                    int isec = Integer.parseInt(array[0]);
                    int lla = Integer.parseInt(array[1]);
                    int cco = Integer.parseInt(array[2]);

                    float  dx  = Float.parseFloat(array[3]);
                    float  dy  = Float.parseFloat(array[4]);
                    float  dz  = Float.parseFloat(array[5]);
                    float  thx = Float.parseFloat(array[6]);
                    float  thy = Float.parseFloat(array[7]);
                    float  thz = Float.parseFloat(array[8]);

                    int[] ind = {0,0};
                    if(convert_indexes(lla, cco, ind)){

                        int ila=ind[0];
                        int ico=ind[1];
                        if(debugMode>=0)System.out.format("MISA conversion %4d %3d --> %4d %3d \n",lla,cco,ila,ico);

                        // the rotation is assumed to be in the component local ref system
                        layer_misa_shift[ila][ico].add( new Vector3D( dx*sscale,  dy*sscale,  dz*sscale));
                        layer_misa_angle[ila][ico].add( new Vector3D(thx*ascale, thy*ascale, thz*ascale));

                        if(debugMode>=1 || geopar.RICH_GEO_DEBUG>0){
                            if(layer_misa_shift[ila][ico].mag()>0 || layer_misa_angle[ila][ico].mag()>0){
                                System.out.format("TXT MISA   layer %4d ico %3d  (%4d %3d)  shift %s  angle %s \n", ila,ico,lla,cco, 
                                   layer_misa_shift[ila][ico].toStringBrief(3), layer_misa_angle[ila][ico].toStringBrief(3));
                            }
                        }

                    }else{
                        System.out.format("Unsupported imisalignment for layer %3d %3d \n",lla,cco);
                    }
                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", misaco_filename);
                e.printStackTrace();
            }

            if(debugMode>=1 || geopar.RICH_GEO_DEBUG>0){
                System.out.format("\n");
                for (int ila=0; ila<NLAY; ila++){
                    for(int ico=0; ico<NCOMPO; ico++){
                        if(layer_misa_shift[ila][ico].multiply(10.).mag()>1.e-3 || layer_misa_angle[ila][ico].multiply(1000.).mag()>1.e-3){
                            System.out.format("ALL MISA   layer %4d ico %3d  shift (mm) %s  angle (mrad) %s \n", ila,ico,
                            layer_misa_shift[ila][ico].multiply(10.).toStringBrief(2), layer_misa_angle[ila][ico].multiply(1000.).toStringBrief(2));
                        }
                    }
                }
                System.out.format("\n");
            }
        }


        if(ifile==3){

            /*
            * AEROGEL NOMINAL OPTCIS
            */

            String aero_filename = new String("CALIB_DATA/aerogel_passports.txt");

            try {

                BufferedReader bf = new BufferedReader(new FileReader(aero_filename));
                String currentLine = null;

                while ( (currentLine = bf.readLine()) != null) {    

                    String[] array = currentLine.split(" ");
                    int idlay = Integer.parseInt(array[1]);
                    int iaer = Integer.parseInt(array[2]);
                    
                    if(debugMode>=1)System.out.format("Read optics for AERO lay %3d  compo %3d", idlay, iaer); 
                    float refi = Float.parseFloat(array[5]);
                    float plana = Float.parseFloat(array[11]);
                    aero_refi[idlay-201][iaer-1] = refi;
                    aero_plan[idlay-201][iaer-1] = plana;
                    //aero_refi[idlay-201][iaer-1] = (float) RICHConstants.RICH_AEROGEL_INDEX;
                    if(debugMode>=1)System.out.format(" n = %8.5f   pla = %8.2f \n", aero_refi[idlay-201][iaer-1], aero_plan[idlay-201][iaer-1]);
                    
                }

            } catch (Exception e) {

                System.err.format("Exception occurred trying to read '%s' \n", aero_filename);
                e.printStackTrace();
            }

            if(debugMode>=1)System.out.format("initConstants: DONE \n");
            
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
    public void init_Survey(){
    //------------------------------

        int debugMode = 0;

        if(debugMode>=1){
            System.out.format("---------------\n");
            System.out.format("Calculate RICH Alignment from Survey\n");
            System.out.format("---------------\n");
        }

        /* 
        *  Define nominal plane
        */
        Point3D RDA = new Point3D(-300.274,  168.299,  460.327);
        Point3D RDB = new Point3D(-300.309,  -168.299, 460.310);
        Point3D RDC = new Point3D(-31.102,     0., 585.886);

        Triangle3D f = new Triangle3D( RDA, RDB, RDC);
        Vector3D nomi_n = f.normal();
        Point3D  nomi_b = f.center();
        Shape3D  nomi_plane = new Shape3D(f);

        Shape3D rich_survey_plane = new Shape3D(f);
        
        /*
        *   Define surveyed plane
        */
        Point3D mRDA = new Point3D(-301.211, 168.505, 467.514);
        Point3D mRDB = new Point3D(-300.514, -167.929, 465.334);
        Point3D mRDC = new Point3D(-31.552, -0.086, 591.329);

        Triangle3D mf= new Triangle3D( mRDA, mRDB, mRDC);
        Shape3D real_plane = new Shape3D(mf);
        Vector3D real_n = mf.normal();
        Point3D  real_b = mf.center();

        if(debugMode>=1){
            // check possible deformations
            double check_a = f.point(1).distance(f.point(0));
            double check_b = f.point(2).distance(f.point(1));
            double check_c = f.point(2).distance(f.point(0));

            double checp_a = mf.point(1).distance(mf.point(0));
            double checp_b = mf.point(2).distance(mf.point(1));
            double checp_c = mf.point(2).distance(mf.point(0));

            System.out.format("Sides nominal    %8.3f %8.3f %8.3f \n",check_a, check_b, check_c);
            System.out.format("Sides real       %8.3f %8.3f %8.3f \n",checp_a, checp_b, checp_c);
        }

        // define shift among barycenters
        Vector3D diff_b = real_b.vectorFrom(nomi_b);
        //rich_survey_center = nomi_b;

        Vector3D rich_xref = new Vector3D(Math.cos(25/180.*Math.PI),0.,Math.sin(25/180.*Math.PI));
        Vector3D rich_yref = new Vector3D(0.,1.,0.);
        Vector3D rich_zref = new Vector3D(-Math.sin(25/180.*Math.PI),0.,Math.cos(25/180.*Math.PI));
        survey_frame = new RICHFrame (rich_xref, rich_yref, rich_zref, nomi_b.toVector3D());


        // define rotation angle and vector
        Vector3D dir = nomi_n.cross(real_n).asUnit();
        double ang = Math.acos(nomi_n.dot(real_n));
        Vector3D rota_n = dir.multiply(ang);

        double mrad = RICHGeoConstants.MRAD;

        rich_survey_shift = diff_b.clone();  
        rich_survey_angle = rota_n.clone();

        //Vector3d dcrich_shift = new Vector3d(global_shift[0], global_shift[1], global_shift[2]);
        //this.rich_survey_shift = new Vector3d(misa_shift.plus(dcrich_shift));
        //Vector3d dcrich_angle = new Vector3d(global_angle[0], global_angle[1], global_angle[2]);
        //this.rich_survey_angle = new Vector3d(misa_angle.plus(dcrich_angle));

        if(debugMode>=1){
            /*System.out.format(" -------------------- \n");
            System.out.format(" survey angle %s \n", rota_n.multiply(mrad).toStringBrief(2));
            System.out.format(" survey shift %.2f %7.2f \n", diff_b.x, diff_b.y, diff_b.z);
            System.out.format(" -------------------- \n");
            System.out.format(" misalg angle %7.2f %7.2f %7.2f \n", misa_angle.x*mrad, misa_angle.y*mrad, misa_angle.z*mrad);
            System.out.format(" misalg shift %7.2f %7.2f %7.2f \n", misa_shift.x, misa_shift.y, misa_shift.z);
            System.out.format(" -------------------- \n");
            System.out.format(" extern angle %7.2f %7.2f %7.2f \n", dcrich_angle.x*mrad, dcrich_angle.y*mrad, dcrich_angle.z*mrad);
            System.out.format(" extern shift %7.2f %7.2f %7.2f \n", dcrich_shift.x, dcrich_shift.y, dcrich_shift.z);
            System.out.format(" -------------------- \n");*/
            System.out.format(" survey angle %s \n", rich_survey_angle.multiply(mrad).toStringBrief(2));
            System.out.format(" survey shift %s \n", rich_survey_shift.toStringBrief(2));
            System.out.format(" -------------------- \n");
        
            System.out.format(" Check survey plane \n");
            System.out.format(" -------------------- \n");
            double thex = rich_survey_angle.dot(new Vector3D(1.,0.,0.));
            double they = rich_survey_angle.dot(new Vector3D(0.,1.,0.));
            double thez = rich_survey_angle.dot(new Vector3D(0.,0.,1.));

            System.out.format("Rot Angles NewRef %7.2f | %7.2f %7.2f %7.2f \n", ang*mrad, thex*mrad, they*mrad, thez*mrad);

            Vector3D new_n = nomi_n.clone();
            new_n.rotateZ(thez);
            new_n.rotateY(they);
            new_n.rotateX(thex);

            System.out.format("Normal nominal %s \n", nomi_n.toString());
            System.out.format("Normal real    %s \n", real_n.toString());
            System.out.format("Normal rotated %s \n", new_n.toString());
            System.out.format("\n");
            System.out.format("Baryc  nominal %s \n", nomi_b.toString());
            System.out.format("Baryc  real    %s \n", real_b.toString());
            System.out.format("Baryc  diff    %s \n", diff_b.toString());
            System.out.format("\n");

            show_Shape3D(nomi_plane, null, null);

            show_Shape3D(real_plane, null, null);

            /* test misalignment angle and shift
            Face3D at = new Triangle3D( RDA, RDB, RDC);
            Shape3D test_plane = new Shape3D(at);

            //misalign_TrackingPlane(test_plane, -1);
            show_Shape3D(test_plane, null, null);

            double aang = 10./57.3;
            Vector3d ini = new Vector3d(Math.sin(aang), 0., Math.cos(aang));
            Vector3d anor = new Vector3d(0., 0.1, 1.);
            Vector3d nor = anor.normalized();
            Vector3d out = Transmission(ini, nor, 1.10, 1.00);
            System.out.format(" nor %s \n", toString(nor));
            System.out.format(" ini %s \n", toString(ini));
            System.out.format(" out %s \n", toString(out));
            

            double aa = Math.acos(ini.dot(nor)/ini.magnitude());
            double bb = Math.acos(out.dot(nor)/out.magnitude());
            double cc = Math.acos(ini.dot(Vector3d.Z_ONE)/ini.magnitude());
            double dd = Math.acos(out.dot(Vector3d.Z_ONE)/out.magnitude());
            System.out.format(" ini angle vn %8.3f  vz %8.3f \n", aa*57.3, bb*57.3);
            System.out.format(" out angle vn %8.3f  vz %8.3f \n", cc*57.3, dd*57.3);

            Vector3d out2 = Transmission2(ini, nor, 1.10, 1.00);
            aa = Math.acos(ini.dot(nor)/ini.magnitude());
            bb = Math.acos(out2.dot(nor)/out2.magnitude());
            cc = Math.acos(ini.dot(Vector3d.Z_ONE)/ini.magnitude());
            dd = Math.acos(out2.dot(Vector3d.Z_ONE)/out2.magnitude());
            System.out.format(" ini angle vn %8.3f  vz %8.3f \n", aa*57.3, bb*57.3);
            System.out.format(" out angle vn %8.3f  vz %8.3f \n", cc*57.3, dd*57.3);
            */


        }

    }


    //------------------------------
    public void init_RICHLayers(int iEngine){
    //------------------------------
    // Take RICHFactory Layers of Geant4 volumes (for GEMC) and convert in coatjava Layers 
    // of RICH components accounting for optical descriptiors plus basic tracking 
    // planes for effective ray tracing
    // ATT: to be done: aerogel cromatic dispersion, mirror reflectivity vs wavelength

        int debugMode = 1;

        /*
        * relevant characterization of the basic tracking planes
        */
        Vector3D front   = new Vector3D(-0.42,   0.00,   0.91);
        Vector3D left    = new Vector3D(-0.50,  -0.87,   0.00);
        Vector3D right   = new Vector3D(-0.50,   0.87,   0.00);
        Vector3D bottom  = new Vector3D(-1.00,   0.00,   0.00);
        Vector3D back    = new Vector3D( 0.42,   0.00,  -0.91);
        Vector3D sphere  = new Vector3D( 0.76,   0.00,  -0.65);

        int factory_lay[] = {201,202,203,204,301,301,301,301,301,301,301,302,401};
        int type_lay[] = {1, 1, 1, 1, 2, 2, 3, 3, 3, 3, 3, 4, 5};
        String name_lay[] = {"aerogel_2cm_B1" ,"aerogel_2cm_B2", "aerogel_3cm_L1", "aerogel_3cm_L2", 
                           "mirror_front_B1", "mirror_front_B2", "mirror_left_L1", "mirror_left_L2", "mirror_right_L1", "mirror_right_L2", "mirror_bottom", 
                           "mirror_sphere", "mapmts"};
        Vector3D view_lay[] = {front, front, front, front, front, front, left, left, right, right, bottom, sphere, back};

        /*
        * Generate the layers of components
        */
        for (int ilay=0; ilay<NLAY; ilay++){

            int idlayer = factory_lay[ilay];
            int tlayer = type_lay[ilay];
            String slayer = name_lay[ilay];
            Vector3D vlayer = view_lay[ilay];
            if(debugMode>=1){
                System.out.format("-------------------------\n");
                System.out.format("Create Layer %d id  %d: %s dir %s \n",ilay, idlayer, slayer, vlayer.toStringBrief(2));
                System.out.format("-------------------------\n");
            }
            RICHLayer layer = new RICHLayer(ilay, slayer, vlayer);

            //if(iEngine==0 && (ilay>3 && ilay<12)) continue;
            if(iEngine==1 || (ilay<4 || ilay==12)) {

            for (int ico=0; ico<get_RICHFactory_Size(idlayer); ico++){
                RICHComponent compo = get_RICHFactory_Component(idlayer, ico);
                compo.set_type(tlayer);
                if(debugMode>=1)System.out.format(" Lay %3d component %3d  bary %s\n", idlayer, ico, get_CSGBary(compo.get_CSGVol()));

                // define optical properties, so far only aerogel
                if(idlayer<300){  
                    compo.set_index((float) aero_refi[ilay][ico]);
                    compo.set_planarity((float) aero_plan[ilay][ico]/10);  // converted into cm
                }else{
                    compo.set_index((float) 1.000);
                    compo.set_planarity((float) 0.000);
                }

                if(debugMode>=3 && idlayer==301){
                    if(get_PlaneMirrorSide(compo).equals(slayer)){
                        compo.showComponent();
                        dump_StlComponent(compo.get_CSGVol());
                    }
                }

                // regrouping of the planar mirros into planes
                if(idlayer!=301){
                    layer.add(compo);
                }else{
                    if(get_PlaneMirrorSide(compo).equals(slayer)){
                        if(debugMode>=1)System.out.format(" ---> add to Layer %3d %s id (%3d %3d) \n",ilay,slayer,idlayer,ico);
                        layer.add(compo); 
                    }
                }

            }
            }

            if(debugMode>=1)System.out.format("add layer %d \n",ilay);
            opticlayers.add(layer);

        }


        /*
        * Generate and misalign the basic planes for tracking 
        */

        rich_frame = survey_frame.clone();
        // QUE: il survey non corrisponde al nominale e sovrascrive il PIVOT

        for (int ilay=0; ilay<NLAY; ilay++){

            if(iEngine==0 && (ilay>3 && ilay<12)) continue;
            if(debugMode>=1)System.out.format("generate surfaces for layer %d \n",ilay);

            generate_TrackingPlane(ilay);

            if(geopar.DO_ALIGNMENT==1)misalign_TrackingPlane(ilay);

            store_TrackingPlane(ilay);

        }

        
        if(iEngine>0){
            /*
            *  Generate Pixel map on the misaligned MAPMT plane
            */
            RICHLayer layer = get_Layer("mapmts");
            List<Integer> compo_list = layer.get_CompoList();
            Shape3D compo_misa = layer.get_TrackingSurf();
            generate_Pixel_Map(layer.get_id(), 0, compo_misa, compo_list);

            if(debugMode>=1)show_Shape3D(compo_misa, null, "CC");
            if(debugMode>=1)show_RICH("Real RICH Geometry", "RR");
        }

    }

    // ----------------
    public void testTraj() {
    // ----------------

        Plane3D pl_mapmt = get_MaPMTforTraj();
        pl_mapmt.show();

        Point3D pa[] = new Point3D[3];
        for (int ia=0; ia<3; ia++){
            Plane3D pl_aero = get_AeroforTraj(ia);
            pl_aero.show();
            pa[ia]=pl_aero.point();
            System.out.format("Ref point %s \n",pa[ia].toStringBrief(2));

        }

        Point3D IP = new Point3D(0.,0.,0.);
        for (int ia=0; ia<3; ia++){
            Line3D lin = new Line3D(IP, pa[ia]);
            int iplane = select_AeroforTraj(lin, lin, lin);
            System.out.format("For LIN %d select plane %d \n",ia,iplane);

        }

    }

    public Plane3D get_TrajPlane(int sector, int iplane) {
        if(sector==4) {
            if(iplane==DetectorLayer.RICH_MAPMT) return this.get_MaPMTforTraj();
            else return this.get_AeroforTraj(iplane);
            }
        else return null;
    }
    
    //------------------------------
    public Plane3D get_MaPMTforTraj() {
    //------------------------------

        RICHLayer layer = get_Layer("mapmts");
        return layer.get_TrajPlane();

    }


    //------------------------------
    public Plane3D get_AeroforTraj(int ilayer) {
    //------------------------------

        Plane3D layer = null;
        if(ilayer==DetectorLayer.RICH_AEROGEL_B1)      layer = this.get_Layer("aerogel_2cm_B1").get_TrajPlane();
        else if(ilayer==DetectorLayer.RICH_AEROGEL_B2) layer = this.get_Layer("aerogel_2cm_B2").get_TrajPlane();
        else if(ilayer==DetectorLayer.RICH_AEROGEL_L1) layer = this.get_Layer("aerogel_3cm_L1").get_TrajPlane();

        return layer;

    }


    //------------------------------
    public int select_AeroforTraj(Line3D first, Line3D second, Line3D third) {
    //------------------------------

        RICHIntersection entra = get_Layer("aerogel_2cm_B2").find_Entrance(second, -2);
        if(entra!=null) return 1;

        if(entra==null) entra = get_Layer("aerogel_3cm_L1").find_Entrance(third, -2);
        if(entra!=null) return 2;

        // return a solution plane in any case
        return 0;

    }


    //------------------------------
    public RICHPixelMap get_PixelMap() { return pixelmap; }
    //------------------------------


    //------------------------------
    public Vector3d GetPixelCenter(int ipmt, int anode){
    //------------------------------

        Vector3d Vertex = richfactory.GetPhotocatode(ipmt).getVertex(2);
        Vector3d VPixel = Vertex.plus(pmtpixels.GetPixelCenter(anode));
        //System.out.format("Std  vtx %8.3f %8.3f %8.3f \n",Vertex.x, Vertex.y, Vertex.z);
        return new Vector3d (VPixel.x, -VPixel.y, VPixel.z);

    }


    //------------------------------
    public Vector3d get_Pixel_Center(int ipmt, int anode){
    //------------------------------

        int ilay = 12;
        Face3D compo_face = get_Layer(ilay).get_CompoFace(ipmt-1, 0);
        Vector3d Vertex = toVector3d( compo_face.point(1) );
        //System.out.format("Misa vtx %8.3f %8.3f %8.3f \n",Vertex.x, Vertex.y, Vertex.z);
        //System.out.println(pmtpixels.GetPixelCenter(anode));
        Vector3d VPixel = Vertex.plus(pmtpixels.GetPixelCenter(anode));
        return new Vector3d (VPixel.x, -VPixel.y, VPixel.z);

    }


    //------------------------------
    public Shape3D build_GlobalPlane(Shape3D plane, Vector3D orient) {
    //------------------------------
        /*
        *  build a global tracking plane from the detailed component surface
        * ATT: assumes a plane (with unique normal) with vertical (along y) edges 
        */

        int debugMode = 0;
        if(plane==null) return null;
        if(debugMode>=1)System.out.format("build_GlobalPlane: orient %s \n",orient.toStringBrief(3));

        Vector3d extre1 = new Vector3d(0.0, 0.0, 0.0);
        Vector3d extre2 = new Vector3d(0.0, 0.0, 0.0);
        Vector3d extre3 = new Vector3d(0.0, 0.0, 0.0);
        Vector3d extre4 = new Vector3d(0.0, 0.0, 0.0);

        /*
        * look for the extremes in x
        */
        double xmin = 999.0;
        double xmax = -999.0;
        for (int ifa=0; ifa<plane.size(); ifa++){
            Face3D f = plane.face(ifa);
            if(toTriangle3D(f).normal().angle(orient)>1.e-2)continue;
            for (int ipo=0; ipo<3; ipo++){

                if(f.point(ipo).x() < xmin) xmin=f.point(ipo).x();
                if(f.point(ipo).x() > xmax) xmax=f.point(ipo).x();
            }
        }  
        if(debugMode>=1)System.out.format("  x range: %7.2f %7.2f \n",xmin,xmax);

        /*
        *  look for the points at exreme y for xmin 
        */
        double ymin = 999.0;
        double ymax = -999.0;
        for (int ifa=0; ifa<plane.size(); ifa++){
            Face3D f = plane.face(ifa);
            if(toTriangle3D(f).normal().angle(orient)>1.e-2)continue;
            for (int ipo=0; ipo<3; ipo++){

                if(Math.abs(f.point(ipo).x() - xmin) < 0.5 && f.point(ipo).y() < ymin ) {
                    ymin = f.point(ipo).y();
                    extre1 = toVector3d(f.point(ipo));
                }
                if(Math.abs(f.point(ipo).x() - xmin) < 0.5 && f.point(ipo).y() > ymax ) {
                    ymax = f.point(ipo).y();
                    extre2 = toVector3d(f.point(ipo));
                }
            }
        }

        /*
        * look for the points at exreme y for xmax 
        */
        ymin = 999.0;
        ymax = -999.0;
        for (int ifa=0; ifa<plane.size(); ifa++){
            Face3D f = plane.face(ifa);
            if(toTriangle3D(f).normal().angle(orient)>1.e-2)continue;
            for (int ipo=0; ipo<3; ipo++){

                if(Math.abs(f.point(ipo).x() - xmax) < 0.5 && f.point(ipo).y() < ymin ) {
                    ymin = f.point(ipo).y();
                    extre3 = toVector3d(f.point(ipo));
                }
                if(Math.abs(f.point(ipo).x() - xmax) < 0.5 && f.point(ipo).y() > ymax ) {
                    ymax = f.point(ipo).y();
                    extre4 = toVector3d(f.point(ipo));
                }
            }
        }

        /*
        *  preserve the same normal of the original plane
        */
        Face3D half1 = new Triangle3D( toPoint3D(extre1), toPoint3D(extre2), toPoint3D(extre3));
        Face3D half2 = new Triangle3D( toPoint3D(extre2), toPoint3D(extre4), toPoint3D(extre3));
        Shape3D guess_one = new Shape3D(half1, half2);

        Face3D half3 = new Triangle3D( toPoint3D(extre3), toPoint3D(extre2), toPoint3D(extre1));
        Face3D half4 = new Triangle3D( toPoint3D(extre3), toPoint3D(extre4), toPoint3D(extre2));
        Shape3D guess_two = new Shape3D(half3, half4);

        Vector3D plane_norm = orient;
        Vector3D guess_norm = toVector3D(get_Shape3D_Normal(guess_one));
        double ang = guess_norm.angle(plane_norm)*RICHGeoConstants.RAD;

        if(debugMode>=1){
            guess_one.show();
            System.out.format("Guess one normal %s --> %7.2f \n",guess_norm.toStringBrief(2), ang*57.3);
            guess_two.show();
            Vector3D other_norm = toVector3D(get_Shape3D_Normal(guess_two));
            double other_ang = other_norm.angle(plane_norm)*RICHGeoConstants.RAD;
            System.out.format("Guess two normal %s --> %7.2f \n",other_norm.toStringBrief(2), other_ang*57.3);
        }

        if(ang<10){
            return guess_one;
        }else{
            return guess_two;
        }

    }


    //------------------------------
    public void build_GlobalPlanes(RICHLayer layer, Vector3D orient) {
    //------------------------------
        //build the tracking plane of the component with given orientation

        int debugMode = 0;


        if(debugMode>=2){
            Vector3D inside = layer.get_Vinside();
            System.out.format("build_GlobalPlane: generate global plane for layer %3d \n",layer.get_id());
            System.out.format("inside vect: %s \n",inside.toStringBrief(3));
            System.out.format("orient vect: %s  --> %7.3f\n",orient.toStringBrief(3), orient.angle(inside)*57.3);
        }

        Shape3D global_surf = null;

        if(layer.is_mirror()){

            if(layer.is_planar_mirror()) global_surf = copy_Shape3D(layer.merge_CompoSurfs());
            if(layer.is_spherical_mirror()) global_surf = copy_Shape3D(layer.get_NominalPlane());

        }else{
            global_surf = build_GlobalPlane(layer.merge_CompoSurfs(), orient);

            if(layer.is_aerogel()){
                Shape3D other_global = build_GlobalPlane(layer.merge_CompoSurfs(), orient.multiply(-1.0));
                merge_Shape3D(global_surf, other_global);
            }
        }

        layer.set_GlobalSurf( global_surf);
        if(debugMode>=1 && global_surf.size()>0){
            String head = String.format("GLOB %3d 0 ",layer.get_id());
            System.out.format("Globa %3d Normal %s \n",layer.get_id(),toString(get_Shape3D_Normal(global_surf)));
            for (int ifa=0; ifa<global_surf.size(); ifa++){
                System.out.format("Face %3d Normal %s \n",ifa,toTriangle3D(global_surf.face(ifa)).normal().asUnit().toStringBrief(3));
            }
            show_Shape3D(global_surf, null, head);
        }
 
    }


    //------------------------------
    public void build_CompoSpheres(RICHLayer layer) {
    //------------------------------
        //build the spherical surface of the component 

        int debugMode = 0;

        /*
        *   define the spherical surfaces when needed
        */
        if(layer.is_spherical_mirror()){

            Sphere3D sphere = new Sphere3D(-45.868, 0.0, 391.977, 270.);
            layer.set_TrackingSphere(sphere);
            for (int ico=0; ico<layer.size(); ico++){ 
                layer.set_TrackingSphere( new Sphere3D(-45.868, 0.0, 391.977, 270.), ico); 
            }

        }

        if(layer.is_aerogel()){

            for (int ico=0; ico<layer.size(); ico++){

                double radius = layer.get(ico).get_radius();
                Vector3D normal = layer.get_CompoNormal(ico);
                Vector3D center = layer.get_CompoCenter(ico, normal);

                Sphere3D sphere = new Sphere3D(center.x(), center.y(), center.z(), radius);
                if(debugMode>=1)System.out.format(" AERO lay %3d ico %3d : sphere center %s radius %7.2f \n",layer.get_id(),ico,toString(center), radius);
                layer.set_TrackingSphere(sphere, ico);

            }
        }

    }


    //------------------------------
    public void build_CompoSurfs(RICHLayer layer, Vector3D orient) {
    //------------------------------
        //build the tracking plane of the component with given orientation

        int debugMode = 0;

        Vector3D inside = layer.get_Vinside();

        if(debugMode>=2){
            System.out.format("build_CompoSurfs: generate tracking plane for layer %3d \n",layer.get_id());
            System.out.format("inside vect: %s \n",inside.toStringBrief(3));
            System.out.format("orient vect: %s  --> %7.3f\n",orient.toStringBrief(3), orient.angle(inside)*57.3);
        }

        for (int ico=0; ico<layer.size(); ico++){
            RICHComponent compo = layer.get(ico);
            Shape3D plane = new Shape3D();
            Vector3D cbary = layer.get_CompoCSGBary(ico);

            if(layer.is_spherical_mirror()){

                /*
                * Build from Nominal planes (nominal orientation)
                */
                Shape3D submir = generate_Nominal_Plane(layer.get_id(), ico+1);
                for(int ifa=0; ifa<submir.size(); ifa++) plane.addFace(submir.face(ifa)); 
                     
            }else{

                /*
                * Build from CSG volumes
                */
                int ipo = 0;
                int igo = 0;
                for (Triangle3D tri: toTriangle3D(compo.get_CSGVol().getPolygons()) ){

                    Vector3D tri_norm = tri.normal().asUnit();
                    double norm_ang = tri_norm.angle(orient);
                    double norm_oppang = tri_norm.angle(orient.multiply(-1));
                    Vector3D bary_diff = (tri.center().toVector3D().sub(cbary)).asUnit();
                    double bary_dot = tri_norm.dot(bary_diff);
                    if(debugMode>=2){System.out.format("Compo %4d tri %4d  norm_ang %7.2f : %7.2f bary_dot %7.2f (%s %s)", ico, ipo, 
                                     norm_ang*57.3, norm_oppang*57.3, bary_dot*57.3, toString(tri.center()), toString(cbary));}
                    
                    /*
                    * in case of multiple surfaces (i.e. for glass skin mirrors), take the innermost.
                    */
                    if((norm_ang<1e-2 && bary_dot>0) || (layer.is_aerogel() && norm_oppang<1e-2 && bary_dot>0) ){  

                        plane.addFace(tri); 
                        if(debugMode>=2)System.out.format("    ---> take this face %3d %s\n",igo,tri_norm.toStringBrief(2));
                        igo++;
                    }else{
                        if(debugMode>=2)System.out.format("  \n");
                    }
                    ipo++;
                }
            }

            compo.set_TrackingSurf(plane);
            if(debugMode>=1 && plane.size()>0){
                System.out.format("Compo %3d %3d Normal %s \n",layer.get_id(),ico,toString(get_Shape3D_Normal(plane)));
                String head = String.format("COMP %3d %3d ",layer.get_id(),ico);
                show_Shape3D(plane, null, head);
            }
        }

    }
 
     //------------------------------
     public Vector3d get_angles(Vector3d vec) {
     //------------------------------

        Vector3d vone = vec.normalized();
        Vector3d vang = new Vector3d( Math.acos(vone.dot(Vector3d.X_ONE)), Math.acos(vone.dot(Vector3d.Y_ONE)), Math.acos(vone.dot(Vector3d.Z_ONE)));
        return vang;

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
    public Triangle3D toTriangle3D(Face3D face){ return new Triangle3D(face.point(0), face.point(1), face.point(2)); }
    //------------------------------

    //------------------------------
    public ArrayList<Triangle3D> toTriangle3D(List<Polygon> pols){
    //------------------------------

        ArrayList<Triangle3D> trias = new ArrayList<Triangle3D>();

        for (Polygon pol: pols){
            for (int iv=2; iv<pol.vertices.size(); iv++){
                Triangle3D tri = new Triangle3D(toPoint3D(pol.vertices.get(0)), toPoint3D(pol.vertices.get(iv-1)), toPoint3D(pol.vertices.get(iv)));
                trias.add(tri);
            }
        }
        
        return trias;
    }

    //------------------------------
    public void misalign_Layer(RICHLayer layer){
    //------------------------------

        int debugMode = 0;

        /*
        *  To account for SURVEY
        */
        if(geopar.APPLY_SURVEY==1){
            if(debugMode>=1)System.out.format(" --> SURVEY %s %s \n", toString(rich_survey_shift), toString(rich_survey_angle));

            misalign_Element( layer.get_GlobalSurf(), survey_frame, rich_survey_angle, rich_survey_shift);
            misalign_Element( layer.get_TrackingSphere(), survey_frame, rich_survey_angle, rich_survey_shift);
            for(int ico=0; ico<layer.size(); ico++){
                misalign_Element( layer.get_TrackingSurf(ico), survey_frame, rich_survey_angle, rich_survey_shift);
                misalign_Element( layer.get_TrackingSphere(ico), survey_frame, rich_survey_angle, rich_survey_shift);
            }
        }


        /*
        *  To account for global RICH misalignments
        */
        Vector3D rshift = layer_misa_shift[0][0];
        Vector3D rangle = layer_misa_angle[0][0];
        if(rangle.mag()>0 || rshift.mag()>0){
            if(debugMode>=1)System.out.format(" -->  asRICH %s %s \n", toString(rshift), toString(rangle));

            
            if(debugMode>=1)System.out.format("     --> global \n");
            misalign_Element( layer.get_GlobalSurf(), rich_frame, rangle, rshift);
            misalign_Element( layer.get_TrackingSphere(), rich_frame, rangle, rshift);

            for(int ico=0; ico<layer.size(); ico++){
                if(debugMode>=1)System.out.format("     --> compo %3d \n",ico);
                misalign_Element( layer.get_TrackingSurf(ico), rich_frame, rangle, rshift);
                misalign_Element( layer.get_TrackingSphere(ico), rich_frame, rangle, rshift);
            }
        }


        /*
        *  To account for Layer misalignment 
        */
        int ilay = layer.get_id();
        RICHFrame lframe = layer.generate_LocalRef();
        Vector3D lshift = layer_misa_shift[ilay+1][0];
        Vector3D langle = layer_misa_angle[ilay+1][0];
        if(langle.mag()>0 || lshift.mag()>0){
            if(debugMode>=1){System.out.format("    -->  asLayer  %d  %s %s \n", ilay, toString(lshift), toString(langle)); }

            if(debugMode>=1)System.out.format("     --> global \n");
            misalign_Element( layer.get_GlobalSurf(), lframe, langle, lshift);
            misalign_Element( layer.get_TrackingSphere(), lframe, langle, lshift);

            for(int ico=0; ico<layer.size(); ico++){
                if(debugMode>=1)System.out.format("     --> compo %3d \n",ico);
                misalign_Element( layer.get_TrackingSurf(ico), lframe, langle, lshift);
                misalign_Element( layer.get_TrackingSphere(ico), lframe, langle, lshift);
            }
        }


        /*
        *  To account for single Component misalignment 
        */
        if(layer.is_spherical_mirror()){
            for(int ico=0; ico<layer.size(); ico++){

                RICHFrame cframe = layer.generate_LocalRef(ico);
                Vector3D cangle = layer_misa_angle[ilay+1][ico+1];
                Vector3D cshift = layer_misa_shift[ilay+1][ico+1];

                if(cangle.mag()==0 && cshift.mag()==0)continue;
                if(debugMode==1){System.out.format("       -->  asCompo %3d  %s %s \n", ico, toString(cshift), toString(cangle));}

                misalign_Element( layer.get_TrackingSurf(ico), cframe, cangle, cshift);
                misalign_Element( layer.get_TrackingSphere(ico), cframe, cangle, cshift);
                if(!layer.CheckSphere(ico))System.out.format("Misalignment issue for lay %3d compo %3d \n",ilay,ico);
            }
        }
    }


    //------------------------------
    public void generate_TrackingPlane(int ilay){
    //------------------------------

        int debugMode = 0;

        RICHLayer layer = get_Layer(ilay);
        Vector3D orient = layer.get_Vinside();

        if(debugMode>=1){
            System.out.format("------------------------\n");
            System.out.format("Generate tracking for Layer %d %s view %s \n", ilay, layer.get_Name(), orient.toStringBrief(3));
            System.out.format("------------------------\n");
        }

        /*
        *  Nominal plane just for reference 
        */
        layer.set_NominalPlane( generate_Nominal_Plane(ilay, 0) );


        /*
        *  For each component, group faces with normal and position vs barycenter along orient
        */
        build_CompoSurfs(layer, orient);
        

        /*
        *  Generate a global plane for fast tracking without gaps
        *  In case of aerogel add the second global face 
        */
        build_GlobalPlanes(layer, orient);


        /*
        *  Select the pivot for the RICH rotations
        */
        if(layer.is_mapmt()) {
            if(geopar.MISA_PMT_PIVOT==1) rich_frame.set_bref(layer.get_SurfBary());
            if(debugMode>=1)System.out.format("RICH PIVOT %s \n",rich_frame.bref().toStringBrief(2));
        }


        /*
        *   define the spherical surfaces when needed
        */
        build_CompoSpheres(layer);

    }

    //------------------------------
    public void misalign_TrackingPlane(int ilay){
    //------------------------------

        int debugMode = 0;

        /*
        *  Apply misalignment around given PIVOT
        */

        if(debugMode>=1){
            System.out.format("------------------------\n");
            System.out.format("Misalign tracking for Layer %d %s\n", ilay, get_Layer(ilay).get_Name());
            System.out.format("------------------------\n");
        }

        RICHLayer layer = get_Layer(ilay);

        /*
        *  Misalign surfs as required
        */
        misalign_Layer(layer);

        /*
        *  Check misalignment effect on survey plane
        *//*
        if(debugMode>=1){
            System.out.format("Centre %s\n",rich_misa_center.toStringBrief(2));
            double mrad = RICHConstants.MRAD;
            System.out.format(" rich   angle %7.2f %7.2f %7.2f \n", this.rich_misa_angle.x*mrad, this.rich_misa_angle.y*mrad, this.rich_misa_angle.z*mrad);
            System.out.format(" rich   shift %7.2f %7.2f %7.2f \n", this.rich_misa_shift.x, this.rich_misa_shift.y, this.rich_misa_shift.z);
            show_Shape3D(rich_survey_plane,"Nominal survey", null);
            //misalign_TrackingPlane(rich_survey_plane, 0);
            show_Shape3D(rich_survey_plane,"Misalig survey", null);
        }*/

    }


    //------------------------------
    public void store_TrackingPlane(int ilay){
    //------------------------------

        int debugMode = 0;

        /*
        *  Store the composite tracking planes
        */

        if(debugMode>=1){
            System.out.format("------------------------\n");
            System.out.format("Store    tracking for Layer %d %s\n", ilay, get_Layer(ilay).get_Name());
            System.out.format("------------------------\n");
        }

        RICHLayer layer = get_Layer(ilay);

        /* 
        *  Store misalignmed tracking surfaces for fast tracking 
        */
        layer.set_TrackingSurf( layer.merge_CompoSurfs());
        layer.set_CompoList( layer.merge_CompoList());
           
    }


    //------------------------------
    public void generate_Pixel_Map(int ilay, int ico, Shape3D compo_plane, List<Integer> compo_list) {
    //------------------------------
    // generate the MAPMT pixel map starting from the corresponding
    // facet of the MAPMT plane aligned in the space
    // QUE: assumes that the MAPMT facets have always the same ordering ?

        int debugMode = 1;

        RICHLayer layer = get_Layer(ilay);
        if(layer.is_mapmt()){

            if(debugMode>=1){
                System.out.format("------------------------\n");
                System.out.format("Generate pixel map for Layer %d %s\n", ilay, get_Layer(ilay).get_Name());
                System.out.format("------------------------\n");
            }

            int found=0;
            Vector3d downversor   = null;
            Vector3d rightversor  = null;
            Vector3d vertex       = null;
            for(int ifa=0; ifa<compo_plane.size(); ifa++){
                if(compo_list.get(ifa)==ico){
                    if(debugMode>=1){ System.out.format("  --> ifa %4d ", ifa); dump_Face( compo_plane.face(ifa) ); }
                    if(found==0){
                        Vector3d vp0 = toVector3d( compo_plane.face(ifa).point(0) );
                        Vector3d vp1 = toVector3d( compo_plane.face(ifa).point(1) );
                        Vector3d vp2 = toVector3d( compo_plane.face(ifa).point(2) );
                        downversor   = (vp0.minus(vp1)).normalized();
                        rightversor  = (vp2.minus(vp1).normalized());
                        vertex       = new Vector3d(vp1);
                        if(debugMode>=1){
                            System.out.format("MAPMT ico %4d  ifa %4d \n",ico, ifa);
                            System.out.format("vtx0  %s \n",toString(vp0));
                            System.out.format("vtx1  %s \n",toString(vp1));
                            System.out.format("vtx2  %s \n",toString(vp2));
                            System.out.format("down  %s \n",toString(downversor));
                            System.out.format("right %s \n",toString(rightversor));
                        }
                        found++;
                    }
                }
            }

            if(downversor!=null && rightversor!= null) {
                pmtpixels = new RICHPixel(new Vector3d(0.,0.,0.), downversor, rightversor);
                if(debugMode>=1){
                    pmtpixels.show_Pixels( vertex );
                    vertex = toVector3d( layer.get_CompoFace(5,0).point(1) );
                    pmtpixels.show_Pixels( vertex );
                    vertex = toVector3d( layer.get_CompoFace(363,0).point(1) );
                    pmtpixels.show_Pixels( vertex );
                    vertex = toVector3d( layer.get_CompoFace(390,0).point(1) );
                    pmtpixels.show_Pixels( vertex );
                }
            }
        }
    }
 

    //------------------------------
    public Shape3D generate_Nominal_Plane(int ilay, int ico){
    //------------------------------

        int debugMode = 0;

        if(ilay<0 || ilay>=NLAY) return null;

        Point3D extre1 = new Point3D(0.0, 0.0, 0.0);
        Point3D extre2 = new Point3D(0.0, 0.0, 0.0);
        Point3D extre3 = new Point3D(0.0, 0.0, 0.0);
        Point3D extre4 = new Point3D(0.0, 0.0, 0.0);

        //  Aerogel 2cm plane within B1 mirror 
        if(ilay==0 && ico==0){
            extre1 = new Point3D(-41.1598, 11.5495, 588.125);
            extre2 = new Point3D(-41.1598, -11.5495, 588.125);
            extre3 = new Point3D(-110.583, 51.6008, 555.752);
            extre4 = new Point3D(-110.583, -51.6008, 555.752);
        }

        //  Aerogel 2cm plane within B2 mirror
        if(ilay==1 && ico==0){
            extre1 = new Point3D(-111.353, 53.6256, 555.393);
            extre2 = new Point3D(-111.353, -53.6256, 555.393);
            extre3 = new Point3D(-165.777, 85.0457, 530.015);
            extre4 = new Point3D(-165.777, -85.0457, 530.015);
        }

        //  Aerogel 6cm plane within CFRP panel
        if((ilay==2 || ilay==3) && ico==0){
            extre1 = new Point3D(-167.565, 85.356, 530.003);
            extre2 = new Point3D(-167.565, -85.356, 530.003);
            extre3 = new Point3D(-240.137, 127.254, 496.162);
            extre4 = new Point3D(-240.137, -127.254, 496.162);
        }

        // Front mirror
        if((ilay==4 || ilay==5) && ico==0){
            extre1 = new Point3D(-37.165,  11.847,  587.781);
            extre2 = new Point3D(-37.165,  -11.847,  587.781);
            extre3 = new Point3D(-165.136,  85.728,  528.107);
            extre4 = new Point3D(-165.136,  -85.728,  528.107);
        }

        // Left-side mirror
        if((ilay==6 || ilay==7) && ico==0){
            extre1 = new Point3D(-39.849,  12.095,  688.630);
            extre2 = new Point3D(-39.849,  12.095,  591.568);
            extre3 = new Point3D(-238.031, 126.515,  526.116);
            extre4 = new Point3D(-229.924, 121.834,  502.935);
        }

        // Right-side mirror
        if((ilay==8 || ilay==9) && ico==0){
            extre1 = new Point3D(-39.849,  -12.095,  688.630);
            extre2 = new Point3D(-39.849,  -12.095,  591.568);
            extre3 = new Point3D(-238.031, -126.515,  526.116);
            extre4 = new Point3D(-229.924, -121.834,  502.935);
        }

        // Bottom mirror
        if(ilay==10 && ico==0){
            extre1 = new Point3D(-39.763,  11.500,  591.601);
            extre2 = new Point3D(-39.763,  -11.500,  591.601);
            extre3 = new Point3D(-39.763,  11.500,  687.101);
            extre4 = new Point3D(-39.763,  -11.500,  687.101);
        }

        //  Spherical mirror
        if(ilay==11){
            if(ico==0){
                extre1 = new Point3D(-146.861, 77.9926, 629.86);
                extre2 = new Point3D(-146.861, -77.9926, 629.86);
                extre3 = new Point3D(-244.481, 134.353, 516.032);
                extre4 = new Point3D(-244.481, -134.353, 516.032);
            }
            if(ico==1){
                extre1 = new Point3D(-186.669, 100.976, 598.990);
                extre2 = new Point3D(-195.823,  42.177, 612.446);
                extre3 = new Point3D(-146.869,  77.996, 629.870);
                extre4 = new Point3D(-150.160,  40.583, 637.633);
            }
            if(ico==2){
                extre1 = new Point3D(-186.670,-100.975, 598.991);
                extre2 = new Point3D(-195.760, -42.186, 612.450);
                extre3 = new Point3D(-146.862, -77.995, 629.860);
                extre4 = new Point3D(-150.160, -40.592, 637.625);
            }
            if(ico==3){
                extre1 = new Point3D(-244.480, 134.356, 516.045);
                extre2 = new Point3D(-219.293, 119.805, 560.660);
                extre3 = new Point3D(-267.718,  66.825, 530.562);
                extre4 = new Point3D(-232.817,  69.694, 573.835);
            }
            if(ico==4){
                extre1 = new Point3D(-244.475,-134.350, 516.040);
                extre2 = new Point3D(-219.291,-119.808, 560.655);
                extre3 = new Point3D(-267.713, -66.825, 530.556);
                extre4 = new Point3D(-232.817, -69.699, 573.836);
            }
            if(ico==5){
                extre1 = new Point3D(-150.187, -40.275, 637.670);
                extre2 = new Point3D(-195.858, -41.878, 612.487);
                extre3 = new Point3D(-150.187,  40.268, 637.668);
                extre4 = new Point3D(-195.842,  41.844, 612.500);
            }
            if(ico==6){
                extre1 = new Point3D(-239.371,   0.150, 580.221);
                extre2 = new Point3D(-274.840,   0.150, 535.006);
                extre3 = new Point3D(-232.873,  69.405, 573.896);
                extre4 = new Point3D(-267.781,  66.526, 530.596);
            }
            if(ico==7){
                extre1 = new Point3D(-239.371,  -0.150, 580.221);
                extre2 = new Point3D(-274.840,  -0.150, 535.010);
                extre3 = new Point3D(-232.873, -69.404, 573.889);
                extre4 = new Point3D(-267.782, -66.530, 530.594);
            }
            if(ico==8){
                extre1 = new Point3D(-236.779,  42.186, 578.135);
                extre2 = new Point3D(-196.078,  42.180, 612.277);
                extre3 = new Point3D(-219.115, 119.693, 560.915);
                extre4 = new Point3D(-186.889, 101.102, 598.779);
            }
            if(ico==9){
                extre1 = new Point3D(-236.810,  41.877, 578.175);
                extre2 = new Point3D(-196.108,  41.835, 612.303);
                extre3 = new Point3D(-236.818, -41.883, 578.157);
                extre4 = new Point3D(-196.105, -41.883, 612.315);
            }
            if(ico==10){
                extre1 = new Point3D(-236.785, -42.182, 578.124);
                extre2 = new Point3D(-196.080, -42.185, 612.272);
                extre3 = new Point3D(-219.114,-119.708, 560.905);
                extre4 = new Point3D(-186.891,-101.097, 598.779);
            }
        }

        // MA-PMTs
        if(ilay==12  && ico==0){
            extre1 = new Point3D(-41.126,  15.850,  694.974);
            extre2 = new Point3D(-41.126,  -15.850,  694.974);
            extre3 = new Point3D(-151.514,  74.150,  643.499);
            extre4 = new Point3D(-151.514,  -74.150,  643.499);
        }

        /*
        *  force the layer orientation 
        */
        Vector3D vinside = get_Layer(ilay).get_Vinside();

        Triangle3D half1 = new Triangle3D( extre1, extre2, extre3);
        Triangle3D half2 = new Triangle3D( extre2, extre4, extre3);
        Vector3D norm1 = half1.normal().asUnit();
        Vector3D norm2 = half2.normal().asUnit();
        Shape3D guess_one = new Shape3D(half1, half2);
        Vector3D norm_one = half1.normal().asUnit();
        double ang_one = norm_one.angle(vinside)*RICHGeoConstants.RAD;

        Triangle3D half3 = new Triangle3D( extre3, extre2, extre1);
        Triangle3D half4 = new Triangle3D( extre3, extre4, extre2);
        Vector3D norm3 = half3.normal().asUnit();
        Vector3D norm4 = half4.normal().asUnit();
        Shape3D guess_two = new Shape3D(half3, half4);
        Vector3D norm_two = half3.normal().asUnit();
        double ang_two = norm_two.angle(vinside)*RICHGeoConstants.RAD;

        if(debugMode>=1){
            System.out.format("Look for Nominal plane %3d ico %3d\n",ilay,ico);
            System.out.format("norm1 %s \n",norm1.toStringBrief(3));
            System.out.format("norm2 %s \n",norm2.toStringBrief(3));
            System.out.format("norm3 %s \n",norm3.toStringBrief(3));
            System.out.format("norm4 %s \n",norm4.toStringBrief(3));
            guess_one.show();
            System.out.format("Guess one normal %s --> %7.2f \n",norm_one.toStringBrief(3), ang_one);
            guess_two.show();
            System.out.format("Guess two normal %s --> %7.2f \n",norm_two.toStringBrief(3), ang_two);
        }

        if(ang_one<30){
            if(debugMode>=1)System.out.format(" --> guess one\n");
            return guess_one;
        }else{
            if(debugMode>=1)System.out.format(" --> guess two\n");
            return guess_two;
        }

    }


    //------------------------------
    public int Maroc2Anode(int channel) {
    //------------------------------

        // return anode from MAROC channel
        return RICHGeoConstants.anode_map[(channel)%64];
    }

    //------------------------------
    public int Tile2PMT(int tile, int channel) {
    //------------------------------

        // return anode from MAROC channel

        return RICHGeoConstants.tile2pmt[tile-1][(int) (channel-1)/64];
    }


    //------------------------------
    public double get_aerorefi(int ila, int ico){
    //------------------------------
 
        return aero_refi[ila][ico];

    }
    

    //------------------------------
    public int get_LayerNumber(String slay){
    //------------------------------
        int debugMode = 0;
        for (int ila=0; ila<opticlayers.size(); ila++){
            if(opticlayers.get(ila).get_Name().equals(slay)) {
                if(debugMode>=1)System.out.format(" Find layer %s --> %4d \n",slay,ila);
                return ila;
            }
        }  
        return -1;
    }


    //------------------------------
    public RICHLayer get_Layer(String slay){
    //------------------------------
        for (int ila=0; ila<opticlayers.size(); ila++){
            if(opticlayers.get(ila).get_Name().equals(slay)) {
                return opticlayers.get(ila);
            }
        }  
        return null;
    }


    //------------------------------
    public RICHLayer get_Layer(int ilay){ 
    //------------------------------
        if(ilay>-1 && ilay<NLAY) return opticlayers.get(ilay);
        return null; 
    }


    //------------------------------
    public RICHComponent get_RICHFactory_Component(int idlay, int ico){ 
    //------------------------------
        if(idlay==401) return new RICHComponent(ico, idlay, 1, richfactory.GetPhotocatode(ico+1));
        if(idlay==201 || idlay==202 || idlay==203 || idlay==204 || idlay==301 || idlay==302) 
                  return new RICHComponent(ico, idlay, 1, richfactory.getStlComponent(idlay, ico));

        return null;
    }


    //------------------------------
    public int get_RICHFactory_Size(int idlay){ 
    //------------------------------
        if(idlay==401) return 391;
        if(idlay==201 || idlay==202 || idlay==203 || idlay==204 || idlay==301 || idlay==302) 
                  {return richfactory.getStlNumber(idlay);}

        return 0;
    }

    //------------------------------
    public RICHComponent get_Component(int ilay, int ico){ 
    //------------------------------
        return opticlayers.get(ilay).get(ico);
    }


    //------------------------------
    public CSG get_CSGVolume(int ilay, int ico){
    //------------------------------
        return opticlayers.get(ilay).get(ico).get_CSGVol();
    }

     //------------------------------
     public ArrayList<CSG> get_CSGLayerVolumes(int ilay){
     //------------------------------
        ArrayList<CSG> vols = new ArrayList<CSG>();
        RICHLayer layer = opticlayers.get(ilay);
        for (int ico=0; ico<layer.size(); ico++){
            CSG vol = get_CSGVolume(ilay, ico);
            if(vol!=null)vols.add(vol);
        }  
        return vols;
     }

     //------------------------------
     public G4Stl get_StlVolume(int ilay, int ico){
     //------------------------------
        RICHComponent compo = opticlayers.get(ilay).get(ico);
        if(compo.get_voltype()==2) return compo.get_StlVol();
        return null;
     }

     //------------------------------
     public ArrayList<G4Stl> get_StlLayerVolumes(int ilay){
     //------------------------------
        ArrayList<G4Stl> vols = new ArrayList<G4Stl>();
        RICHLayer layer = opticlayers.get(ilay);
        for (int ico=0; ico<layer.size(); ico++){
            G4Stl vol = get_StlVolume(ilay, ico);
            if(vol!=null)vols.add(vol);
        }  
        return vols;
     }

     //------------------------------
     public G4Box get_BoxVolume(int ilay, int ico){
     //------------------------------
        RICHComponent compo = opticlayers.get(ilay).get(ico);
        if(compo.get_voltype()==1) return compo.get_BoxVol();
        return null;
     }

     //------------------------------
     public ArrayList<G4Box> get_BoxLayerVolumes(int ilay){
     //------------------------------
        ArrayList<G4Box> vols = new ArrayList<G4Box>();
        RICHLayer layer = opticlayers.get(ilay);
        for (int ico=0; ico<layer.size(); ico++){
            G4Box vol = get_BoxVolume(ilay, ico);
            if(vol!=null)vols.add(vol);
        }  
        return vols;
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

    //------------------------------
    public void translate_Triangle3D(Triangle3D tri, Vector3d shift) {
    //------------------------------

        tri.translateXYZ(shift.x, shift.y, shift.z);

    }

    //------------------------------
    public void translate_Sphere3D(Sphere3D sphere, Vector3D shift) { sphere.translateXYZ(shift.x(), shift.y(), shift.z()); }
    //------------------------------

    //------------------------------
    public void translate_Shape3D(Shape3D shape, Vector3D shift) { shape.translateXYZ(shift.x(), shift.y(), shift.z()); }
    //------------------------------

    //------------------------------
    public void translate_Sphere3D(Sphere3D sphere, Vector3d shift) { sphere.translateXYZ(shift.x, shift.y, shift.z); }
    //------------------------------

    //------------------------------
    public void translate_Shape3D(Shape3D shape, Vector3d shift) { shape.translateXYZ(shift.x, shift.y, shift.z); }
    //------------------------------

    //------------------------------
    public void rotate_Triangle3D(Triangle3D tri, Vector3d angle) {
    //------------------------------

        Vector3d bary = get_Triangle3D_Bary(tri);

        tri.rotateZ(angle.z);
        tri.rotateY(angle.y);
        tri.rotateX(angle.x);

        Vector3d shift = bary.minus( get_Triangle3D_Bary(tri) );
        translate_Triangle3D(tri, shift);

    }


    // ----------------
    public void show_RICH(String name, String head){
    // ----------------

        System.out.format(" -----------------------\n  %s \n ----------------------- \n", name);

        for (int ilay=0; ilay<NLAY; ilay++){
            String ini = head + " "+ ilay;
            RICHLayer layer = get_Layer(ilay);
            if(layer.is_aerogel() || layer.is_mapmt()){
                show_Shape3D(layer.get_GlobalSurf(), null, ini);
                if(layer.is_aerogel()){
                    show_Shape3D(layer.get_TrackingSurf(), null, "AA");
                    for(int ico=0; ico<layer.size(); ico++) System.out.format("HH %4d %4d %s \n", ilay, ico, layer.get_CompoBary(ico).toStringBrief(2));
                }
                if(layer.is_mapmt())show_Shape3D(layer.get_TrackingSurf(), null, "PP");

            }else{
                if(layer.is_spherical_mirror()) show_Shape3D(layer.get_GlobalSurf(), null, ini);
                show_Shape3D(layer.get_TrackingSurf(), null, ini);
            }
        }
      

    }

    // ----------------
    public void show_Triangle3D(Triangle3D tri, String name){
    // ----------------

        if(name!=null) System.out.format(" %s ----------------------- %s \n", name, toString(get_Triangle3D_Bary(tri)));
        System.out.format(" %s %s %s \n", toString(tri.point(0)), toString(tri.point(1)), toString(tri.point(2)));
    }


    // ----------------
    public void show_Shape3D(Shape3D plane, String name, String head){
    // ----------------

        if(name!=null) System.out.format(" %s ----------------------- %s \n", name, toString(get_Shape3D_Bary(plane)));
        for (int ifa=0; ifa<plane.size(); ifa++){
            Face3D f = plane.face(ifa);
            if(head==null){
                System.out.format(" %s %s %s \n", toString(f.point(0)), toString(f.point(1)), toString(f.point(2)));
            }else{
                System.out.format(" %s %s %s %s \n", head, toString(f.point(0)), toString(f.point(1)), toString(f.point(2)));
            }
        }
    }


    // ----------------
    public void show_Sphere3D(Sphere3D sphere, String name, String head){
    // ----------------

        if(name!=null) System.out.format(" %s ----------------------- \n", name);
        if(head==null){
            System.out.format(" %s %7.2f \n", toString(sphere.getCenter()), sphere.getRadius());
        }else{
            System.out.format(" %s %s %7.2f \n", head, toString(sphere.getCenter()), sphere.getRadius());
        }
    }


    //------------------------------
    public Vector3D into_LabFrame(Vector3D vec, RICHFrame frame) {
    //------------------------------

        return into_LabFrame(vec, frame.xref(), frame.yref(), frame.zref());

    }

    //------------------------------
    public Vector3D into_LabFrame(Vector3D vec, Vector3D xref, Vector3D yref, Vector3D zref) {
    //------------------------------

        // decompose each vector/rotation along a ref axis (i.e. angle.x*xref) into three cartesian rotation in the lab system 
        return new Vector3D( vec.z()*zref.x() + vec.y()*yref.x() + vec.x()*xref.x(),
                             vec.z()*zref.y() + vec.y()*yref.y() + vec.x()*xref.y(),
                             vec.z()*zref.z() + vec.y()*yref.z() + vec.x()*xref.z());
    }


    //------------------------------
    public void misalign_Element(Shape3D shape, RICHFrame frame, Vector3D angle, Vector3D shift) {
    //------------------------------

        int debugMode = 0;

        if(shape!=null){

            if(debugMode>=1)System.out.format(" FRAME %s %s %s %s\n",frame.xref().toStringBrief(2),frame.yref().toStringBrief(2),
                                                     frame.zref().toStringBrief(2),frame.bref().toStringBrief(2));
            if(debugMode>=1)show_Shape3D(shape, "BEFORE", null);

            if(angle.mag()>0){
                translate_Shape3D(shape, frame.bref().multiply(-1.0));

                Vector3D ang_lab = into_LabFrame(angle, frame);
                if(debugMode>=1)System.out.format(" ang_lab %s \n", ang_lab.toStringBrief(2));
                shape.rotateZ(ang_lab.z());
                shape.rotateY(ang_lab.y());
                shape.rotateX(ang_lab.x());

                translate_Shape3D(shape, frame.bref());
            }

            if(shift.mag()>0){
                Vector3D shift_lab = into_LabFrame(shift, frame);
                if(debugMode>=1)System.out.format(" shift_lab %s \n", shift_lab.toStringBrief(2));
                translate_Shape3D(shape, shift_lab);
            }

            if(debugMode>=1)show_Shape3D(shape, "AFTER ", null);
        }

    }


    //------------------------------
    public void misalign_Element(Sphere3D sphere, RICHFrame frame, Vector3D angle, Vector3D shift) {
    //------------------------------

        int debugMode = 0;

        if(sphere!=null){

            if(debugMode>=1) System.out.format(" FRAME %s %s %s %s\n",frame.xref().toStringBrief(2),frame.yref().toStringBrief(2),
                                                     frame.zref().toStringBrief(2),frame.bref().toStringBrief(2));
            if(debugMode>=1)show_Sphere3D(sphere, "BEFORE", null);

            if(angle.mag()>0){
                translate_Sphere3D(sphere, frame.bref().multiply(-1.0));

                Vector3D ang_lab = into_LabFrame(angle, frame);
                if(debugMode>=1)System.out.format(" ang_lab %s \n", ang_lab.toStringBrief(2));
                sphere.rotateZ(ang_lab.z());
                sphere.rotateY(ang_lab.y());
                sphere.rotateX(ang_lab.x());

                translate_Sphere3D(sphere, frame.bref());
            }
            
            if(shift.mag()>0){
                Vector3D shift_lab = into_LabFrame(shift, frame);
                if(debugMode>=1)System.out.format(" shift_lab %s \n", shift_lab.toStringBrief(2));
                translate_Sphere3D(sphere, shift_lab);
            }

            if(debugMode>=1)show_Sphere3D(sphere, "AFTER ", null);
        }

    }

    //------------------------------
    public Shape3D copy_Shape3D(Shape3D shape) { 
    //------------------------------

        Shape3D copy = new Shape3D();
        for (int ifa=0; ifa<shape.size(); ifa++){copy.addFace( toTriangle3D(shape.face(ifa)));}
        return copy;

    }

    // ----------------
    public void merge_Shape3D(Shape3D shape, Shape3D other) {
    // ----------------

        for(int ifa=0; ifa<other.size(); ifa++)shape.addFace( other.face(ifa) );

    }


    //------------------------------
    public Vector3d get_Shape3D_Center(Shape3D shape) { return toVector3d(shape.center()); }
    //------------------------------

    
    // ----------------
    public Vector3d get_CSGBary(CSG CSGVol) {
    // ----------------

        /*
        *   Avoid double counting of points  
        */
        int debugMode = 0;
        List<Vector3d> pts = new ArrayList<Vector3d>();
        if(debugMode>=1)System.out.format(" get_CSGBary %d \n", CSGVol.getPolygons().size());

        double cX=0.0;
        double cY=0.0;
        double cZ=0.0;
        double np=0.0;
        int ii=0;
        for (Polygon pol: CSGVol.getPolygons()){
            if(debugMode>=1)System.out.format(" poli  %4d ",ii);
            for (Vertex vert: pol.vertices ){
                Vector3d p = toVector3d(vert);
                int found = 0;
                for(int i=0; i<pts.size(); i++){
                    if(p.distance(pts.get(i))<1.e-3)found=1;
                }

                if(found==0){
                    if(debugMode>=1)System.out.format(" --> New Vertex %s\n",toString(p));
                    pts.add(p);
                    cX += p.x;
                    cY += p.y;
                    cZ += p.z;
                    np += 1;
                }else{
                    if(debugMode>=1)System.out.format(" --> Old Vertex %s\n",toString(p));
                }
                
            }
            ii++;
        } 

        if(np>0)return new Vector3d(cX/np, cY/np, cZ/np);
        return new Vector3d(0., 0., 0.);
    }


    //------------------------------
    public Vector3d get_Shape3D_Bary(Shape3D shape) { 
    //------------------------------
     
        /*
        *   Avoid double counting of points  
        */
        int debugMode = 0;
        List<Vector3d> pts = new ArrayList<Vector3d>();
        if(debugMode>=1)System.out.format(" get_Shape3D_Bary %d \n", shape.size());

        double cX=0.0;
        double cY=0.0;
        double cZ=0.0;
        double np=0.0;
        for (int ifa=0; ifa<shape.size(); ifa++){
            Face3D f = shape.face(ifa);
            if(debugMode>=1)System.out.format(" --> get_face %d \n",ifa);
            for (int ipo=0; ipo<3; ipo++){

                Vector3d p = toVector3d(f.point(ipo));
                int found = 0;
                for(int i=0; i<pts.size(); i++){
                    if(p.distance(pts.get(i))<1.e-3)found=1;
                }

                if(found==0){
                    if(debugMode>=1)System.out.format(" --> New Vertex %s\n",toString(p));
                    pts.add(p);
                    cX += p.x;
                    cY += p.y;
                    cZ += p.z;
                    np += 1;
                }else{
                    if(debugMode>=1)System.out.format(" --> Old Vertex %s\n",toString(p));
                }
                
            }
        } 

        if(np>0)return new Vector3d(cX/np, cY/np, cZ/np);
        return new Vector3d(0., 0., 0.);
    }


    //------------------------------
    public Vector3d get_Triangle3D_Bary(Triangle3D tri) { return toVector3d(tri.center()); }
    //------------------------------


    //------------------------------
    public Vector3d get_Shape3D_Normal(Shape3D shape, int iface) {
    //------------------------------

        Triangle3D face = new Triangle3D(shape.face(iface).point(0), shape.face(iface).point(1), shape.face(iface).point(2));
        Vector3D normal = face.normal();
        return toVector3d(normal).normalized(); 

    }


    //------------------------------
    public Vector3d get_Shape3D_Normal(Shape3D shape) {
    //------------------------------

        Triangle3D face = new Triangle3D(shape.face(0).point(0), shape.face(0).point(1), shape.face(0).point(2));
        Vector3D normal = face.normal();
        return toVector3d(normal).normalized(); 

    }


    //------------------------------
    public Vector3d get_Poly_Normal(Polygon pol) {
    //------------------------------
        Vector3d a = pol.vertices.get(0).pos;
        Vector3d b = pol.vertices.get(1).pos;
        Vector3d c = pol.vertices.get(2).pos;
        Vector3d n = b.minus(a).cross(c.minus(a)).normalized();
        return n;
    }


    //------------------------------
    public Vector3d get_Poly_Bary(Polygon pol) {
    //------------------------------
        Vector3d a = pol.vertices.get(0).pos;
        Vector3d b = pol.vertices.get(1).pos;
        Vector3d c = pol.vertices.get(2).pos;
        Vector3d bary = a.plus(b);
        bary = bary.add(c);
        return bary.dividedBy(3.);
    }

    //------------------------------
    public double get_Poly_Area(Polygon pol) {
    //------------------------------
        Vector3d a = pol.vertices.get(0).pos;
        Vector3d b = pol.vertices.get(1).pos;
        Vector3d c = pol.vertices.get(2).pos;
        Line3D base = new Line3D(toPoint3D(a), toPoint3D(b));
        Line3D h = base.distance( toPoint3D(c));
        return base.length()*h.length()/2;
    }

    
    // ----------------
    public String get_PlaneMirrorSide(RICHComponent compo) {
    // ----------------

        int debugMode = 0;

        Vector3D front   = new Vector3D(-0.42,   0.00,   0.91);
        Vector3D left    = new Vector3D(-0.50,  -0.87,   0.00);
        Vector3D right   = new Vector3D(-0.50,   0.87,   0.00);
        Vector3D bottom  = new Vector3D(-1.00,   0.00,   0.00);

        //ATT: this is before having set the layer components
        Vector3D bary = toVector3D(get_CSGBary( compo.get_CSGVol() ));
        if(debugMode>=1)System.out.format(" compo bary %s \n", toString(bary));

        for (Triangle3D pol: toTriangle3D(compo.get_CSGVol().getPolygons()) ){

            if(debugMode>=1)System.out.format("Test front %7.3f  left %7.3f  right %7.3f  bot %7.3f \n",
                     pol.normal().angle(front), pol.normal().angle(left),
                     pol.normal().angle(right), pol.normal().angle(bottom));

            if(pol.normal().angle(front)<5.e-3){
                 if(bary.x() > -100){
                     return new String("mirror_front_B1");
                 }else{
                     return new String("mirror_front_B2");
                 }
            }
            if(pol.normal().angle(left)<5.e-3){
                 if(bary.x() > -100){
                     return new String("mirror_left_L1");
                 }else{
                     return new String("mirror_left_L2");
                 }
            }
            if(pol.normal().angle(right)<5.e-3){
                 if(bary.x() > -100){
                     return new String("mirror_right_L1");
                 }else{
                     return new String("mirror_right_L2");
                 }
            }
            if(pol.normal().angle(bottom)<5.e-3){
                 return new String("mirror_bottom");
            }
            
        }
        return new String("none");
    }


    // ----------------
    public void dump_Face(Face3D face) {
    // ----------------

            Vector3d p0 = toVector3d( face.point(0) );
            Vector3d p1 = toVector3d( face.point(1) );
            Vector3d p2 = toVector3d( face.point(2) );
            System.out.format(" %8.3f %8.3f %8.3f   %8.3f %8.3f %8.3f  %8.3f %8.3f %8.3f \n",p0.x,p0.y,p0.z,p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
    }


    // ----------------
    public void dump_Polygon(Polygon pol) {
    // ----------------
        for (Vertex vert: pol.vertices ){
            System.out.format(" %7.2f %7.2f %7.2f  ",vert.pos.x,vert.pos.y,vert.pos.z);
        }
        System.out.format(" Norm %7.2f %7.2f %7.2f A %7.2f \n",get_Poly_Normal(pol).x,get_Poly_Normal(pol).y,get_Poly_Normal(pol).z,get_Poly_Area(pol));
    }

    // ----------------
    public void dump_StlComponent(CSG CSGVol) {
    // ----------------

        System.out.format(" ------------------\n");
        System.out.format(" Dump of Stl \n");
        System.out.format(" ------------------\n");
        int ii=0;
        for (Polygon pol: CSGVol.getPolygons()){
            System.out.format("  %4d ",ii);
            for (Vertex vert: pol.vertices ){
                System.out.format(" Vtx %7.2f %7.2f %7.2f  ",vert.pos.x,vert.pos.y,vert.pos.z);
            }
            System.out.format(" Norm %7.2f %7.2f %7.2f A %7.2f \n",get_Poly_Normal(pol).x,get_Poly_Normal(pol).y,get_Poly_Normal(pol).z,get_Poly_Area(pol));
            ii++;
        }

    }


    // ----------------
    public void dump_StlComponent(int ilay, int ico) {
    // ----------------

        System.out.format(" ------------------\n");
        System.out.format(" Dump of Stl %d in layer %d \n", ico, ilay);
        System.out.format(" ------------------\n");
        int ii=0;
        for (Polygon pol: get_CSGVolume(ilay, ico).getPolygons()){
            System.out.format("  %4d ",ii);
            for (Vertex vert: pol.vertices ){
                System.out.format(" Vtx %7.2f %7.2f %7.2f  ",vert.pos.x,vert.pos.y,vert.pos.z);
            }
            System.out.format(" Norm %7.2f %7.2f %7.2f A %7.2f \n",get_Poly_Normal(pol).x,get_Poly_Normal(pol).y,get_Poly_Normal(pol).z,get_Poly_Area(pol));
            ii++;
        }

    }


    // ----------------
    public Vector3d find_intersection_UpperHalf_RICH(Line3D ray){
    // ----------------

        int debugMode = 0;
        RICHIntersection inter = get_Layer("mirror_sphere").find_Entrance(ray, -1);

        if(inter!=null){
            if(debugMode>=1)  System.out.format("find_intersection with SPHERICAL (%d, %d): %s\n",
                 inter.get_layer(), inter.get_component(), inter.get_pos().toStringBrief(2));
            return toVector3d(inter.get_pos());
        }else{
            if(debugMode>=1)  System.out.format("find NO intersection with SPHERICAL \n");
        }

        return null;

    }

    // ----------------
    public Vector3d find_intersection_MAPMT(Line3D ray){
    // ----------------

        int debugMode = 0;

        RICHIntersection inter = get_Layer("mapmts").find_Entrance(ray, -1);

        if(inter!=null){
            if(debugMode>=1)  System.out.format("find_intersection with MAPMT (%d, %d): %s\n",
                 inter.get_layer(), inter.get_component(), inter.get_pos().toStringBrief(2));
            return toVector3d(inter.get_pos());
        }

        return null;
    }


    // ----------------
    public boolean is_Spherical_Mirror (int ilay){
    // ----------------

        if(opticlayers.get(ilay).get_Name().equals("mirror_sphere"))return true;
        return false;  

    }





}
