package org.jlab.rec.cvt.services;

import java.util.ArrayList;
import java.util.List;

import java.util.Map;
import java.util.HashMap;
import org.jlab.rec.cvt.trajectory.Helix;
import org.jlab.clas.tracking.kalmanfilter.Surface;
import org.jlab.clas.tracking.objects.Strip;
import org.jlab.geom.prim.Plane3D;
import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.rec.cvt.Constants;
import org.jlab.rec.cvt.cluster.Cluster;
import org.jlab.rec.cvt.cross.Cross;
import org.jlab.rec.cvt.hit.FittedHit;
import org.jlab.rec.cvt.track.Seed;
import org.jlab.rec.cvt.track.Track;
import org.jlab.clas.swimtools.Swim;
import org.jlab.geom.prim.Line3D;
import org.jlab.rec.cvt.bmt.BMTType;
import org.jlab.rec.cvt.fit.CosmicFitter;
import org.jlab.rec.cvt.track.StraightTrack;
import org.jlab.rec.cvt.track.StraightTrackSeeder;
import org.jlab.rec.cvt.track.TrackCandListFinder;
import org.jlab.rec.cvt.track.TrackSeeder;
import org.jlab.rec.cvt.track.TrackSeederCA;

import java.util.Collections;
import java.util.Comparator;
import java.util.Map.Entry;
import org.jlab.detector.base.DetectorType;
import org.jlab.geom.prim.Arc3D;

import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.cvt.bmt.BMTGeometry;
import org.jlab.rec.cvt.svt.SVTGeometry;
import org.jlab.rec.cvt.trajectory.Ray;
import org.jlab.rec.cvt.trajectory.TrajectoryFinder;
/**
 * Service to return reconstructed TRACKS
 * format
 *
 * @author ziegler
 *
 */
public class RecUtilities {

    public void CleanupSpuriousCrosses(List<ArrayList<Cross>> crosses, List<Track> trks,
            SVTGeometry SVTGeom) {
        List<Cross> rmCrosses = new ArrayList<Cross>();
        
        for(Cross c : crosses.get(0)) {
//            double z = SVTGeom.toLocal(c.get_Region()*2,
//                                       c.get_Sector(),
//                                       c.get_Point()).z();
//        
//            if(z<-0.1 || z>SVTGeometry.getModuleLength()) {
//                rmCrosses.add(c);
//            }
            if(!SVTGeom.isInFiducial(c.get_Cluster1().get_Layer(), c.get_Sector(), c.get_Point()))
                rmCrosses.add(c);
        }
       
        
        for(int j = 0; j<crosses.get(0).size(); j++) {
            for(Cross c : rmCrosses) {
                if(crosses.get(0).get(j).get_Id()==c.get_Id())
                    crosses.get(0).remove(j);
            }
        } 
        
       
        if(trks!=null && rmCrosses!=null) {
            List<Track> rmTrks = new ArrayList<Track>();
            for(Track t:trks) {
                boolean rmFlag=false;
                for(Cross c: rmCrosses) {
                    if(c!=null && t!=null && c.get_AssociatedTrackID()==t.get_Id())
                        rmFlag=true;
                }
                if(rmFlag==true)
                    rmTrks.add(t);
            }
            trks.removeAll(rmTrks);
        }
    }
    
    public List<Surface> setMeasVecs(Seed trkcand, Swim swim) {
        //Collections.sort(trkcand.get_Crosses());
        List<Surface> KFSites = new ArrayList<Surface>();
        Plane3D pln0 = new Plane3D(new Point3D(Constants.getXb(),Constants.getYb(),Constants.getZoffset()),
        new Vector3D(0,0,1));
        Surface meas0 = new Surface(pln0,new Point3D(Constants.getXb(),Constants.getYb(),0),
        new Point3D(Constants.getXb()-300,Constants.getYb(),0), new Point3D(Constants.getXb()+300,Constants.getYb(),0), Constants.DEFAULTSWIMACC);
        meas0.setSector(0);
        meas0.setLayer(0);
        meas0.setError(1);
        KFSites.add(meas0); 
        // SVT measurements
        for (int i = 0; i < trkcand.get_Clusters().size(); i++) { 
            if(trkcand.get_Clusters().get(i).get_Detector()==DetectorType.BST) {
                int mlayer = trkcand.get_Clusters().get(i).get_Layer();
                Surface meas = trkcand.get_Clusters().get(i).measurement(mlayer);
                if((int)Constants.getLayersUsed().get(meas.getLayer())<1)
                    meas.notUsedInFit=true;
                if(i>0 && KFSites.get(KFSites.size()-1).getLayer()==meas.getLayer())
                    continue;
                KFSites.add(meas);
            }
        }
       
        // adding the BMT
        double hemisp = Math.signum(trkcand.get_Helix().getPointAtRadius(300).y());
        for (int c = 0; c < trkcand.get_Crosses().size(); c++) {
            if (trkcand.get_Crosses().get(c).get_Detector()==DetectorType.BMT) {                
                int mlayer=trkcand.get_Crosses().get(c).get_Cluster1().get_Layer()+6;
                Surface meas = trkcand.get_Crosses().get(c).get_Cluster1().measurement(mlayer);
                meas.hemisphere = hemisp;
                if((int)Constants.getLayersUsed().get(meas.getLayer())<1) {
                    //System.out.println("Exluding layer "+meas.getLayer()+trkcand.get_Crosses().get(c).printInfo());
                    meas.notUsedInFit=true;
                }
                if(c>0 && KFSites.get(KFSites.size()-1).getLayer()==meas.getLayer())
                    continue;
                KFSites.add(meas);
            }
        }
        return KFSites;
    }
    private TrajectoryFinder tf = new TrajectoryFinder();
    
    
    
    // RDV: switch to cluster.mesurement()
    public List<Surface> setMeasVecs(StraightTrack trkcand, 
            SVTGeometry sgeo, BMTGeometry bgeo, Swim swim) {
        //Collections.sort(trkcand.get_Crosses());
        List<Surface> KFSites = new ArrayList<Surface>();
        Plane3D pln0 = new Plane3D(new Point3D(Constants.getXb(),Constants.getYb(),Constants.getZoffset()),
                                    new Vector3D(0,0,1));
        Surface meas0 = new Surface(pln0,new Point3D(0,0,0),
        new Point3D(-300,0,0), new Point3D(300,0,0),Constants.DEFAULTSWIMACC);
        meas0.setSector(0);
        meas0.setLayer(0);
        meas0.setError(1);
        meas0.hemisphere = 1;
        KFSites.add(meas0); 
        Map<Integer, Cluster> clsMap = new HashMap<Integer, Cluster>();
        trkcand.sort(Comparator.comparing(Cross::getY).reversed());
        for (int i = 0; i < trkcand.size(); i++) { //SVT
            if(trkcand.get(i).get_Detector()==DetectorType.BST) {
                List<Cluster> cls = new ArrayList<Cluster>();
                
                int sector   = trkcand.get(i).get_Cluster1().get_Sector();
                int layertop = trkcand.get(i).get_Cluster1().get_Layer();
                int layerbot = trkcand.get(i).get_Cluster2().get_Layer();
                Ray ray = trkcand.get_ray();
                Point3D top    = new Point3D();
                Point3D bottom = new Point3D();
                sgeo.getPlane(layertop, sector).intersection(ray.toLine(), top);
                sgeo.getPlane(layerbot, sector).intersection(ray.toLine(), bottom);
                
                if(top.y()>bottom.y()) {
                    cls.add(trkcand.get(i).get_Cluster1());
                    cls.add(trkcand.get(i).get_Cluster2());
                } else {
                    cls.add(trkcand.get(i).get_Cluster2());
                    cls.add(trkcand.get(i).get_Cluster1());
                }
                for (int j = 0; j < cls.size(); j++) { 
                    int mlayer = cls.get(j).get_Layer();
                    Surface meas = cls.get(j).measurement(mlayer);
                    // set SVT material budget according to track direction
                    if(j==0) meas.setl_over_X0(SVTGeometry.getToverX0());
                    else     meas.setl_over_X0(0);
                    // RDV to be tested
//                    if((int) Constants.getLayersUsed().get(meas.getLayer())<1)
//                        meas.notUsedInFit=true;
                    if(i>0 && KFSites.get(KFSites.size()-1).getLayer()==meas.getLayer())
                        continue;
                    KFSites.add(meas);
                    
                    clsMap.put(KFSites.size()-1, cls.get(j));
                    trkcand.clsMap = clsMap;
                }
            }

            // adding the BMT
            if (trkcand.get(i).get_Detector()==DetectorType.BMT) {
                int layer  = trkcand.get(i).get_Cluster1().get_Layer();
                int sector = trkcand.get(i).get_Cluster1().get_Sector();

                int id = trkcand.get(i).get_Cluster1().get_Id();
                double ce = trkcand.get(i).get_Cluster1().get_Centroid();
                Surface meas = trkcand.get(i).get_Cluster1().measurement(layer);
                meas.hemisphere = Math.signum(trkcand.get(i).get_Point().y());;
                if((int)Constants.getLayersUsed().get(meas.getLayer())<1) {
                    meas.notUsedInFit=true;
                }
                if(i>0 && KFSites.get(KFSites.size()-1).getLayer()==meas.getLayer())
                    continue;
                KFSites.add(meas);
                clsMap.put(KFSites.size()-1, trkcand.get(i).get_Cluster1());
            }
        }
        for(int i = 0; i< KFSites.size(); i++) {
            KFSites.get(i).setLayer(i);
        }
        return KFSites;
    }
    
    public List<Cluster> FindClustersOnTrkNew (List<Cluster> allClusters, List<Cluster> seedCluster, Helix helix, double P, int Q,
            SVTGeometry sgeo, Swim swimmer) { 

        // initialize swimmer starting from the track vertex
        double maxPathLength = 1; 
        swimmer.SetSwimParameters((helix.xdca()+Constants.getXb()) / 10, (helix.ydca()+Constants.getYb()) / 10, helix.get_Z0() / 10, 
                     Math.toDegrees(helix.get_phi_at_dca()), Math.toDegrees(Math.acos(helix.costheta())),
                     P, Q, maxPathLength) ;
        double[] inters = null;

        // load SVT clusters that are in the seed
        Map<Integer,Cluster> clusterMap = new HashMap<>();
        for(Cluster cluster : seedCluster) {
            if(cluster.get_Detector() == DetectorType.BMT)
                continue;
            clusterMap.put(SVTGeometry.getModuleId(cluster.get_Layer(), cluster.get_Sector()), cluster);
        }   
        
        // for each layer
        for (int ilayer = 0; ilayer < SVTGeometry.NLAYERS; ilayer++) {
            int layer = ilayer + 1;

            // identify the sector the track may be going through (this doesn't account for misalignments
            Point3D helixPoint = helix.getPointAtRadius(sgeo.getLayerRadius(layer));
            
            // reinitilize swimmer from last surface
            if(inters!=null) {
                double intersPhi   = Math.atan2(inters[4], inters[3]);
                double intersTheta = Math.acos(inters[5]/Math.sqrt(inters[3]*inters[3]+inters[4]*inters[4]+inters[5]*inters[5]));
                swimmer.SetSwimParameters(inters[0], inters[1], inters[2], Math.toDegrees(intersPhi), Math.toDegrees(intersTheta), 
                        P, Q, maxPathLength) ;
            }
            
            for(int isector=0; isector<SVTGeometry.NSECTORS[ilayer]; isector++) {
                int sector = isector+1;
                
                // check the angle between the trajectory point and the sector 
                // and skip sectors that are too far (more than the sector angular coverage)
                Vector3D n = sgeo.getNormal(layer, sector);
                double deltaPhi = Math.acos(helixPoint.toVector3D().asUnit().dot(n));
                if(Math.abs(deltaPhi)>2*Math.PI/SVTGeometry.NSECTORS[ilayer]) continue;
                
                int key = SVTGeometry.getModuleId(layer, sector);
                
                // calculate trajectory
                Point3D traj = null;
                Point3D   p = sgeo.getModule(layer, sector).origin();
                Point3D pcm = new Point3D(p.x()/10, p.y()/10, p.z()/10);
                inters = swimmer.AdaptiveSwimPlane(pcm.x(), pcm.y(), pcm.z(), n.x(), n.y(), n.z(), Constants.DEFAULTSWIMACC/10);
                if(inters!=null) {
                    traj = new Point3D(inters[0]*10, inters[1]*10, inters[2]*10);
                }
                // if trajectory is valid, look for missing clusters
                if(traj!=null && sgeo.isInFiducial(layer, sector, traj)) {
                    double  doca    = Double.MAX_VALUE;
                    if(clusterMap.containsKey(key)) {
                        Cluster cluster = clusterMap.get(key);
                        doca = cluster.residual(traj);
                    }
                    // loop over all clusters in the same sector and layer that are noy associated to sector track
                    for(Cluster cls : allClusters) {
                        if(cls.get_AssociatedTrackID()==-1 && cls.get_Sector()==sector && cls.get_Layer()==layer) {
                            double clsDoca = cls.residual(traj);
                            // save the ones that have better doca
                            if(Math.abs(clsDoca)<Math.abs(doca) && Math.abs(clsDoca)<cls.get_CentroidError()*5) {
                                clusterMap.replace(key, cls);
                                doca = clsDoca;
                            }                           
                        }
                    }
                }
            }
        }
        // if any lost cluster with doca better than the seed is found, save it
        List<Cluster> clustersOnTrack = new ArrayList<>();
        for(Entry<Integer,Cluster> entry : clusterMap.entrySet()) {
            if(entry.getValue().get_AssociatedTrackID()==-1) clustersOnTrack.add(entry.getValue());
        }
        return clustersOnTrack;
    }
    
    @Deprecated
    public List<Cluster> FindClustersOnTrk (List<Cluster> allClusters, List<Cluster> seedCluster, Helix helix, double P, int Q,
            SVTGeometry sgeo, Swim swimmer) { 
        Map<Integer, Cluster> clusMap = new HashMap<Integer, Cluster>();
        //Map<Integer, Double> stripMap = new HashMap<Integer, Double>();
        Map<Integer, Double> docaMap = new HashMap<Integer, Double>();
        Map<Integer, Point3D> trajMap = new HashMap<Integer, Point3D>();
        int[] Sectors = new int[SVTGeometry.NLAYERS];
        // RDV it is not correct for tilte/shifted geometry
        for (int a = 0; a < Sectors.length; a++) {
            Point3D I = helix.getPointAtRadius(sgeo.getLayerRadius(a+1));
           int sec = sgeo.getSector(a+1, I);   
           Sectors[a] = sec;
        }
        // initialize swimmer starting from the track vertex
        double maxPathLength = 1; 
        swimmer.SetSwimParameters((helix.xdca()+Constants.getXb()) / 10, (helix.ydca()+Constants.getYb()) / 10, helix.get_Z0() / 10, 
                     Math.toDegrees(helix.get_phi_at_dca()), Math.toDegrees(Math.acos(helix.costheta())),
                     P, Q, maxPathLength) ;
        double[] inters = null;
        double     path = 0;
        // SVT
        for (int l = 0; l < SVTGeometry.NLAYERS; l++) {
            // reinitilize swimmer from last surface
            if(inters!=null) {
                double intersPhi   = Math.atan2(inters[4], inters[3]);
                double intersTheta = Math.acos(inters[5]/Math.sqrt(inters[3]*inters[3]+inters[4]*inters[4]+inters[5]*inters[5]));
                swimmer.SetSwimParameters(inters[0], inters[1], inters[2], Math.toDegrees(intersPhi), Math.toDegrees(intersTheta), 
                        P, Q, maxPathLength) ;
            }
            int layer = l + 1;
            int sector = Sectors[l];
            if(sector == -1)
                continue;
            
            Vector3D  n = sgeo.getNormal(layer, sector);
            Point3D   p = sgeo.getModule(layer, sector).origin();
            Point3D pcm = new Point3D(p.x()/10, p.y()/10, p.z()/10);
            inters = swimmer.AdaptiveSwimPlane(pcm.x(), pcm.y(), pcm.z(), n.x(), n.y(), n.z(), 2*Constants.SWIMACCURACYSVT/10);
            if(inters!=null) {
                Point3D trp = new Point3D(inters[0]*10, inters[1]*10, inters[2]*10);
                int nearstp = sgeo.calcNearestStrip(inters[0]*10, inters[1]*10, inters[2]*10, layer, sector);
                //stripMap.put((sector*1000+layer), nearstp);
                docaMap.put((sector*1000+layer), sgeo.getDoca(layer, sector, nearstp, trp)); 
                trajMap.put((sector*1000+layer), trp); 
            }
        }
        for(Cluster cls : seedCluster) {
            if(cls.get_Detector() == DetectorType.BMT)
                continue;
            int clsKey = cls.get_Sector()*1000+cls.get_Layer();
            clusMap.put(clsKey, cls);
        }
        for(Cluster cls : allClusters) {
            if(cls.get_Detector() == DetectorType.BMT)
                continue;
            int clsKey = cls.get_Sector()*1000+cls.get_Layer();
            if(cls.get_AssociatedTrackID()==-1 && trajMap!=null && trajMap.get(clsKey)!=null) {
                //double trjCent = stripMap.get(clsKey);
                double clsDoca = cls.residual(trajMap.get(clsKey));
                if(clusMap.containsKey(clsKey)) {
                    //double filldCent = clusMap.get(clsKey).get_Centroid();
                    double filldDoca = docaMap.get(clsKey);
                    if(Math.abs(clsDoca)<Math.abs(filldDoca)) {//closer doca
                        clusMap.put(clsKey, cls); //fill it
                    }
                }
                if(Math.abs(clsDoca)<cls.get_CentroidError()*5){ //5sigma cut
                    clusMap.put(clsKey, cls);
                }
            }
        }
        List<Cluster> clustersOnTrack = new ArrayList<Cluster>();
        for(Cluster cl : clusMap.values()) {
            clustersOnTrack.add(cl);
        }
        // RDV can lead to duplicates
        for(Cluster cls : seedCluster) {
            if(cls.get_Detector() == DetectorType.BMT)
                clustersOnTrack.add(cls);
        }
        return clustersOnTrack;
    }
    
    public void MatchTrack2Traj(Seed trkcand, Map<Integer, 
            org.jlab.clas.tracking.kalmanfilter.helical.KFitter.HitOnTrack> traj, 
            SVTGeometry sgeo, BMTGeometry bgeo) {
        
        for (int i = 0; i < trkcand.get_Clusters().size(); i++) { //SVT
            if(trkcand.get_Clusters().get(i).get_Detector()==DetectorType.BST) {
                Cluster cluster = trkcand.get_Clusters().get(i);
                int layer  = trkcand.get_Clusters().get(i).get_Layer();
                int sector = trkcand.get_Clusters().get(i).get_Sector();
                Point3D p = new Point3D(traj.get(layer).x, traj.get(layer).y, traj.get(layer).z);
                cluster.set_CentroidResidual(traj.get(layer).resi);
                cluster.set_SeedResidual(p);             
                for (FittedHit hit : cluster) {
                    double doca1 = hit.residual(p);
                    double sigma1 = sgeo.getSingleStripResolution(layer, hit.get_Strip().get_Strip(), traj.get(layer).z);
                    hit.set_stripResolutionAtDoca(sigma1);
                    hit.set_docaToTrk(doca1);  
                    if(traj.get(layer).isMeasUsed)
                        hit.set_TrkgStatus(1);
                }
            }
        }

        // adding the cross infos
        for (int c = 0; c < trkcand.get_Crosses().size(); c++) {
            if (trkcand.get_Crosses().get(c).get_Detector()==DetectorType.BST) {
                int  layer = trkcand.get_Crosses().get(c).get_Cluster1().get_Layer();
                Vector3D d = new Vector3D(traj.get(layer).px, traj.get(layer).py, traj.get(layer).pz).asUnit();
                trkcand.get_Crosses().get(c).setSVTCrossPosition(d, sgeo);
            }
            if (trkcand.get_Crosses().get(c).get_Detector()==DetectorType.BMT) {
                // update cross position
                int layer = trkcand.get_Crosses().get(c).get_Cluster1().get_Layer()+6;
                Point3D  p = new Point3D(traj.get(layer).x, traj.get(layer).y, traj.get(layer).z);
                Vector3D v = new Vector3D(traj.get(layer).px, traj.get(layer).py, traj.get(layer).pz).asUnit();
                trkcand.get_Crosses().get(c).setBMTCrossPosition(p);
                trkcand.get_Crosses().get(c).set_Dir(v); 
                Cluster cluster = trkcand.get_Crosses().get(c).get_Cluster1();
                if (trkcand.get_Crosses().get(c).get_Type()==BMTType.Z) {
                    cluster.set_CentroidResidual(traj.get(layer).resi*cluster.getTile().baseArc().radius());
                }
                else if (trkcand.get_Crosses().get(c).get_Type()==BMTType.C) {
                    cluster.set_CentroidResidual(traj.get(layer).resi);
                    cluster.set_SeedResidual(p); 
                }
                for (FittedHit hit : cluster) {
                    hit.set_docaToTrk(hit.residual(p));
                    if(traj.get(layer).isMeasUsed) hit.set_TrkgStatus(1);
                }
            }
        }
    }
    
    public Track OutputTrack(Seed seed, org.jlab.clas.tracking.kalmanfilter.helical.KFitter kf,
            SVTGeometry SVTGeom, BMTGeometry BMTGeom) {
        org.jlab.rec.cvt.trajectory.Helix helix = new org.jlab.rec.cvt.trajectory.Helix(kf.KFHelix.getD0(), 
                kf.KFHelix.getPhi0(), kf.KFHelix.getOmega(), 
                kf.KFHelix.getZ0(), kf.KFHelix.getTanL());
        helix.B = kf.KFHelix.getB();
        Track cand = new Track(helix);
        cand.setNDF(kf.NDF);
        cand.setChi2(kf.chi2);
        
        for (Cross c : seed.get_Crosses()) {
            if (c.get_Detector()==DetectorType.BST) {
                continue;
            }
        }
        
        this.MatchTrack2Traj(seed, kf.TrjPoints, SVTGeom, BMTGeom);
        cand.addAll(seed.get_Crosses());
        for(Cluster cl : seed.get_Clusters()) {
            
            int layer = cl.get_Layer();
            int sector = cl.get_Sector();
            
            if(cl.get_Detector()==DetectorType.BMT) {
                
                layer = layer + 6;
                
               if(cl.get_Type() == BMTType.Z) {
                   
                Line3D cln = cl.getAxis();
                cl.setN(cln.distance(new Point3D(kf.TrjPoints.get(layer).x,kf.TrjPoints.get(layer).y,kf.TrjPoints.get(layer).z)).direction().asUnit());
                cl.setS(cl.getL().cross(cl.getN()).asUnit());
                 
               }
                
            }
            //double x = kf.TrjPoints.get(layer).x;
            //double y = kf.TrjPoints.get(layer).y;
            //double z = kf.TrjPoints.get(layer).z;
            //double px = kf.TrjPoints.get(layer).px;
            //double py = kf.TrjPoints.get(layer).py;
            //double pz = kf.TrjPoints.get(layer).pz;
            cl.setTrakInters(new Point3D(kf.TrjPoints.get(layer).x,kf.TrjPoints.get(layer).y,kf.TrjPoints.get(layer).z));
        }
        
        return cand;
        
    }
    public Track OutputTrack(StraightTrack seed, org.jlab.clas.tracking.kalmanfilter.helical.KFitter kf,
            SVTGeometry SVTGeom, BMTGeometry BMTGeom) {
        org.jlab.rec.cvt.trajectory.Helix helix = new org.jlab.rec.cvt.trajectory.Helix(kf.KFHelix.getD0(), 
                kf.KFHelix.getPhi0(), kf.KFHelix.getOmega(), 
                kf.KFHelix.getZ0(), kf.KFHelix.getTanL());
        helix.B = kf.KFHelix.getB();
        Track cand = new Track(helix);
        cand.setNDF(kf.NDF);
        cand.setChi2(kf.chi2); 
        cand.addAll(seed); 
        
        return cand;
        
    }
    public Track OutputTrack(Seed seed) {
        
        Track cand = new Track(seed.get_Helix());
        
        for (Cross c : seed.get_Crosses()) {
            if (c.get_Detector()==DetectorType.BST) {
                continue;
            }
        }
        cand.addAll(seed.get_Crosses());
        return cand;
        
    }
    
    public List<Seed> reFit(List<Seed> seedlist,
            SVTGeometry SVTGeom, BMTGeometry BMTGeom,
            Swim swimmer,  StraightTrackSeeder trseed) {
        List<Seed> filtlist = new ArrayList<Seed>();
        if(seedlist==null)
            return filtlist;
        for (Seed bseed : seedlist) {
            if(bseed == null)
                continue;
            List<Seed>  fseeds = this.reFitSeed(bseed, SVTGeom, BMTGeom, trseed);
            if(fseeds!=null) {
                filtlist.addAll(fseeds);
            }
        }
        return filtlist;
    }        
    
    public List<Seed> reFitSeed(Seed bseed, 
            SVTGeometry SVTGeom, BMTGeometry BMTGeom,
            StraightTrackSeeder trseed) {
        
        List<Seed> seedlist = new ArrayList<Seed>();
        List<Cross> refib = new ArrayList<Cross>();
        List<Cross> refi = new ArrayList<Cross>();
        for(Cross c : bseed.get_Crosses()) {
            int layr = 0;
            int layr2 = 0;
            if(c.get_Detector()==DetectorType.BMT) {
                layr = c.getOrderedRegion()+3;
                if((int)Constants.getLayersUsed().get(layr)>0) {
                    c.isInSeed = false;
                    //System.out.println("refit "+c.printInfo());
                    refib.add(c);
                }
            } else {
                layr = c.get_Cluster1().get_Layer();
                layr2 = c.get_Cluster2().get_Layer();
                if((int)Constants.getLayersUsed().get(layr)>0 
                        && (int)Constants.getLayersUsed().get(layr2)>0) {
                    c.setSVTCrossPosition(null, SVTGeom); 
                    c.isInSeed = false;
                    refi.add(c); 
                }
            }
        }
        Collections.sort(refi);
        seedlist.addAll(trseed.findSeed(refi, refib, SVTGeom, BMTGeom, false));
        return seedlist;
    }
    
    public List<Seed> reFit(List<Seed> seedlist,
            SVTGeometry SVTGeom, BMTGeometry BMTGeom,
            Swim swimmer,  TrackSeederCA trseed,  TrackSeeder trseed2) {
        trseed = new TrackSeederCA();
        trseed2 = new TrackSeeder();
        List<Seed> filtlist = new ArrayList<Seed>();
        if(seedlist==null)
            return filtlist;
        for (Seed bseed : seedlist) {
            if(bseed == null)
                continue;
            List<Seed>  fseeds = this.reFitSeed(bseed, SVTGeom, BMTGeom, swimmer, trseed, trseed2);
            if(fseeds!=null) {
                filtlist.addAll(fseeds);
            }
        }
        return filtlist;
    }
    public List<Seed> reFitSeed(Seed bseed, 
            SVTGeometry SVTGeom, BMTGeometry BMTGeom,
            Swim swimmer,  TrackSeederCA trseed,  TrackSeeder trseed2) {
        boolean pass = true;

        List<Seed> seedlist = new ArrayList<Seed>();
        List<Cross> refib = new ArrayList<Cross>();
        List<Cross> refi = new ArrayList<Cross>();
        for(Cross c : bseed.get_Crosses()) {
            int layr = 0;
            int layr2 = 0;
            c.set_AssociatedTrackID(-1);
            if(c.get_Detector()==DetectorType.BMT) {
                layr = c.getOrderedRegion()+3;
                if((int)Constants.getLayersUsed().get(layr)>0) {
                    c.isInSeed = false;
                    refib.add(c);
                }
            } else {
                layr = c.get_Cluster1().get_Layer();
                layr2 = c.get_Cluster2().get_Layer();
                if((int)Constants.getLayersUsed().get(layr)>0 
                        && (int)Constants.getLayersUsed().get(layr2)>0) {
                    c.setSVTCrossPosition(null, SVTGeom);
                    c.isInSeed = false;
                   // System.out.println("refit "+c.printInfo());
                    refi.add(c); 
                }
            }
        }
        Collections.sort(refi);
        seedlist = trseed.findSeed(refi, refib, SVTGeom, BMTGeom, swimmer);
        
        trseed2.unUsedHitsOnly = true;
        seedlist.addAll( trseed2.findSeed(refi, refib, SVTGeom, BMTGeom, swimmer)); 
        
        return seedlist;
    }
    
    public List<StraightTrack> reFit(List<StraightTrack> seedlist, 
            SVTGeometry SVTGeom, CosmicFitter fitTrk,  TrackCandListFinder trkfindr) {
        fitTrk = new CosmicFitter();
        trkfindr = new TrackCandListFinder();
        List<StraightTrack> filtlist = new ArrayList<StraightTrack>();
        if(seedlist==null)
            return filtlist;
        for (StraightTrack bseed : seedlist) {
            if(bseed == null)
                continue;
            List<StraightTrack> fseeds = this.reFitSeed(bseed, SVTGeom, fitTrk, trkfindr);
            if(fseeds!=null) {
                filtlist.addAll(fseeds);
            }
        }
        return filtlist;
    }
    
    
    public List<StraightTrack> reFitSeed(StraightTrack cand, 
            SVTGeometry SVTGeom, CosmicFitter fitTrk,TrackCandListFinder trkfindr) {
        boolean pass = true;

        List<StraightTrack> seedlist = new ArrayList<StraightTrack>();
        List<Cross> refib = new ArrayList<Cross>();
        List<Cross> refi = new ArrayList<Cross>();
        for(Cross c : cand) {
            int layr = 0;
            int layr2 = 0;
            if(c.get_Detector()==DetectorType.BMT) {
                layr = c.getOrderedRegion()+3;
                if((int)Constants.getLayersUsed().get(layr)>0) {
                    c.isInSeed = false;
                //    System.out.println("refit "+c.printInfo());
                    refib.add(c);
                }
            } else {
                layr = c.get_Cluster1().get_Layer();
                layr2 = c.get_Cluster2().get_Layer();
                if((int)Constants.getLayersUsed().get(layr)>0 
                        && (int)Constants.getLayersUsed().get(layr2)>0) {
                    c.setSVTCrossPosition(null, SVTGeom);
                    c.isInSeed = false;
                   // System.out.println("refit "+c.printInfo());
                    refi.add(c); 
                }
            }
        }
        if(refi.size()>=3) {
            TrackCandListFinder.RayMeasurements NewMeasArrays = trkfindr.
                get_RayMeasurementsArrays((ArrayList<Cross>) refi, false, false, true);
            fitTrk.fit(NewMeasArrays._X, NewMeasArrays._Y, NewMeasArrays._Z,
                    NewMeasArrays._Y_prime, NewMeasArrays._ErrRt, 
                    NewMeasArrays._ErrY_prime, NewMeasArrays._ErrZ);
            if(fitTrk.get_ray()!=null) {
                cand = new StraightTrack(fitTrk.get_ray());
                cand.addAll(refi);
                //refit with the SVT included to determine the z profile
                NewMeasArrays = trkfindr.
                get_RayMeasurementsArrays((ArrayList<Cross>) refi, false, false, false);
                fitTrk.fit(NewMeasArrays._X, NewMeasArrays._Y, NewMeasArrays._Z, 
                        NewMeasArrays._Y_prime, NewMeasArrays._ErrRt, NewMeasArrays._ErrY_prime, NewMeasArrays._ErrZ);
                cand = new StraightTrack(fitTrk.get_ray()); 
                cand.addAll(refi);
                seedlist.add(cand);
            }
        }
        return seedlist;
    }
    
    public double[] MCtrackPars(DataEvent event) {
        double[] value = new double[6];
        if (event.hasBank("MC::Particle") == false) {
            return value;
        }
        DataBank bank = event.getBank("MC::Particle");
        
        // fills the arrays corresponding to the variables
        if(bank!=null) {
            value[0] = (double) bank.getFloat("vx", 0)*10;
            value[1] = (double) bank.getFloat("vy", 0)*10;
            value[2] = (double) bank.getFloat("vz", 0)*10;
            value[3] = (double) bank.getFloat("px", 0);
            value[4] = (double) bank.getFloat("py", 0);
            value[5] = (double) bank.getFloat("pz", 0);
        }
        return value;
    }

    
}
