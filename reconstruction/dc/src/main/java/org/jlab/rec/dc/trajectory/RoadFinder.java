package org.jlab.rec.dc.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.jlab.rec.dc.Constants;
import org.jlab.rec.dc.cluster.Cluster;
import org.jlab.rec.dc.cluster.ClusterFitter;
import org.jlab.rec.dc.cluster.FittedCluster;
import org.jlab.rec.dc.hit.FittedHit;
import org.jlab.rec.dc.segment.Segment;

import Jama.Matrix;
import org.jlab.detector.geant4.v2.DCGeant4Factory;



public class RoadFinder  {

    public RoadFinder() {
        
    }
	
	public List<Segment> findPseudoSegList(List<Segment> segs, DCGeant4Factory DcDetector)  {
                
		QuadraticFit qf = new QuadraticFit();
                List<Segment> Roads = new ArrayList<Segment>();
		List<ArrayList<ArrayList<Segment>>> superLayerLists = new ArrayList<ArrayList<ArrayList<Segment>>>();
                
		for(int sec=0; sec<6; sec++)  {
                    ArrayList<ArrayList<Segment>> sLyrs = new ArrayList<ArrayList<Segment>>();
                    ArrayList<ArrayList<ArrayList<Segment>>> rLyrs = new ArrayList<ArrayList<ArrayList<Segment>>>();
                    
                    for(int sly=0; sly<6; sly++) {
                        sLyrs.add(new ArrayList<Segment>());
                    }
                    
                    superLayerLists.add(sLyrs);
		}
                
		//make an array sorted by sector, superlayers
		for(Segment seg: segs) {			
                    if(seg.isOnTrack==false)
			superLayerLists.get(seg.get_Sector()-1).get(seg.get_Superlayer()-1).add((Segment) seg.clone());	
                }
		for(int sec=0; sec<6; sec++)  {
			for(int sly=0; sly<6; sly++) {
				//if(superLayerLists.get(sec).get(sly).size()==0) { // add a blank to each superlayer
                                Segment blank = new Segment(new FittedCluster(new Cluster(sec+1, sly+1, -1)));
                                blank.set_Id(-10);
                                superLayerLists.get(sec).get(sly).add(blank);
				//} 
			}
		}
		
		for(int sec=0; sec<6; sec++)  {
			for(int j =0; j<2; j++) {
				for(int i1 = 0; i1<superLayerLists.get(sec).get(0+j).size(); i1++) {
					Segment s1 = superLayerLists.get(sec).get(0+j).get(i1);
					for(int i2 = 0; i2<superLayerLists.get(sec).get(2+j).size(); i2++) {
						Segment s2 = superLayerLists.get(sec).get(2+j).get(i2);
						for(int i3 = 0; i3<superLayerLists.get(sec).get(4+j).size(); i3++) {
							Segment s3 = superLayerLists.get(sec).get(4+j).get(i3);
							ArrayList<Segment> sLyr = new ArrayList<Segment>();	
                                                        
							if(s1.get_Id()!=-10) {
                                                                sLyr.add(s1);
                                                        }
							if(s2.get_Id()!=-10) {
                                                                sLyr.add(s2);
                                                        }
							if(s3.get_Id()!=-10) {
                                                                sLyr.add(s3);
                                                        }
                                                        if(this.fitRoad(sLyr, qf, DcDetector)==true) {
                                                            if(sLyr.size()==3 && qf.chi2<100) { // road is good --> pass w.out looking for missing segment
                                                                s1.set_Id(-10);
                                                                s2.set_Id(-10);
                                                                s3.set_Id(-10);
                                                            }
                                                            if(sLyr.size()>1 && sLyr.size()<3 && qf.chi2<100) {
                                                                Segment pseudoSeg = this.findRoads(sLyr, qf, DcDetector);
                                                                if(pseudoSeg!=null)
                                                                    Roads.add(pseudoSeg);
                                                            }
                                                        }
							
						}
					}
				}
                        }
		}
                return Roads;
        }
	
	private SegmentTrajectory segTrj = new SegmentTrajectory();
	
	private ClusterFitter cf = new ClusterFitter();

	public Segment findRoads(ArrayList<Segment> segList, QuadraticFit qf, DCGeant4Factory DcDetector)  { 
            Segment pseudoSeg = null;
            if(segList.size()<3) { // make pseudo-segment for missing segment
                // find missing segment superlayer
                int s1 = (segList.get(0).get_Superlayer()-(segList.get(0).get_Superlayer()+1)%2-1)/2; // odd superlayers
                int s2 = (segList.get(1).get_Superlayer()-(segList.get(1).get_Superlayer()+1)%2-1)/2; // even superlayers
                int smiss = -1;
                if(s1==0) {
                        if(s2==1)
                                smiss =2;
                        if(s2==2)
                                smiss =1;
                } else {
                        smiss =0;
                }

                int slyr = (segList.get(0).get_Superlayer()+1)%2+2*smiss+1;
                // make the missing segment
                Cluster pseudoCluster = new Cluster(segList.get(0).get_Sector(),slyr,-1);
                        FittedCluster fpseudoCluster = new FittedCluster(pseudoCluster);
                for(int l = 0; l<6; l++) {
                        int layer = l+1;
                        double z = DcDetector.getWireMidpoint(slyr-1,layer-1,0).z;
                        double trkX = qf.a[0]*z*z+qf.a[1]*z+qf.a[2]; 
                        int calcWire = segTrj.getWireOnTrajectory(slyr, layer, trkX, DcDetector) ;
                        FittedHit pseudoHit = new FittedHit(segList.get(0).get_Sector(),slyr, layer, calcWire,
                                        0, -1); 
                        pseudoHit.calc_CellSize(DcDetector);
                        pseudoHit.set_DocaErr(pseudoHit.get_CellSize()/Math.sqrt(12.));

                        pseudoHit.updateHitPosition(DcDetector);
                        fpseudoCluster.add(pseudoHit);
                }

                cf.SetFitArray(fpseudoCluster, "TSC");
                cf.Fit(fpseudoCluster, true);

                cf.SetSegmentLineParameters(fpseudoCluster.get(0).get_Z(), fpseudoCluster) ;
                pseudoSeg = new Segment(fpseudoCluster); 
                pseudoSeg.set_fitPlane(DcDetector);	
            }
            return pseudoSeg;
	}

	private Segment reFit(Segment pseudoSeg, ArrayList<Segment> segList, DCGeant4Factory DcDetector ) {
		QuadraticFit qf = new QuadraticFit();
		this.fitRoad(segList, qf, DcDetector);
		
		Cluster pseudoCluster = new Cluster(segList.get(0).get_Sector(),pseudoSeg.get_Superlayer(),-1);
		FittedCluster fpseudoCluster = new FittedCluster(pseudoCluster);
		
		System.out.println(segList.size()+" pseudoSeg "+pseudoSeg.printInfo()+"   "+pseudoSeg.get(0).get_Wire());
		
		for(int l = 0; l<6; l++) {
    		int layer = l+1;
    		double z = DcDetector.getWireMidpoint(pseudoSeg.get_Superlayer()-1,layer-1,0).z;
    		double trkX = qf.a[0]*z*z+qf.a[1]*z+qf.a[2]; 
    		double delta = (trkX-pseudoSeg.get(l).get_X())/pseudoSeg.get(l).get_CellSize()/Math.cos(Math.toRadians(6.)) ;
    		int calcWire = segTrj.getWireOnTrajectory(pseudoSeg.get_Superlayer(), layer, trkX, DcDetector);
    		
    		FittedHit pseudoHit = new FittedHit(segList.get(0).get_Sector(),pseudoSeg.get_Superlayer(), layer, calcWire,
    				0, -1); 
    		pseudoHit.set_DocaErr(pseudoHit.get_CellSize()/Math.sqrt(12.)/Math.cos(Math.toRadians(6.)));
    		pseudoHit.updateHitPosition(DcDetector);
    		fpseudoCluster.add(pseudoHit);
    		System.out.println((int)Math.round(delta)+" calcW "+calcWire);
    	}
    	System.out.println("Chi2 "+qf.chi2);
    	 cf.SetFitArray(fpseudoCluster, "TSC");
         cf.Fit(fpseudoCluster, true);
         
         cf.SetSegmentLineParameters(fpseudoCluster.get(0).get_Z(), fpseudoCluster) ;
         Segment pseudoSeg1 = new Segment(fpseudoCluster);
         
         pseudoSeg1.set_fitPlane(DcDetector);	
		
         return pseudoSeg1;
	}

	private boolean fitRoad(ArrayList<Segment> segList, QuadraticFit qf, DCGeant4Factory DcDetector) {
		
		int NbHits =0;		
		if(segList.size()<2)
			return false;
		
		for(Segment s : segList)
			NbHits+=s.size();
		double[] X = new double[NbHits];
		double[] Z = new double[NbHits];
		double[] errX = new double[NbHits];
		
		int hitno =0; 
		for(Segment s : segList) {
			for(int j =0; j<s.size(); j++) { 
				X[hitno] = s.get(j).get_X();
				//X[hitno] = GeometryLoader.dcDetector.getSector(0).getSuperlayer(s.get(j).get_Superlayer()-1).getLayer(s.get(j).get_Layer()-1).getComponent(s.get(j).get_Wire()-1).getMidpoint().x();
				Z[hitno] = s.get(j).get_Z();
				//errX[hitno] = s.get(j).get_DocaErr()/Math.cos(Math.toRadians(6.)); 
				errX[hitno] = s.get(j).get_CellSize()/Math.sqrt(12.)/Math.cos(Math.toRadians(6.)); 
				hitno++;
			}
		}
		
		qf.evaluate(Z, X, errX);
        
        double WChi2 =0;
        for(Segment s : segList) {
        	for(FittedHit h : s) {
        		double trkX = qf.a[0]*h.get_Z()*h.get_Z()+qf.a[1]*h.get_Z()+qf.a[2]; 
        		int calcWire = segTrj.getWireOnTrajectory(h.get_Superlayer(), h.get_Layer(), trkX, DcDetector) ;
        		WChi2+=(h.get_Wire()-calcWire)*(h.get_Wire()-calcWire);
        	} 
        }
        
        if(WChi2/qf.NDF>1)
        	return false;
        
        return true;
	}

    /**
     * 
     * @param superlayer segment superlayer
     * @return the superlayer in a region in which there should be a match to make a cross; i.e. for a segmenet in superlayer 1 there should be a matched segment in superlayer 2
     */    
    private int SuperlayerInWhichToSearchMatchingSeg(int superlayer) {
        if(superlayer%2==0) { //even layer
            return superlayer-1; 
        } else { //odd layer
            return superlayer+1; 
        }
            
        
      //  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

	private class QuadraticFit {
		public double chi2;
		public double NDF;
		public double[] a;
		
		public void evaluate(double[] x, double[] y, double[] err) {
			
			double[] ret = {0.,0.,0.};
			Matrix A = new Matrix(3,3);
			Matrix V = new Matrix(3,1);
			double sum1 = 0.0;
			double sum2 = 0.0;
			double sum3 = 0.0;
			double sum4 = 0.0;
			double sum5 = 0.0;
			double sum6 = 0.0;
			double sum7 = 0.0;
			double sum8 = 0.0;
			for (int i = 0; i < x.length; ++i) {
				double y1 = y[i];
				double x1 = x[i];
				double x2 = x1 * x1;
				double x3 = x2 * x1;
				double x4 = x2 * x2;
				double e2 = err[i]*err[i];
				sum1 += x4/e2;
				sum2 += x3/e2;
				sum3 += x2/e2;
				sum4 += x1/e2;
				sum5 += 1.0/e2;
				sum6 += y1 * x2/e2;
				sum7 += y1 * x1/e2;
				sum8 += y1/e2;
			}
			A.set(0,0,sum1);
			A.set(0,1,sum2);
			A.set(0,2,sum3);
			A.set(1,0,sum2);
			A.set(1,1,sum3);
			A.set(1,2,sum4);
			A.set(2,0,sum3);
			A.set(2,1,sum4);
			A.set(2,2,sum5);
			V.set(0,0,sum6);
			V.set(1,0,sum7);
			V.set(2,0,sum8);
			Matrix Ainv = A.inverse();
			Matrix X;
		try {
			
			X = Ainv.times(V);
			for (int i = 0; i < 3; ++i) {
				ret[i] = X.get(i, 0);
				//System.out.println(X.get(i, 0));
			}
			double _chi2 =0;
			for (int i = 0; i<x.length; i++) {				
				double tiltSysXterm = ret[0]*x[i]*x[i]+ret[1]*x[i]+ret[2];	
				_chi2+=(tiltSysXterm-y[i])*(tiltSysXterm-y[i])/(err[i]*err[i]);
			}
			this.chi2 = _chi2;
			this.NDF = x.length -3;
		} catch (ArithmeticException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		a = ret;
		
		}
	}
	
}
