/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.jlab.service.dc;

import org.apache.commons.math3.util.FastMath;
import org.jlab.clas.swimtools.Swim;
import org.jlab.geom.prim.Point3D;
import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;

/**
 *
 * @author ziegler
 */
class recUtil {

    
    static double[] mcTrackPars(DataEvent event, Swim sw, double Zref) {
        double[] value = new double[6];
         double[] trk = null;
        if (event.hasBank("MC::Particle") == false) {
            return value;
        }
        DataBank bank = event.getBank("MC::Particle");
        
        // fills the arrays corresponding to the variables
        if(bank!=null) {
            value[0] = (double) bank.getFloat("vx", 0);
            value[1] = (double) bank.getFloat("vy", 0);
            value[2] = (double) bank.getFloat("vz", 0);
            value[3] = (double) bank.getFloat("px", 0);
            value[4] = (double) bank.getFloat("py", 0);
            value[5] = (double) bank.getFloat("pz", 0);
            int q = getMCPidCharge( bank.getInt("pid", 0)) ;
        
            double[] swimVal = new double[8];

            sw.SetSwimParameters(value[0], value[1], value[2], value[3], value[4], value[5], q);
            swimVal = sw.SwimToPlaneLab(175.);

            //Point3D rotatedP = tw.rotateToTiltedCoordSys(new Point3D(px, py, pz));
            Point3D rotatedP = rotateToTiltedCoordSys(new Point3D(swimVal[3], swimVal[4], swimVal[5]));
            Point3D rotatedX = rotateToTiltedCoordSys(new Point3D(swimVal[0], swimVal[1], swimVal[2]));

            int sector = getSector(swimVal[0], swimVal[1], swimVal[2]);
            sw.SetSwimParameters(rotatedX.x(), rotatedX.y(), rotatedX.z(), rotatedP.x(), rotatedP.y(), rotatedP.z(), q);
            trk = sw.SwimToPlaneTiltSecSys(sector, Zref); 
        }
       
        return trk;
    }
    
    static Point3D rotateToTiltedCoordSys(Point3D labFramePars){
        double[] XinSec = new double[3];
        double[] XinTiltSec = new double[3];

        int sector = getSector(labFramePars.x(), labFramePars.y(), labFramePars.z());

        if ((sector < 1) || (sector > 6)) {
            return new Point3D(0,0,0);
        }
        if (sector == 1) {
            XinSec[0] = labFramePars.x();
            XinSec[1] = labFramePars.y();
        } else {

            double midPlanePhi = Math.toRadians(60 * (sector - 1));
            double cosPhi = Math.cos(midPlanePhi);
            double sinPhi = Math.sin(midPlanePhi);
            XinSec[0] = cosPhi * labFramePars.x() + sinPhi * labFramePars.y();
            XinSec[1] = -sinPhi * labFramePars.x() + cosPhi * labFramePars.y();
        }

        //z coordinates are the same
        XinSec[2] = labFramePars.z();

        // rotate in tilted sector
        XinTiltSec[2] = XinSec[0] * Math.sin(Math.toRadians(25.)) + XinSec[2] * Math.cos(Math.toRadians(25.));
        XinTiltSec[0] = XinSec[0] * Math.cos(Math.toRadians(25.)) - XinSec[2] * Math.sin(Math.toRadians(25.));
        XinTiltSec[1] = XinSec[1];

        return new Point3D(XinTiltSec[0],XinTiltSec[1],XinTiltSec[2]);
    }
    
    static int getSector(double x, double y, double z) {
        double phi = Math.toDegrees(FastMath.atan2(y, x));
        double ang = phi + 30;
        while (ang < 0) {
            ang += 360;
        }
        int sector = 1 + (int) (ang / 60.);

        if (sector == 7) {
            sector = 6;
        }

        if ((sector < 1) || (sector > 6)) {
            System.err.println("Track sector not found....");
        }
        return sector;
    }

    static int getMCPidCharge(int i) {
        if((int) (i/100)==0 ) {
            return (int) -Math.signum(i);
        } else {
            return (int) Math.signum(i);
        }
    }
}
