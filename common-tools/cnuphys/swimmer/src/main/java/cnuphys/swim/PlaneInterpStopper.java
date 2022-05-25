package cnuphys.swim;

import cnuphys.adaptiveSwim.AdaptiveSwimIntersection;
import cnuphys.rk4.IStopper;
import cnuphys.swim.util.Plane;

public class PlaneInterpStopper implements IStopper {

	private double _totalPathLength;
	private double _maxS;
	private double _accuracy;
	private double _s; //current path length meters
	private final double _sf; //max pathlength meters
	private final int _dim;   //dimension of our system
	private double[] _u; //current state vector
	
	private SwimTrajectory _trajectory;


	private cnuphys.adaptiveSwim.geometry.Plane _targetPlane;
	
	//is the starting rho bigger or smaller than the target
	protected int _startSign;

	
	//for estimating the intersection
	private AdaptiveSwimIntersection _intersection;

	//have we ever crossed
	private boolean _everCrossed;
	
	/**
	 * Plane  stopper  (does check max path length)
	 * @param u0           initial state vector
	 * @param sf           the maximum value of the path length in meters
	 * @param targetPlane  the target plane
	 * @param accuracy     the accuracy in meters
	 * @param trajectory   optional swim trajectory (can be null)
	 * @param intersection will hold the intersection estimate
	 */
	public PlaneInterpStopper(final double[] u0, final double sf, 
			cnuphys.adaptiveSwim.geometry.Plane targetPlane, double accuracy, SwimTrajectory trajectory,
			AdaptiveSwimIntersection intersection) {
		
		_dim = u0.length;
		_s = 0;
		_sf = sf;
		_accuracy = accuracy;
		_u = new double[_dim];
		copy(u0, _u);
		_trajectory = trajectory;
		
        if (_trajectory != null) {
        	_trajectory.add(_u, 0);
        }

        _targetPlane = targetPlane;
		_startSign = sign(signedDistance(u0));
		_intersection = intersection;
		_intersection.checkSetLeft(u0, targetPlane.distance(u0));
	}
	
	/**
	 * Get the signed distance from a state vector to a point on the target plane
	 * @param u the state vector
	 * @return the signed distance from a state vector to a point on the target plane in meters
	 */
	private double signedDistance(double u[]) {
		return _targetPlane.signedDistance(u[0], u[1], u[2]);
	}


	@Override
	public boolean stopIntegration(double snew, double[] unew) {
		
		double newSignedDist = signedDistance(unew);
		double newAbsDist = Math.abs(newSignedDist);

		_totalPathLength = snew;
		
		//crossed this time?
		boolean crossed = (sign(newSignedDist) != _startSign);

		if (crossed) {
			_intersection.checkSetRight(unew, newAbsDist);
		}
		else {
			_intersection.checkSetLeft(unew, newAbsDist);
		}
				
		// have we ever crossed the boundary?
		_everCrossed = _everCrossed || crossed;

		// if within accuracy and have ever crossed  or exceeded smax we are done
		if (((snew > _sf) || (_everCrossed && (newAbsDist < _accuracy)))) {
			accept(snew, unew);
  			return true;
		}

        // not within accuracy and still on start side, accept the point.
		//if we are on the right side, we do not accept the point
		if (!crossed) {
			accept(snew, unew);
		}

		return false;
	}
	
	/**
	 * Get the current state vector
	 * @return the current state vector
	 */
	public double[] getU() {
		return _u;
	}

	
	/**
	 * Have we ever crossed the boundary
	 * @return true if we ever crossed the boundary
	 */
	public boolean everCrossed() {
		return _everCrossed;
	}
	
	//get the sign based on the signed distance
	protected int sign(double dist) {
		return ((dist < 0) ? -1 : 1);
	}
	
	/**
	 * Accept a new integration step
	 * @param snew the new value of s in meters
	 * @param unew the new state vector
	 */
	protected void accept(double snew, double[] unew) {
        copy(unew, _u);
        _s = snew;
        
        //add to trajectory?
        if (_trajectory != null) {
        	_trajectory.add(_u, _s);
        }
	}
	
	/**
	 * Get the current path length
	 * @return the current path length in meters
	 */
	public double getS() {
		return _s;
	}
	
	/**
	 * Get the max or final value of the path length in meters
	 * @return the max or final value of the path length
	 */
	public double getSmax() {
		return _sf;
	}
	

	@Override
	public double getFinalT() {
		return _totalPathLength;
	}

	@Override
	public void setFinalT(double finalT) {
//do nothing
	}

	/**
	 * Generally this is the same as stop integration. So most will just return
	 * stopIntegration().
	 * 
	 * @param t the current value of the independent variable (typically pathlength)
	 * @param y the current state vector (typically [x, y, z, vx, vy, vz])
	 * @return <code>true</code> if we should stop now.
	 */
	@Override
	public boolean terminateIntegration(double t, double y[]) {
		System.err.println("terminateIntegration should not be called in PlaneInterpStopper");
		return stopIntegration(t, y);
	}
	
	/**
	 * Copy a state vector
	 * @param uSrc the source
	 * @param uDest the destination
	 */
	public void copy(double uSrc[], double[] uDest) {
		System.arraycopy(uSrc, 0, uDest, 0, _dim);
	}

}
