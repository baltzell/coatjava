package cnuphys.adaptiveSwim;

import cnuphys.adaptiveSwim.geometry.Plane;
import cnuphys.swim.SwimTrajectory;

/**
 * Stopper for swimming to a plane. This differs from the AdaptivePlaneStopper
 * in that it will interpolate the last two points (should be one on each side) 
 * to the plane
 * @author heddle
 *
 */
public class AdaptivePlaneInterpStopper extends AdaptivePlaneStopper {
	
	//for estimating the intersection
	private AdaptiveSwimIntersection _intersection;

	//have we ever crossed
	private boolean _everCrossed;
	
			
	/**
	 * Rho  stopper  (does check max path length)
	 * @param u0           initial state vector
	 * @param sf           the maximum value of the path length in meters
	 * @param targetPlane  the target plane
	 * @param accuracy     the accuracy in meters
	 * @param trajectory   optional swim trajectory (can be null)
	 * @param intersection will hold the intersection estimate
	 */
	public AdaptivePlaneInterpStopper(final double[] u0, final double sf, 
			Plane targetPlane, double accuracy, SwimTrajectory trajectory,
			AdaptiveSwimIntersection intersection) {
		super(u0, sf, targetPlane, accuracy, trajectory);
		_intersection = intersection;
		_intersection.checkSetLeft(u0, targetPlane.distance(u0));
	}


	@Override
	public boolean stopIntegration(double snew, double[] unew) {
		
		double newSignedDist = signedDistance(unew);
		double newAbsDist = Math.abs(newSignedDist);
		
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
		if (((snew > _sf) || (_everCrossed && newAbsDist < _accuracy))) {
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
	 * Have we ever crossed the boundary
	 * @return true if we ever crossed the boundary
	 */
	public boolean everCrossed() {
		return _everCrossed;
	}
	

}