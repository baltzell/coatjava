package cnuphys.adaptiveSwim;

import cnuphys.adaptiveSwim.geometry.AGeometric;
import cnuphys.adaptiveSwim.geometry.Point;

/**
 * This is used to hold information about the intersection with a 
 * boundary, for example the intersection of the plane in a swim-to-plane
 * method
 * @author heddle
 *
 */
public class AdaptiveSwimIntersection {
	
	//closest we get on the "left"
	//the original side is always called "left"
	private Point _left = new Point();
	private double _leftAbsDist = Double.POSITIVE_INFINITY;
	
	//closest we get on the "right"
	private Point _right = new Point();
	private double _rightAbsDist = Double.POSITIVE_INFINITY;


	//The estimate of the xyz intersection
	private Point _intersection = new Point();
	
	//distance from intersection point to object (should be very small)
	private double _distance = Double.NaN;
	
	
	public AdaptiveSwimIntersection() {
	}
	
	/**
	 * See if the next step on the left is closer to the target.
	 * @param u the state vector
	 * @param absDist the absolute distance to the target
	 */
	public void checkSetLeft(double u[], double absDist) {
		if (absDist < _leftAbsDist) {
			_left.set(u[0], u[1], u[2]);
			_leftAbsDist = absDist;
		}
	}
	
	/**
	 * See if the next step on the right is closer to the target.
	 * @param u the state vector
	 * @param absDist the absolute distance to the target
	 */
	public void checkSetRight(double u[], double absDist) {
		if (absDist < _rightAbsDist) {
			_right.set(u[0], u[1], u[2]);
			_rightAbsDist = absDist;
		}
	}
	
	/**
	 * Reset the parameters so the object can be reused in another swim.
	 */
	public void reset() {
		_left.set(Double.NaN, Double.NaN, Double.NaN);
		_right.set(Double.NaN, Double.NaN, Double.NaN);
		_leftAbsDist = Double.POSITIVE_INFINITY;
		_rightAbsDist = Double.POSITIVE_INFINITY;
		_intersection.set(Double.NaN, Double.NaN, Double.NaN);
		_distance = Double.NaN;

	}
	
	/**
	 * Compute and store the interpolation ad distance from the object.
	 * Note that if the intersection is successful, the distance should be zero!
	 * @param geom the object (e.g., plane) that is being intersected.
	 */
	public void computeIntersection(AGeometric geom) {
		geom.interpolate(_right, _left, _intersection);
		_distance = geom.distance(_intersection);
	}
	
	/**
	 * Get the intersection distance. Note that if the intersection 
	 * is successful, the distance should be zero! This assumes
	 * a call to computeIntersection has been made at the end
	 * of a swim.
	 * @return the intersection distance
	 */
	public double getIntersectDistance() {
		return _distance;
	}
	
	/**
     * Get the intersection distance. This assumes
	 * a call to computeIntersection has been made at the end
	 * of a swim.
	 * @return the intersection point
	 */
	public Point getIntersectionPoint() {
		return _intersection;
	}
	
	@Override
	public String toString() {
		String istr = _intersection.toString();
		return String.format("  int: %s  D: %-6.2e", 
				istr, _distance);
	}

}