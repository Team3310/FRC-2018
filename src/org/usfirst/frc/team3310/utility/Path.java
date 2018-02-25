package org.usfirst.frc.team3310.utility;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.usfirst.frc.team3310.utility.Path.Waypoint;
import org.usfirst.frc.team3310.utility.geom.AffineTransform2D;
import org.usfirst.frc.team3310.utility.geom.Point2D;
import org.usfirst.frc.team3310.utility.geom.StraightLine2D;

/**
 * A Path is a recording of the path that the robot takes. Path objects consist
 * of a List of Waypoints that the robot passes by. Using multiple Waypoints in
 * a Path object and the robot's current speed, the code can extrapolate future
 * Waypoints and predict the robot's motion. It can also dictate the robot's
 * motion along the set path.
 */
public class Path {
    protected static final double kSegmentCompletePercentage = .99;
    protected static final double kPointsPerUnit = 0.04;

    protected List<Waypoint> mWaypoints;
    protected List<PathSegment> mSegments;
    protected Set<String> mMarkersCrossed;

    /**
     * A point along the Path, which consists of the location, the speed, and a
     * string marker (that future code can identify). Paths consist of a List of
     * Waypoints.  A radius can be specified to add a fillet to neighboring segments.
     */
    public static class Waypoint {
        public final Translation2d position;
        public final double speed;
        public final Double radius;
        public final String marker;
 
        public Waypoint(Translation2d position, double speed) {
            this.position = position;
            this.speed = speed;
            this.radius = null;
            this.marker = null;
        }

        public Waypoint(Translation2d position, double speed, double radius) {
            this.position = position;
            this.speed = speed;
            this.radius = radius;
            this.marker = null;
        }

        public Waypoint(Translation2d position, double speed, String marker) {
            this.position = position;
            this.speed = speed;
            this.radius = null;
            this.marker = marker;
        }

        public Waypoint(Translation2d position, double speed, double radius, String marker) {
            this.position = position;
            this.speed = speed;
            this.radius = radius;
            this.marker = marker;
        }
    }

    public Path(List<Waypoint> waypoints) {
    	waypoints = processFillets(waypoints);
        mMarkersCrossed = new HashSet<String>();
        mWaypoints = waypoints;
        mSegments = new ArrayList<PathSegment>();
        for (int i = 0; i < waypoints.size() - 1; ++i) {
            mSegments.add(
                    new PathSegment(waypoints.get(i).position, waypoints.get(i + 1).position, waypoints.get(i).speed));
        }
        // The first waypoint is already complete
        if (mWaypoints.size() > 0) {
            Waypoint first_waypoint = mWaypoints.get(0);
            if (first_waypoint.marker != null) {
                mMarkersCrossed.add(first_waypoint.marker);
            }
            mWaypoints.remove(0);
        }
    }
    
    public static List<Waypoint> processFillets(List<Waypoint> waypoints) {
        if (waypoints == null || waypoints.size() == 0) {
        	return waypoints;
        }

        // Check for any radius values 
    	boolean isRadius = false;
        for (int i = 1; i < waypoints.size() - 1; i++) {
            if (waypoints.get(i).radius != null) {
            	isRadius = true;
            	break;
            }
        }		
        if (!isRadius) {
        	return waypoints;
        }

        // Create fillets
        List<Waypoint> filletedWayPoints = new ArrayList<Waypoint>();
        filletedWayPoints.add(waypoints.get(0));
        for (int i = 1; i < waypoints.size(); i++) {
            if (waypoints.get(i).radius != null) {
            	addFillet(filletedWayPoints, waypoints, i);
            }
            else {
                filletedWayPoints.add(waypoints.get(i));
            }
        }	
        
        for (int i = 0; i < filletedWayPoints.size(); i++) {
        	transformPoint(filletedWayPoints.get(i));
        }
        
    	return filletedWayPoints;
    }
    
    private static void addFillet(List<Waypoint> filletedWayPoints, List<Waypoint> waypoints, int index) {
    	if (index == waypoints.size() - 1) {
    		System.out.println("Can't have a radius on the last point");
    		return;
    	}
    	
    	Waypoint currentWaypoint = waypoints.get(index);
    	
    	Translation2d previous = filletedWayPoints.get(filletedWayPoints.size() - 1).position;
    	Translation2d current = currentWaypoint.position;
    	Translation2d next = waypoints.get(index + 1).position;
    	
    	StraightLine2D firstLine = new StraightLine2D(previous.x_, previous.y_, current.x_ - previous.x_, current.y_ - previous.y_); 
    	StraightLine2D secondLine = new StraightLine2D(current.x_, current.y_, next.x_ - current.x_, next.y_ - current.y_); 
    	
    	double deltaAngle = firstLine.horizontalAngle() - secondLine.horizontalAngle();
    	StraightLine2D firstLineOffset = firstLine.parallel(Math.copySign(currentWaypoint.radius, deltaAngle));
    	StraightLine2D secondLineOffset = secondLine.parallel(Math.copySign(currentWaypoint.radius, deltaAngle));
    	
    	Point2D center = firstLineOffset.intersection(secondLineOffset);
    	Point2D startArc = firstLine.projectedPoint(center);
    	
    	filletedWayPoints.add(new Waypoint(new Translation2d(startArc.x(), startArc.y()), currentWaypoint.speed, currentWaypoint.marker));
//    	int numPoints = (int)Math.abs(kPointsPerUnit * (Math.PI - Math.abs(deltaAngle)) * (2 * Math.PI * currentWaypoint.radius));
//    	double angleInc = Math.copySign((Math.PI - Math.abs(deltaAngle)) / numPoints, -deltaAngle);
    	int numPoints = (int)Math.abs(kPointsPerUnit * (Math.abs(deltaAngle)) * (2 * Math.PI * currentWaypoint.radius));
    	double angleInc = Math.copySign((Math.abs(deltaAngle)) / numPoints, -deltaAngle);
    	
    	System.out.println("Delta Angle = " + Math.toDegrees(deltaAngle) + ", PI - delta Angle = " + Math.toDegrees((Math.PI - Math.abs(deltaAngle))));
    	
    	StraightLine2D startLine = new StraightLine2D(center, startArc);

    	double currentAngle = angleInc;
    	for (int i = 0; i < numPoints; i++) {
        	AffineTransform2D xform = AffineTransform2D.createRotation(center, currentAngle);
			StraightLine2D xformedLine = startLine.transform(xform);
			Point2D newPoint = xformedLine.point(1);
	    	filletedWayPoints.add(new Waypoint(new Translation2d(newPoint.x(), newPoint.y()), currentWaypoint.speed));
        	currentAngle += angleInc;  
		}
    	
    	return;
    }
    
    public static Waypoint transformPoint(Waypoint waypoint) {
    	Point2D point = new Point2D(waypoint.position.x_, waypoint.position.y_);
    	Point2D rotatedPoint = point.rotate(-Math.PI/2);
    	waypoint.position.setX(rotatedPoint.getX());
    	waypoint.position.setY(-rotatedPoint.getY());
    	
    	return waypoint;
    }

     /**
     * @param An
     *            initial position
     * @return Returns the distance from the position to the first point on the
     *         path
     */
    public double update(Translation2d position) {
        double rv = 0.0;
        for (Iterator<PathSegment> it = mSegments.iterator(); it.hasNext();) {
            PathSegment segment = it.next();
            PathSegment.ClosestPointReport closest_point_report = segment.getClosestPoint(position);
            if (closest_point_report.index >= kSegmentCompletePercentage) {
                it.remove();
                if (mWaypoints.size() > 0) {
                    Waypoint waypoint = mWaypoints.get(0);
                    if (waypoint.marker != null) {
                        mMarkersCrossed.add(waypoint.marker);
                    }
                    mWaypoints.remove(0);
                }
            } else {
                if (closest_point_report.index > 0.0) {
                    // Can shorten this segment
                    segment.updateStart(closest_point_report.closest_point);
                }
                // We are done
                rv = closest_point_report.distance;
                // ...unless the next segment is closer now
                if (it.hasNext()) {
                    PathSegment next = it.next();
                    PathSegment.ClosestPointReport next_closest_point_report = next.getClosestPoint(position);
                    if (next_closest_point_report.index > 0
                            && next_closest_point_report.index < kSegmentCompletePercentage
                            && next_closest_point_report.distance < rv) {
                        next.updateStart(next_closest_point_report.closest_point);
                        rv = next_closest_point_report.distance;
                        mSegments.remove(0);
                        if (mWaypoints.size() > 0) {
                            Waypoint waypoint = mWaypoints.get(0);
                            if (waypoint.marker != null) {
                                mMarkersCrossed.add(waypoint.marker);
                            }
                            mWaypoints.remove(0);
                        }
                    }
                }
                break;
            }
        }
        return rv;
    }

    public Set<String> getMarkersCrossed() {
        return mMarkersCrossed;
    }

    public double getRemainingLength() {
        double length = 0.0;
        for (int i = 0; i < mSegments.size(); ++i) {
            length += mSegments.get(i).getLength();
        }
        return length;
    }

    /**
     * @param The
     *            robot's current position
     * @param A
     *            specified distance to predict a future waypoint
     * @return A segment of the robot's predicted motion with start/end points
     *         and speed.
     */
    public PathSegment.Sample getLookaheadPoint(Translation2d position, double lookahead_distance) {
        if (mSegments.size() == 0) {
            return new PathSegment.Sample(new Translation2d(), 0);
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        Translation2d position_inverse = position.inverse();
        if (position_inverse.translateBy(mSegments.get(0).getStart()).norm() >= lookahead_distance) {
            // Special case: Before the first point, so just return the first
            // point.
            return new PathSegment.Sample(mSegments.get(0).getStart(), mSegments.get(0).getSpeed());
        }
        for (int i = 0; i < mSegments.size(); ++i) {
            PathSegment segment = mSegments.get(i);
            double distance = position_inverse.translateBy(segment.getEnd()).norm();
            if (distance >= lookahead_distance) {
                // This segment contains the lookahead point
                Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(segment, position,
                        lookahead_distance);
                if (intersection_point.isPresent()) {
                    return new PathSegment.Sample(intersection_point.get(), segment.getSpeed());
                } else {
                    System.out.println("ERROR: No intersection point?");
                }
            }
        }
        // Special case: After the last point, so extrapolate forward.
        PathSegment last_segment = mSegments.get(mSegments.size() - 1);
        PathSegment new_last_segment = new PathSegment(last_segment.getStart(), last_segment.interpolate(10000),
                last_segment.getSpeed());
        Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(new_last_segment, position,
                lookahead_distance);
        if (intersection_point.isPresent()) {
            return new PathSegment.Sample(intersection_point.get(), last_segment.getSpeed());
        } else {
            System.out.println("ERROR: No intersection point anywhere on line?");
            return new PathSegment.Sample(last_segment.getEnd(), last_segment.getSpeed());
        }
    }

    static Optional<Translation2d> getFirstCircleSegmentIntersection(PathSegment segment, Translation2d center,
            double radius) {
        double x1 = segment.getStart().getX() - center.getX();
        double y1 = segment.getStart().getY() - center.getY();
        double x2 = segment.getEnd().getX() - center.getX();
        double y2 = segment.getEnd().getY() - center.getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr_squared = dx * dx + dy * dy;
        double det = x1 * y2 - x2 * y1;

        double discriminant = dr_squared * radius * radius - det * det;
        if (discriminant < 0) {
            // No intersection
            return Optional.empty();
        }

        double sqrt_discriminant = Math.sqrt(discriminant);
        Translation2d pos_solution = new Translation2d(
                (det * dy + (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx + Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());
        Translation2d neg_solution = new Translation2d(
                (det * dy - (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx - Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());

        // Choose the one between start and end that is closest to start
        double pos_dot_product = segment.dotProduct(pos_solution);
        double neg_dot_product = segment.dotProduct(neg_solution);
        if (pos_dot_product < 0 && neg_dot_product >= 0) {
            return Optional.of(neg_solution);
        } else if (pos_dot_product >= 0 && neg_dot_product < 0) {
            return Optional.of(pos_solution);
        } else {
            if (Math.abs(pos_dot_product) <= Math.abs(neg_dot_product)) {
                return Optional.of(pos_solution);
            } else {
                return Optional.of(neg_solution);
            }
        }
    }
    
    public static void addCircleArc(List<Waypoint> waypoints, double radius, double angleDeg, int numPoints, String endMarker ) {
    	Waypoint last = waypoints.get(waypoints.size() - 1);
    	
    	double centerX = last.position.x_;
    	double centerY = radius;
    	
    	double deltaAngle = angleDeg / numPoints;
    	double currentAngle = deltaAngle;
    	for (int i = 0; i < numPoints; i++ ) {
    		double x = radius * Math.sin(Math.toRadians(currentAngle)) + centerX;
    		double y = centerY - radius * Math.cos(Math.toRadians(currentAngle));
    		
    		if (i == numPoints - 1 && endMarker != null) {
    			Waypoint point = new Waypoint(new Translation2d(x, y), last.speed, endMarker);
    			waypoints.add(point);
    		}
    		else {
    			Waypoint point = new Waypoint(new Translation2d(x, y), last.speed);
    			waypoints.add(point);
    		}
    		
    		currentAngle += deltaAngle;
    	}
    }
    
	public static void main(String[] args) {
		
//        List<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 40.0));
//        waypoints.add(new Waypoint(new Translation2d(-35, 0), 40.0));
//        Path.addCircleArc(waypoints, 30.0, -45.0, 10, "hopperSensorOn");
//        waypoints.add(new Waypoint(new Translation2d(-85, 30), 40.0));
        
        List<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 40.0));
//        waypoints.add(new Waypoint(new Translation2d(0, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(10, 10), 40.0));
//        waypoints = processFillets(waypoints);
//
//    	System.out.println("Case 1 x");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.x_ );
//        }
//    	System.out.println("");
//    	System.out.println("Case 1 y");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.y_);
//        }
//    	System.out.println("");
//
//        waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 40.0));
//        waypoints.add(new Waypoint(new Translation2d(0, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(-10, 10), 40.0));
//        waypoints = processFillets(waypoints);
//
//    	System.out.println("Case 2 x");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.x_ );
//        }
//    	System.out.println("");
//    	System.out.println("Case 2 y");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.y_);
//        }
//    	System.out.println("");
//    	
//        waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 40.0));
//        waypoints.add(new Waypoint(new Translation2d(0, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(-10, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(-10, 20), 40.0));
//        waypoints = processFillets(waypoints);
//
//    	System.out.println("Case 3 x");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.x_ );
//        }
//    	System.out.println("");
//    	System.out.println("Case 3 y");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.y_);
//        }
//    	System.out.println("");
        
//        Point2D myPoint = new Point2D(10,10);
//        Point2D rotatedPoint = myPoint.rotate(-Math.PI/2);
//        System.out.println(rotatedPoint.toString());
//        
//        Waypoint myWaypoint = new Waypoint(new Translation2d(0, 10), 40.0, 5);
//        transformPoint(myWaypoint);
//        System.out.println("Waypoint(" + myWaypoint.position.x_ + ", " + myWaypoint.position.y_ + ")");
//    	
//        waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 40.0));
//        waypoints.add(new Waypoint(new Translation2d(0, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(10, 10), 40.0, 5));
//        waypoints.add(new Waypoint(new Translation2d(10, 20), 40.0, 2));
//        waypoints.add(new Waypoint(new Translation2d(-10, 20), 40.0));
//        waypoints = processFillets(waypoints);
//
//    	System.out.println("Case 4 x");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.x_ );
//        }
//    	System.out.println("");
//    	System.out.println("Case 4 y");
//        for (int i = 0; i < waypoints.size(); i++) {
//        	Waypoint curPoint = waypoints.get(i);
//        	System.out.println(curPoint.position.y_);
//        }
//    	System.out.println("");
    	
    	System.out.println("Case 4 x");
        waypoints = new ArrayList<>();
		waypoints.add(new Waypoint(new Translation2d(0, 0), 50.0));
		waypoints.add(new Waypoint(new Translation2d(0, -30), 50.0, 40.0));
		waypoints.add(new Waypoint(new Translation2d(100, -90), 50.0, 70.0));
		waypoints.add(new Waypoint(new Translation2d(100, -160), 50.0, 20.0));
		waypoints.add(new Waypoint(new Translation2d(85, -260), 50.0));
		waypoints = processFillets(waypoints);
		
        for (int i = 0; i < waypoints.size(); i++) {
        	Waypoint curPoint = waypoints.get(i);
        	System.out.println(curPoint.position.x_ );
        }
    	System.out.println("");
    	System.out.println("Case 4 y");
        for (int i = 0; i < waypoints.size(); i++) {
        	Waypoint curPoint = waypoints.get(i);
        	System.out.println(curPoint.position.y_);
        }
    	System.out.println("");

	}
}
