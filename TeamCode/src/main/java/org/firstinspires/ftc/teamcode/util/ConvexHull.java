package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collection;
// import java.util.LinkedList;

/**
 * A class that abstracts a convex polygon, including the boundary. 
 * Utilities are provided, such as finding the closest point or testing 
 * whether a circle intersects with it
 */
public class ConvexHull {
    private static Pose leftmost(Pose[] points) {
        Pose leftMost = points[0];
        double leftX = leftMost.getX();

        for(int i = 1; i < points.length; i++) {
            final Pose current = points[i];
            final double curX = current.getX();

            if(curX < leftX || (
                curX == leftX 
                && current.getY() < leftMost.getY()
            )) {
                leftMost = current;
                leftX = curX;
            }
        }

        return leftMost;
    }

    private static void validatePoints(Pose[] points) {
        if(points == null) {
            throw new IllegalArgumentException("Array of points cannot be null");
        }

        if(points.length < 3) {
            throw new IllegalArgumentException("Size of points (" + points.length + ") less than 3");
        }
    }

    private static boolean pointIsLeftToLine(Pose other, Pose line1, Pose line2) {
        final double a = line2.getY() - line1.getY();
        final double b = line2.getX() - line1.getX();
        
        return 
            a * other.getX() - b * other.getY() 
            < a * line1.getX() - b * line1.getY();
    }
    
    private static Pose closestPointOnLine(Pose other, Pose endpoint1, Pose endpoint2) {
        final double x0 = other.getX();
        final double y0 = other.getY();
        final double x1 = endpoint1.getX();
        final double y1 = endpoint1.getY();

        final double a = endpoint2.getY() - y1;
        final double b = endpoint2.getX() - x1;

        final double normalizer = 1 / (a * a + b * b);
        return new Pose(
            (a * a * x1 + a * b * (y0 - y1) + b * b * x0) * normalizer,
            (a * a * y0 + a * b * (x0 - x1) + b * b * y1) * normalizer
        );
    }

    private static Pose closestPointOnSegment(Pose other, Pose endpoint1, Pose endpoint2) {
        final double x0 = other.getX();
        final double y0 = other.getY();
        final double x1 = endpoint1.getX();
        final double y1 = endpoint1.getY();

        final double a = endpoint2.getY() - y1;
        final double b = endpoint2.getX() - x1;

        final double clampedT = Util.clamp(0, (a * (y0 - y1) + b * (x0 - x1)) / (a * a + b * b), 1);
        return new Pose(
            Util.lerp(endpoint1.getX(), clampedT, endpoint2.getX()),
            Util.lerp(endpoint1.getY(), clampedT, endpoint2.getY())
        );
    }

    private static double sqrMagnitude(Pose pose) {
        return pose.distSquared(new Pose(0, 0));
    }
    
    private static double magnitude(Pose pose) {
        return pose.distanceFrom(new Pose(0, 0));
    }

    private static double determinant(Pose vec1, Pose vec2) {
        return vec1.getX() * vec2.getY() - vec1.getY() * vec2.getX();
    }

    /**
     * Gets the convex hull that bounds all the points in the data set.
     * The specific implementation uses the Gift Wrapping algorithm,
     * so expect an O(n^2) runtime.
     * 
     * The only restrictions on the input are that there are 3 or more 
     * points, no 3 points are collinear, and nothing is null. An 
     * IllegalArgumentException is thrown if any of these except 
     * noncollinearity is violated. 
     * 
     * @return The convex hull that contains all the given points. Never null
     */
    public static ConvexHull of(Pose[] points) {
        validatePoints(points);

        // Wrapping around our points
        // The following implementation was made from Wikipedia's pseudocode
        // for the algorithm, from the article "Gift wrapping algorithm"
        final ArrayList<Pose> result = new ArrayList<>();
        Pose pointOnHull = leftmost(points);
        int i = 0;

        final int MAX_ITERS = 1000;
        int iters = 0;
        do {
            result.add(pointOnHull);
            Pose endpoint = points[0]; // Placeholder
                

            // Getting the point with the left-most angle
            for(int j = 0; j < points.length; j++) {
                final Pose other = points[j];
                if(endpoint == pointOnHull || pointIsLeftToLine(other, result.get(i), endpoint)) {
                    endpoint = other;
                }
                iters++;
            }
            pointOnHull = endpoint;
            i++;
        } while(pointOnHull != result.get(0) && iters < MAX_ITERS);


        // Constructing the hull
        return new ConvexHull(result);
    }

    /**
     * Bounds creating the boundary of the convex hull, ordered clockwise
     */
    private final Pose[] boundary;

    // the following fields are used only for AABB collision detection
    private final double halfWidth;
    private final double halfHeight;
    private final double x;
    private final double y;

    private ConvexHull(Pose[] boundary) {
        this.boundary = boundary;
        
        // Finding the corners
        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;

        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for(final Pose point : this.boundary) {
            final double x = point.getX();
            final double y = point.getY();

            if(x < minX) {
                minX = x;
            } else if(x > maxX) {
                maxX = x;
            }
            
            if(y < minY) {
                minY = y;             
            } else if(y > maxY) {
                maxY = y;
            }
        }

        // Setting the appropriate properties
        this.halfWidth = 0.5 * (maxX - minX);
        this.halfHeight = 0.5 * (maxY - minY);
        this.x = minX + this.halfWidth;
        this.y = minY + this.halfHeight;
    }

    private ConvexHull(Collection<Pose> boundary) {
        this(boundary.toArray(new Pose[boundary.size()]));
    }

    public Pose[] getBoundary() {
        return this.boundary;
    }

    public boolean contains(Pose point) {
        // Becuase the points are clockwise, a point on the left of any edge is outside
        // We separately check the only edge where indexOf(endpoint2) - indexOf(endpoint1) < 0
        Pose endpoint1 = boundary[boundary.length - 1];
        Pose endpoint2 = boundary[0];

        if(pointIsLeftToLine(point, endpoint1, endpoint2)) {
            return false;
        }

        // Checking all the other edges
        for(int i = 0; i < boundary.length - 1; i++) {
            // Becuase the points are clockwise, a point on the left is outside
            endpoint1 = boundary[i];
            endpoint2 = boundary[i + 1];

            if(pointIsLeftToLine(point, endpoint1, endpoint2)) {
                return false;
            }
        }

        // We did not fail, so we must've succeeded
        return true;
    }

    public Pose closestPointTo(Pose other) {
        if(this.contains(other)) {
            return other;
        } 

        // Getting the closest edge
        Pose endpoint1 = boundary[boundary.length - 1];
        Pose endpoint2 = boundary[0];
        Pose closestPoint = closestPointOnSegment(other, endpoint1, endpoint2);
        double minDist = sqrMagnitude(closestPoint.minus(other));

        for(int i = 0; i < boundary.length - 1; i++) {
            endpoint1 = boundary[i];
            endpoint2 = boundary[i + 1];
            
            final Pose candidate = closestPointOnSegment(other, endpoint1, endpoint2);
            final double currentDist = sqrMagnitude(candidate.minus(other));
            if(currentDist < minDist) {
                closestPoint = candidate;
            }
        }

        return closestPoint;
    }

    public double distanceTo(Pose point) {
        return magnitude(point.minus(closestPointTo(point)));
    } 

    /**
     * Checks if the two axis-aligned bounding boxes of these hulls intersect.
     * 
     * @param other The other hull to check against
     * @return Whether the two bounds have overlap (edges are excluded)
     */
    public boolean aabb(ConvexHull other) {
        return 
            Math.abs(other.x - this.x) < other.halfWidth + this.halfWidth
            && Math.abs(other.y - this.y) < other.halfHeight + this.halfHeight;
    }

    public boolean intersects(ConvexHull other) {
        // Checking whether these hulls' bounding boxes are even intersecting
        if(!this.aabb(other)) {
            return false;
        }

        // Checking whether any pair of edges intersect
        // Yes, this is O(n^2), but, we're not comparing 100-gons, are we?
        for(int i = 0; i < boundary.length; i++) {
            final Pose vert1 = boundary[i];
            final Pose vert2 = boundary[(i + 1) % boundary.length];

            for(int j = 0; j < boundary.length; j++) {
                final Pose vert3 = boundary[j];
                final Pose vert4 = boundary[(j + 1) % boundary.length];

                final double det = determinant(vert2.minus(vert1), vert3.minus(vert4));
                final double t   = determinant(vert3.minus(vert1), vert3.minus(vert4)) / det;
                final double u   = determinant(vert2.minus(vert1), vert3.minus(vert1)) / det;

                if((0 < t && t < 1) && (0 < u && u < 1)) {
                    return true;
                }
            }
        }

        // If no egdes intersected, check the vertices
        for(final Pose otherVert : other.boundary) {
            if(this.contains(otherVert)) {
                return true;
            }
        }

        for(final Pose thisVert : this.boundary) {
            if(other.contains(thisVert)) {
                return true;
            }
        }

        // Nothing registered as intersecting, so it doesn't
        return false;
    }

    public boolean intersectsCircle(Pose center, double radius) {
        if(this.contains(center)) {
            return true;
        }

        return sqrMagnitude(center.minus(closestPointTo(center))) <= radius * radius;
    } 
}