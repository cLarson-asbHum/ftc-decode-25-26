package ballistics;

import java.util.Iterator;

public final class ImmutableBallisticArc implements BallisticArc {
    public final Vector[] points;
    public final Vector[] vels;
    public final double elapsedTime;

    public ImmutableBallisticArc(BallisticArc arc) {
        this.elapsedTime = arc.getElapsedTime();

        // Supplying all the points
        final Iterator<Vector> pointIterator = arc.pointIterator();
        this.points = new Vector[arc.size()];

        for(int i = 0; i < this.points.length && pointIterator.hasNext(); i++) {
            this.points[i] = pointIterator.next();
        }
        
        // Supplying all the velocities
        final Iterator<Vector> velIterator = arc.velIterator();
        this.vels = new Vector[arc.size()];

        for(int i = 0; i < this.vels.length && velIterator.hasNext(); i++) {
            this.vels[i] = velIterator.next();
        }
    }

    @Override
    public int size() {
        return points.length;
    }

    @Override
    public Vector getPoint(int i) {
        return points[i];
    } 

    @Override
    public Vector getVel(int i) {
        return vels[i];
    }

    @Override
    public double getElapsedTime() {
        return this.elapsedTime;
    }

    @Override
    public boolean compute(double unused1, ArcFunction unused2) {
        return false;
    }

    @Override
    public String toString() {
        return BallisticArc.stringify(this);
    }
}