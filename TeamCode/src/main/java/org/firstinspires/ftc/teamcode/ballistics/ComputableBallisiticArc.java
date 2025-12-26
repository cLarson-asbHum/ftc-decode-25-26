package ballistics;

import java.util.Iterator;
import java.util.LinkedList;

public class ComputableBallisiticArc implements BallisticArc {
    public static double TIMEOUT = 10.0; // 10 seconds worth (or equivalent)

    private final LinkedList<Vector> points = new LinkedList<>();
    private final LinkedList<Vector> vels = new LinkedList<>();
    private final Vector gravity;
    private final double dragScalar;
    private double elapsed = 0;

    private ComputableBallisiticArc(Builder builder) {
        vels.add(builder.getVel());
        points.add(builder.getStart());
        this.gravity = builder.getGravity();
        this.dragScalar = builder.getDragScalar();
    }

    // private ComputableBallisiticArc() {}
    
    public static class Builder {
        private Vector start = new Vector(0,0);
        private Vector vel = new Vector(0, 0);
        private double dragScalar = 1;
        private Vector gravity = new Vector(0, -1);
        
        public Builder setStart(Vector newStart) {
            this.start = newStart;
            return this;
        }

        public Builder setVel(Vector newVelocity) {
            this.vel = newVelocity;
            return this;
        }

        public Builder setVel(double theta, double speed) {
            this.vel = new Vector(Math.cos(theta), Math.sin(theta)).scale(speed);
            return this;
        }

        public Builder setDragScalar(double newDragScalar) {
            this.dragScalar = newDragScalar;
            return this;
        }

        public Builder setGravity(Vector newGravity) {
            this.gravity = newGravity;
            return this;
        }

        public Builder setGravity(double gravity) {
            this.gravity = new Vector(0, gravity);
            return this;
        }

        public Vector getStart() {
            return this.start;
        }

        public Vector getVel() {
            return this.vel;
        }

        public double getDragScalar() {
            return this.dragScalar;
        }

        public Vector getGravity() {
            return this.gravity;
        }
    
        public BallisticArc build() {
            return new ComputableBallisiticArc(this);
        }
    }    

    @Override
    public Iterator<Vector> iterator() {
        return points.iterator();
    }

    @Override
    public Iterator<Vector> pointIterator() {
        return points.iterator();
    }

    @Override
    public Iterator<Vector> velIterator() {
        return vels.iterator();
    }

    @Override
    public int size() {
        return points.size();
    }

    @Override
    public Vector getPoint(int i) {
        final int clamped = Math.max(-size(), Math.min(i, size() - 1));

        if(clamped < 0) {
            return points.get(this.size() + clamped);
        }

        return points.get(clamped);
    }

    @Override
    public Vector getVel(int i) {
        final int clamped = Math.max(-size(), Math.min(i, size() - 1));

        if(clamped < 0) {
            return vels.get(this.size() + clamped);
        }

        return vels.get(clamped);
    }

    public double getElapsedTime() {
        return this.elapsed;
    }

    private void update(double deltaTime) {
        final Vector curPos = getCurrentPoint();
        final Vector curVel = getCurrentVelocity();

        // Getting the forces
        final double dragMagnitude = 0.5 * dragScalar * curVel.norm();
        final Vector accel = curVel.scale(-dragMagnitude).add(gravity);
        final Vector deltaPos = accel
            .scale(0.5 * deltaTime * deltaTime)
            .add(curVel.scale(2 * deltaTime));

        //Updating the velocity and the position
        vels.add(curVel.add(accel.scale(deltaTime)));
        points.add(curPos.add(deltaPos));
        elapsed += deltaTime;
    }

    @Override
    public boolean compute(double stepSize, ArcFunction continuationCondition) {
        boolean didCompute = false;

        while(elapsed < TIMEOUT && continuationCondition.apply(this)) {
            didCompute = true;
            update(stepSize);
        }

        return didCompute;
    }
}