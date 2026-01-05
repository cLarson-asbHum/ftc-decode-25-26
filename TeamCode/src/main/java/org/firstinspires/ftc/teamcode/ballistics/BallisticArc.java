package org.firstinspires.ftc.teamcode.ballistics;

import java.util.Iterator;
import com.pedropathing.math.Vector;

public interface BallisticArc extends Iterable<Vector> {
    public static String stringify(BallisticArc arc) {
        final Vector firstVel = arc.firstVel();
        return String.format(
            "BallisticArc<%f, %f°>(%d)",
            firstVel.getMagnitude(),
            Math.toDegrees(firstVel.getTheta()),
            arc.size()
        );
    }

    double DEFAULT_STEP_SIZE = 1.0 / 240.0;
    
    int size();

    Vector getPoint(int i);

    Vector getVel(int i);

    double getElapsedTime();

    boolean compute(double timeStepSize, ArcFunction continuationCondition);
    
    default boolean compute(ArcFunction continuationCondition) {
        return compute(DEFAULT_STEP_SIZE, continuationCondition);
    }

    default boolean compute() {
        return compute((arc) -> arc.getCurrentPoint().getYComponent() > 0);
    }

    default Vector firstPoint() {
        return getPoint(0);
    }

    default Vector lastPoint() {
        return getPoint(size() - 1);
    }

    default Vector getCurrentPoint() {
        return lastPoint();
    }

    default Vector firstVel() {
        return getVel(0);
    }

    default Vector lastVel() {
        return getVel(size() - 1);
    }

    default Vector getCurrentVelocity() {
        return lastVel();
    }

    default Iterator<Vector> pointIterator() {
        return new Iterator<Vector>() {
            private int i = 0;

            @Override
            public boolean hasNext() {
                return i < size();
            }

            @Override
            public Vector next() {
                return getPoint(i++);
            }
        }; 
    }
    
    default Iterator<Vector> velIterator() {
        return new Iterator<Vector>() {
            private int i = 0;

            @Override
            public boolean hasNext() {
                return i < size();
            }

            @Override
            public Vector next() {
                return getVel(i++);
            }
        }; 
    }

    @Override
    default Iterator<Vector> iterator() {
        return pointIterator();
    }
}