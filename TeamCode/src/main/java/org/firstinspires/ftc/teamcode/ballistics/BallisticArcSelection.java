package org.firstinspires.ftc.teamcode.ballistics;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.Map;

/**
 * Contains a list of arcs that can be quickly filtered using
 * preset criteria, or through other criteria.
 */
public final class BallisticArcSelection {
    public static enum Criterion implements ArcTransformer {
        /**
         * Gets the horizontal distance traveled by the arc when it reaches the 
         * goal. In other words, this is the final x coordinate of the arc.
         */
        DISTANCE((arc) -> arc.lastPoint().getXComponent()), 

        /**
         * Gets the speed of the arc's first velocity.
         */
        SPEED((arc) -> arc.firstVel().getMagnitude()),

        /**
         * Gets the angle from the x-axis of the arc's first velocity. The 
         * returned value is in radians
         */
        ANGLE((arc) -> arc.firstVel().getTheta()),
        
        /** 
         * The specific metric of arcs used in the selection is not known. 
         */
        UNKNOWN(null);

        private final ArcTransformer transformer;
        private Criterion(ArcTransformer transformer) {
            this.transformer = transformer;
        }

        @Override
        public Double apply(BallisticArc arc) {
            return transformer.apply(arc);
        }

        /**
         * Gets the value of the criterion that the given arc has. For example,
         * `Crtierion.DISTANCE.of(arc)` would get the distance traveled of `arc`
         * 
         * This is an alias for `apply()`
         * 
         * @return The criterion's value on the given arc.
         */
        public double of(BallisticArc arc) {
            return apply(arc);
        }
    }

    /**
     * The actual arcs, in no particular order. Duplicates are allowed but strongly
     * discouraged.
     */
    private final BallisticArc[] arcs;

    /**
     * Arrays of indices whose arcs are sorted ascendingly by the given criterion.
     */
    private final Map<Criterion, Integer[]> sortedPointers = new HashMap<>(Criterion.values().length, 1.0f);

    /**
     * The indices of the arcs array that this selection contains.
     * What order elements are sorted in is specified by the `criterion` 
     * field. Null if the selection contains the entire all of the arcs.
     */
    private final Integer[] indices;

    /**
     * The criterion by which indices is sorted. `UNKNOWN` if the selection 
     * contains all of the arcs or has been filtered using `filter()`
     */
    private final Criterion sortedBy;

    public BallisticArcSelection(Collection<BallisticArc> arcs) {
        this.arcs = arcs.toArray(new BallisticArc[arcs.size()]);
        this.sortedBy = null;
        this.indices = null;

        // Sorting indices by initial angle
        //
        // Because parallel sort is in-place, this sorting can be used as a tie breaker
        // if other selections have identical speeds or distances.
        final Integer[] angle = new Integer[this.arcs.length];
        Arrays.setAll(angle, (i) -> i);
        Arrays.parallelSort( angle, compareBy(Criterion.ANGLE));

        // Sorting indices by initial speed
        final Integer[] speed = Arrays.copyOf(angle, angle.length);
        Arrays.parallelSort( speed, compareBy(Criterion.SPEED));

        // Sorting by distances
        final Integer[] dist = Arrays.copyOf(speed, speed.length);
        Arrays.parallelSort( dist, compareBy(Criterion.DISTANCE));

        // Putting the sorted indices into the sortBy map
        sortedPointers.put(Criterion.ANGLE, angle);
        sortedPointers.put(Criterion.SPEED, speed);
        sortedPointers.put(Criterion.DISTANCE, dist);
    }

    private Comparator<Integer> compareBy(ArcTransformer transformer) {
        return (a, b) -> (int) Math.signum(transformer.apply(arcs[a]) - transformer.apply(arcs[b]));
    }

    private BallisticArcSelection(BallisticArcSelection sources, Criterion crit, Collection<Integer> indices) {
        this(sources, crit, indices.toArray(new Integer[indices.size()]));
    }

    private BallisticArcSelection(BallisticArcSelection sources, Criterion crit, Integer[] indices) {
        this.arcs = sources.arcs;
        this.sortedPointers.putAll(sources.sortedPointers);
        this.sortedBy = crit;
        this.indices = indices;
    }

    private BallisticArcSelection linearFilter(Criterion crit, ArcFunction condition) {
        final Collection<Integer> subIndices = new ArrayList<>();

        for(final Integer index : this.indices) {
            if(condition.apply(arcs[index])) {
                subIndices.add(index);
            }
        }

        return new BallisticArcSelection(this, crit, subIndices);
    }
    
    private BallisticArcSelection linearFilter(ArcFunction condition) {
        return linearFilter(sortedBy, condition);
    }

    /**
     * Retains only the arcs in the selection that satisfy the given condition.
     * The selection is not modified as a result; instead, a new selection is 
     * created
     * 
     * @param condition True if the given arc should be kept; false otherwise.
     * @return All arcs that satisfied the given condition
     */
    public BallisticArcSelection filter(ArcFunction condition) {
        return linearFilter(Criterion.UNKNOWN, condition);
    }

    private int binarySearch(Integer[] arr, int start, int end, boolean exclusive, ArcTransformer comparator) {
        int min = start;
        int max = end;

        while(min <= max) {
            final int avg = (min + max) / 2;
            final double comp = comparator.apply(arcs[arr[avg]]);
            
            if(comp == 0) {
               return exclusive ? avg + 1 : avg;
            } else if(comp < 0) {
                min = avg + 1;
            } else {
                max = avg - 1;
            }
        }

        if(exclusive) {
            return -min - 1;
        } else {
            return -max - 1;
        }
    }

    private int binarySearch(Integer[] arr, ArcTransformer comparator) {
        return binarySearch(arr, 0, arr.length - 1, false, comparator);
    }

    private static enum BinarySearch {
        TOO_FAR_LEFT(-1.0),
        TOO_FAR_RIGHT(1.0),
        RIGHT_ON_THE_MONEY(0.0);

        public final double sign;
        private BinarySearch(double sign) {
            this.sign = sign;
        }
    }

    /**
     * Returns all arcs whose distance from the goal is within tolerance of the 
     * specified distance.
     * 
     * Tolerance is inclusive. Because of this, a tolerance of 0 will get only
     * arcs whose distance is exactly the target distance.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param target Target distance, in inches
     * @param tolerance Allowed plus-or-minus difference, in inches
     * @return Arcs whose point of scoring (y = 39in) is approximately the given 
     * distance
     */
    public BallisticArcSelection withinDistance(final double target, final double tolerance) {
        // If the selection cannot be guaranteed to be sorted by distance, use a linear search
        if(indices != null && sortedBy != Criterion.DISTANCE) {
            return linearFilter((arc) -> Math.abs(Criterion.DISTANCE.of(arc) - target) <= tolerance);
        }
        
        // We are sorted by distance; searching for the bounds 
        final Integer[] sorted = indices != null ? indices : sortedPointers.get(Criterion.DISTANCE);
        int lower = binarySearch(sorted, (arc) -> {
            final double delta = Criterion.DISTANCE.of(arc) - target;

            // Quartiles 2, 3, 4
            if(delta >= -tolerance) {
                return BinarySearch.TOO_FAR_RIGHT.sign;
            } 
            
            // Quartiles 1
            return BinarySearch.TOO_FAR_LEFT.sign;
        });

        // Handling bounds that were not found exactly
        if(lower < 0) {
            lower = -lower; // Adding one so that it is inside Q2, not Q1
        }

        int upper = binarySearch(sorted, lower, sorted.length - 1, true, (arc) -> {
            final double delta = Criterion.DISTANCE.of(arc) - target;

            // Quartiles 1,2,3
            if(delta <= tolerance) {
                return BinarySearch.TOO_FAR_LEFT.sign;
            } 
            
            // Quartiles 4
            return BinarySearch.TOO_FAR_RIGHT.sign;
        });

        if(upper < 0) {
            upper = -upper - 1;
        }

        // Slicing the indices
        return new BallisticArcSelection(
            this, 
            Criterion.DISTANCE, 
            Arrays.copyOfRange(sorted, lower, upper)
        );
    }

    /**
     * Returns all arcs whose initial firing angle is within tolerance of the 
     * specified angle, in radians. 
     * 
     * Tolerance is inclusive. Because of this, a tolerance of 0 will get only
     * arcs whose initial angle is exactly the target angle.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param target Target angle, in radians
     * @param tolerance Allowed plus-or-minus difference, in radians
     * @return Arcs whose initial firing angle is approximately the given angle.
     */
    public BallisticArcSelection withinAngle(final double target, final double tolerance) {
        // If the selection cannot be guaranteed to be sorted by angle, use a linear search
        if(indices != null && sortedBy != Criterion.ANGLE) {
            return linearFilter((arc) -> Math.abs(Criterion.ANGLE.of(arc) - target) <= tolerance);
        }
        
        // We are sorted by angle; searching for the bounds 
        final Integer[] sorted = indices != null ? indices : sortedPointers.get(Criterion.ANGLE);
        int lower = binarySearch(sorted, (arc) -> {
            final double delta = Criterion.ANGLE.of(arc) - target;

            // Quartiles 2, 3, 4
            if(delta >= -tolerance) {
                return BinarySearch.TOO_FAR_RIGHT.sign;
            } 
            
            // Quartiles 1
            return BinarySearch.TOO_FAR_LEFT.sign;
        });

        // Handling bounds that were not found exactly
        if(lower < 0) {
            lower = -lower; // Adding one so that it is inside Q2, not Q1
        }

        int upper = binarySearch(sorted, lower, sorted.length - 1, true, (arc) -> {
            final double delta = Criterion.ANGLE.of(arc) - target;
            
            // Quartiles 1,2,3
            if(delta <= tolerance) {
                return BinarySearch.TOO_FAR_LEFT.sign;
            } 
            
            // Quartiles 4
            return BinarySearch.TOO_FAR_RIGHT.sign;
        });

        if(upper < 0) {
            upper = -upper - 1;
        }

        // Slicing the indices
        return new BallisticArcSelection(
            this, 
            Criterion.ANGLE, 
            Arrays.copyOfRange(sorted, lower, upper)
        );
    }

    /**
     * Returns all arcs whose initial firing speed is within tolerance of the 
     * specified speed, in inches per second. 
     * 
     * Tolerance is inclusive. Because of this, a tolerance of 0 will get only
     * arcs whose initial speed is exactly the target speed.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param target Target speed, in inches per second
     * @param tolerance Allowed plus-or-minus difference, in inches per second
     * @return Arcs whose initial firing speed is approximately the given speed.
     */
    public BallisticArcSelection withinSpeed(final double target, final double tolerance) {
        // If the selection cannot be guaranteed to be sorted by speed, use a linear search
        if(indices != null && sortedBy != Criterion.SPEED) {
            return linearFilter((arc) -> Math.abs(Criterion.SPEED.of(arc) - target) <= tolerance);
        }
        
        // We are sorted by speed; searching for the bounds 
        final Integer[] sorted = indices != null ? indices : sortedPointers.get(Criterion.SPEED);
        int lower = binarySearch(sorted, (arc) -> {
            final double delta = Criterion.SPEED.of(arc) - target;

            // Quartiles 2, 3, 4
            if(delta >= -tolerance) {
                return BinarySearch.TOO_FAR_RIGHT.sign;
            } 
            
            // Quartiles 1
            return BinarySearch.TOO_FAR_LEFT.sign;
        });

        // Handling bounds that were not found exactly
        if(lower < 0) {
            lower = -lower; // Adding one so that it is inside Q2, not Q1
        }

        int upper = binarySearch(sorted, lower, sorted.length - 1, true, (arc) -> {
            final double delta = Criterion.SPEED.of(arc) - target;

            // Quartiles 1,2,3
            if(delta <= tolerance) {
                return BinarySearch.TOO_FAR_LEFT.sign;
            } 
            
            // Quartiles 4
            return BinarySearch.TOO_FAR_RIGHT.sign;
        });

        if(upper < 0) {
            upper = -upper - 1;
        }

        // Slicing the indices
        return new BallisticArcSelection(
            this, 
            Criterion.SPEED, 
            Arrays.copyOfRange(sorted, lower, upper)
        );
    }

    private BallisticArcSelection sortedMinimum(
        Integer[] sortedIndices, 
        double tolerance,
        Criterion criterion, 
        ArcTransformer transformer
    ) {
        final LinkedList<Integer> resultIndices = new LinkedList<>();            
        final double minValue = transformer.apply(arcs[sortedIndices[0]]);

        for(int i = 0; i < sortedIndices.length; i++) {
            final double currentValue = transformer.apply(arcs[sortedIndices[i]]);

            // Breaking if the value was greater than the previous one
            // We do this becuase the array is sorted, so the next will also be greater
            if(currentValue > minValue + tolerance) {            
                return new BallisticArcSelection(this, criterion, resultIndices);
            }

            // The value is equal; add it to the array
            resultIndices.add(sortedIndices[i]);
        }

        return new BallisticArcSelection(this, criterion, resultIndices);

    }

    private BallisticArcSelection unsortedMinimum(
        Integer[] indices,
        double tolerance,
        Criterion criterion,
        ArcTransformer transformer
    ) {
        double minValue = Double.POSITIVE_INFINITY;

        // Finding the absolute minimum
        for(final Integer index : indices) {
            final double currentValue = transformer.apply(arcs[index]);

            if(currentValue < minValue) {
                // The value is less than the previous minimum, then make this the new min
                minValue = currentValue;
            }
        }

        // Getting all the values that are within tolerance of the minimum
        final LinkedList<Integer> resultIndices = new LinkedList<>();

        for(final Integer index : indices) {
            final double currentValue = transformer.apply(arcs[index]);

            if(Math.abs(currentValue - minValue) <= tolerance) {
                resultIndices.add(index);
            }
        }

        return new BallisticArcSelection(this, criterion, resultIndices);
    }

    /**
     * Gets the arcs in this selection that have the shortest distance to the 
     * goal. If two or more arcs have the same distance, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what distances are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute minimum are returned
     * For example, a selection may contain the values `[1.1, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[1.0]` because it is the 
     * smallest value. Specifying a tolerance of 0.2 would return `[1.1, 1.0]` because 
     * 1.0 is the smallest  value and 1.1 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-minus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the minimum distance; can be empty, but never null.
     */
    public BallisticArcSelection minDistance(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the DISTANCE sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.DISTANCE && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.DISTANCE) 
                : indices;
            return sortedMinimum(sortedIndices, tolerance, Criterion.DISTANCE, Criterion.DISTANCE);
        }

        // Linear search for the minimum distance
        return unsortedMinimum(this.indices, tolerance, this.sortedBy, Criterion.DISTANCE);
    }

    /**
     * Gets the arcs in this selection that have the shortest distance to the 
     * goal. If two or more arcs have the same distance, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the minimum distance; can be empty, but never null.
     */
    public BallisticArcSelection minDistance() {
        return minDistance(0.0);
    } 

    /**
     * Gets the arcs in this selection that have the shallowest angle to the 
     * goal. If two or more arcs have the same angle, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what angles are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute minimum are returned
     * For example, a selection may contain the values `[1.1, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[1.0]` because it is the 
     * smallest value. Specifying a tolerance of 0.2 would return `[1.1, 1.0]` because 
     * 1.0 is the smallest  value and 1.1 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-minus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the minimum angle; can be empty, but never null.
     */
    public BallisticArcSelection minAngle(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the ANGLE sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.ANGLE && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.ANGLE) 
                : indices;
            return sortedMinimum(sortedIndices, tolerance, Criterion.ANGLE, Criterion.ANGLE);
        }

        // Linear search for the minimum angle
        return unsortedMinimum(this.indices, tolerance, this.sortedBy, Criterion.ANGLE);
    }

    /**
     * Gets the arcs in this selection that have the shallowest angle to the 
     * goal. If two or more arcs have the same angle, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the minimum angle; can be empty, but never null.
     */
    public BallisticArcSelection minAngle() {
        return minAngle(0.0);
    }

    /**
     * Gets the arcs in this selection that have the slowest speed to the 
     * goal. If two or more arcs have the same speed, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what speeds are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute minimum are returned
     * For example, a selection may contain the values `[1.1, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[1.0]` because it is the 
     * smallest value. Specifying a tolerance of 0.2 would return `[1.1, 1.0]` because 
     * 1.0 is the smallest  value and 1.1 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-minus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the minimum speed; can be empty, but never null.
     */
    public BallisticArcSelection minSpeed(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the SPEED sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.SPEED && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.SPEED) 
                : indices;
            return sortedMinimum(sortedIndices, tolerance, Criterion.SPEED, Criterion.SPEED);
        }

        // Linear search for the minimum speed
        return unsortedMinimum(this.indices, tolerance, this.sortedBy, Criterion.SPEED);
    }

    /**
     * Gets the arcs in this selection that have the slowest speed to the 
     * goal. If two or more arcs have the same speed, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the minimum speed; can be empty, but never null.
     */
    public BallisticArcSelection minSpeed() {
        return minSpeed(0.0);
    }

    private BallisticArcSelection sortedMaximum(
        Integer[] sortedIndices, 
        double tolerance,
        Criterion criterion, 
        ArcTransformer transformer
    ) {
        final LinkedList<Integer> resultIndices = new LinkedList<>();            
        final double maxValue = transformer.apply(arcs[ sortedIndices[sortedIndices.length - 1 ] ]);

        for(int i = sortedIndices.length - 1; i >= 0; i--) {
            final double currentValue = transformer.apply(arcs[sortedIndices[i]]);

            // Breaking if the value was greater than the previous one
            // We do this becuase the array is sorted, so the next will also be greater
            if(currentValue < maxValue - tolerance) {            
                return new BallisticArcSelection(this, criterion, resultIndices);
            }

            // The value is equal; add it to the array
            resultIndices.addFirst(sortedIndices[i]); // addFirst() because the loop moves backwards
        }

        return new BallisticArcSelection(this, criterion, resultIndices);

    }

    private BallisticArcSelection unsortedMaximum(
        Integer[] indices,
        double tolerance,
        Criterion criterion,
        ArcTransformer transformer
    ) {
        double maxValue = Double.NEGATIVE_INFINITY;

        // Finding the absolute maximum
        for(final Integer index : indices) {
            final double currentValue = transformer.apply(arcs[index]);

            if(currentValue > maxValue) {
                // The value is greater than the previous maximum, then make this the new max
                maxValue = currentValue;
            }
        }

        // Getting all the values that are within tolerance of the maximum
        final LinkedList<Integer> resultIndices = new LinkedList<>();

        for(final Integer index : indices) {
            final double currentValue = transformer.apply(arcs[index]);

            if(Math.abs(currentValue - maxValue) <= tolerance) {
                resultIndices.add(index);
            }
        }

        return new BallisticArcSelection(this, criterion, resultIndices);
    }

    /**
     * Gets the arcs in this selection that have the longest distance to the 
     * goal. If two or more arcs have the same distance, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what distances are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute maximum are returned
     * For example, a selection may contain the values `[1.9, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[2.0]` because it is the 
     * largest value. Specifying a tolerance of 0.2 would return `[1.9, 2.0]` because 
     * 2.0 is the largest  value and 1.9 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-minus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the maximum distance; can be empty, but never null.
     */
    public BallisticArcSelection maxDistance(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the DISTANCE sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.DISTANCE && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.DISTANCE) 
                : indices;
            return sortedMaximum(sortedIndices, tolerance, Criterion.DISTANCE, Criterion.DISTANCE);
        }

        // Linear search for the maximum distance
        return unsortedMaximum(this.indices, tolerance, this.sortedBy, Criterion.DISTANCE);
    }

    /**
     * Gets the arcs in this selection that have the longest distance to the 
     * goal. If two or more arcs have the same distance, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the maximum distance; can be empty, but never null.
     */
    public BallisticArcSelection maxDistance() {
        return maxDistance(0.0);
    } 

    /**
     * Gets the arcs in this selection that have the steepest angle to the 
     * goal. If two or more arcs have the same angle, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what angles are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute maximum are returned
     * For example, a selection may contain the values `[1.9, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[2.0]` because it is the 
     * largest value. Specifying a tolerance of 0.2 would return `[1.9, 2.0]` because 
     * 2.0 is the largest  value and 1.9 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-maxus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the maximum angle; can be empty, but never null.
     */
    public BallisticArcSelection maxAngle(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the ANGLE sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.ANGLE && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.ANGLE) 
                : indices;
            return sortedMaximum(sortedIndices, tolerance, Criterion.ANGLE, Criterion.ANGLE);
        }

        // Linear search for the maximum angle
        return unsortedMaximum(this.indices, tolerance, this.sortedBy, Criterion.ANGLE);
    }

    /**
     * Gets the arcs in this selection that have the steepst angle to the 
     * goal. If two or more arcs have the same angle, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the maximum angle; can be empty, but never null.
     */
    public BallisticArcSelection maxAngle() {
        return maxAngle(0.0);
    }

    /**
     * Gets the arcs in this selection that have the fastest speed to the 
     * goal. If two or more arcs have the same speed, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * Specifying a tolerance affects what speeds are considered equal. In general, 
     * only values that `tolerance` or less away from the absolute maximum are returned
     * For example, a selection may contain the values `[1.9, 1.0, 2.0]`. Specifying a 
     * tolerance of 0 would return a selection containing only `[2.0]` because it is the 
     * largest value. Specifying a tolerance of 0.2 would return `[1.9, 2.0]` because 
     * 2.0 is the largest  value and 1.9 is only 0.1 away from 1.0, which is less 
     * than 0.2. 
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @param tolerance Plus-or-maxus difference that two values can have and still be 
     * considered equal.
     * @return The arcs with the maximum speed; can be empty, but never null.
     */
    public BallisticArcSelection maxSpeed(double tolerance) {
        // If we can use a sorted array, use that instead
        // When indices is null, we can use the SPEED sortedPointers indices.
        if(indices == null || (sortedBy == Criterion.SPEED && indices != null)) {
            final Integer[] sortedIndices = indices == null 
                ? sortedPointers.get(Criterion.SPEED) 
                : indices;
            return sortedMaximum(sortedIndices, tolerance, Criterion.SPEED, Criterion.SPEED);
        }

        // Linear search for the maximum speed
        return unsortedMaximum(this.indices, tolerance, this.sortedBy, Criterion.SPEED);
    }

    /**
     * Gets the arcs in this selection that have the slowest speed to the 
     * goal. If two or more arcs have the same speed, they will all be 
     * included in the returned selection. If no arcs are contained in the current
     * selection, an empty selection is returned.
     * 
     * This method does not modify the selection, instead creating a new 
     * selection.
     * 
     * @return The arcs with the maximum speed; can be empty, but never null.
     */
    public BallisticArcSelection maxSpeed() {
        return maxSpeed(0.0);
    }

    
    /**
     * Gets the number of arcs contained in this selection. This is equal to 
     * the length of the array returned by `toArray()`.
     * 
     * @return The total number of arcs in this selection.
     */
    public int size() {
        return indices == null ? arcs.length : indices.length;
    }

    /**
     * Gets the arrays this selection contains. This is a deep copy
     * of the actual contents of this selection. 
     * 
     * @return A deep copy of the contents of this selection.
     */
    public BallisticArc[] toArray() {
        final BallisticArc[] result = new BallisticArc[this.size()];

        // Using indices if that is provided
        if(indices != null) {
            for(int i = 0; i < indices.length; i++) {
                result[i] = new ImmutableBallisticArc(arcs[ indices[i] ]);
            }

            return result;
        }

        // Copying the entire arry if no indices are provided 
        for(int i = 0; i < arcs.length; i++) {
            result[i] = new ImmutableBallisticArc(arcs[i]);
        }        

        return result;
    }

    /**
     * Gets the first arc contained by this selection. No order of arcs is 
     * guaranteed by this method. This returns null if the selection is empty
     * 
     * @return The first arc in the selection's arcs. 
     */
    public BallisticArc first() {
        if(indices != null && indices.length == 0) {
            return null;
        }

        return arcs[indices[0]];
    }
    
    /**
     * Gets the last arc contained by this selection. No order of arcs is 
     * guaranteed by this method. This returns null is the selection is empty
     * 
     * @return The last arc in the selection's arcs. 
     */
    public BallisticArc last() {
        if(indices != null && indices.length == 0) {
            return null;
        }

        return arcs[ indices[indices.length - 1] ];
    }

    @Override
    public String toString() {
        return String.format("BallisticArcSelection (" + this.size() + ")");
    }
}