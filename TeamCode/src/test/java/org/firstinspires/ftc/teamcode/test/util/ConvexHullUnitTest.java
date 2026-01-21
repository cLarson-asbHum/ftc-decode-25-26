package org.firsinspires.ftc.teamcode.test.util;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.util.ConvexHull;

import static org.firstinspires.ftc.teamcode.util.Util.*;
import static org.firstinspires.ftc.teamcode.test.TestUtil.*;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

class ConvexHullUnitTest {
    @DisplayName("Can construct")
    @Test
    void canConstruct() {
        ConvexHull.of(new Pose[]{ new Pose(1, 1), new Pose(2, 3), new Pose(10, -1)  });
        ConvexHull.of(new Pose[]{ new Pose(0, 144), new Pose(144, 144), new Pose(72, 72)  });
    }

    @DisplayName("Throws when invalid")
    @Test
    void throwsWhenInvalid() {
        doesThrow(() -> ConvexHull.of(null));

        doesThrow(() -> ConvexHull.of(new Pose[]{}));
        doesThrow(() -> ConvexHull.of(new Pose[]{null }));
        doesThrow(() -> ConvexHull.of(new Pose[]{null, null}));
        doesThrow(() -> ConvexHull.of(new Pose[]{null, null, null}));
        doesThrow(() -> ConvexHull.of(new Pose[]{null, null, null, null}));
        
        doesThrow(() -> ConvexHull.of(new Pose[]{new Pose(1, 1) }));
        doesThrow(() -> ConvexHull.of(new Pose[]{new Pose(1, 1), new Pose(2, 1)}));
        doesThrow(() -> ConvexHull.of(new Pose[]{new Pose(1, 1), new Pose(2, 1), null}));
        doesThrow(() -> ConvexHull.of(new Pose[]{null, new Pose(1, 1), new Pose(2, 1)}));
        doesThrow(() -> ConvexHull.of(new Pose[]{new Pose(1, 1), null, new Pose(2, 1)}));
    }

    @DisplayName("Boundary is correct")
    @Test
    void boundaryIsCorrect() {
        final Pose[] points1 = new Pose[] {
            new Pose(0, 144),
            new Pose(72, 108),
            new Pose(144, 144),
            new Pose(72, 72),
            new Pose(100, 50),
        };
        final ConvexHull hull1 = ConvexHull.of(points1);
        final Pose[] expected1 = new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(100, 50),
            new Pose(72, 72)
        };
        final Pose[] result1 = hull1.getBoundary();

        assertEquals(expected1.length, result1.length, "Are lengths equal");
        for(int i = 0; i < result1.length; i++) {
            System.out.printf("[Are correct order] \t %s, \t %s \n", result1[i], expected1[i]);
            assertTrue(expected1[i].roughlyEquals(result1[i], 1e-9), "Are correct poses in corretc order?");
        }
        
        final Pose[] points2 = new Pose[] {
            new Pose(48, 0),
            new Pose(72, 24),
            new Pose(96, 0)
        };
        final Pose[] expected2 = points2;
        final ConvexHull hull2 = ConvexHull.of(points2);
        final Pose[] result2 = hull2.getBoundary();
        assertEquals(expected2.length, result2.length, "Are lengths equal");
        for(int i = 0; i < result2.length; i++) {
            System.out.printf("[Are correct order] \t %s, \t %s \n", result2[i], expected2[i]);
            assertTrue(expected2[i].roughlyEquals(result2[i], 2e-9), "Are correct poses in corretc order?");
        }
    }

    @DisplayName("Contains correct points")
    @Test
    void contains() {

        final Pose[] points1 = new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        };
        final ConvexHull hull1 = ConvexHull.of(points1);

        assertTrue(hull1.contains(new Pose(100, 130)), "Contains (100, 130)");
        assertTrue(hull1.contains(new Pose(72, 108)), "Contains (72, 108)");
        assertFalse(hull1.contains(new Pose(0, 72)), "Not Contains (0, 72)");
        assertFalse(hull1.contains(new Pose(144, 108)), "Not Contains left");
        assertFalse(hull1.contains(new Pose(0, 108)), "Not Contains right");
        assertFalse(hull1.contains(new Pose(72, 200)), "Not contains above");
        assertTrue(hull1.contains(new Pose(72, 72)), "Contains vertex");
        
        final Pose[] points2 = new Pose[] {
            new Pose(48, 0),
            new Pose(72, 24),
            new Pose(96, 0)
        };
        final ConvexHull hull2 = ConvexHull.of(points2);

        assertTrue(hull2.contains(new Pose(90, 2)), "Contains (90, 2)");
        assertFalse(hull2.contains(new Pose(72, 72)), "Not Contains above");
        assertFalse(hull2.contains(new Pose(0, 0)), "Contains Edge");
    }

    @DisplayName("ClosestPoint get edges")
    @Test
    void closestPointIsCorrect() {
        final ConvexHull hull1 = ConvexHull.of(new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        });

        assertTrue(hull1.closestPointTo(new Pose(36, 108)).roughlyEquals(new Pose(36, 108), 1e-9), "Edge point returns itself");
        assertTrue(hull1.closestPointTo(new Pose(0, 72)).roughlyEquals(new Pose(36, 108), 1e-9), "Out of edge returns closests on closest edge");
        assertTrue(hull1.closestPointTo(new Pose(144, 72)).roughlyEquals(new Pose(108, 108), 1e-9), "Out of edge returns closests on closest edge");
        assertTrue(hull1.closestPointTo(new Pose(20, 200)).roughlyEquals(new Pose(20, 144), 1e-9), "Out of edge returns closests on closest edge");
        
        final ConvexHull hull2 = ConvexHull.of(new Pose[] {
            new Pose(48, 0),
            new Pose(96, 0),
            new Pose(72, 24),
        });

        assertTrue(hull2.closestPointTo(new Pose(60, 12)).roughlyEquals(new Pose(60, 12), 1e-9), "Edge point returns itself");
        assertTrue(hull2.closestPointTo(new Pose(48, 24)).roughlyEquals(new Pose(60, 12), 1e-9), "Out of edge returns closests on closest edge");
        assertTrue(hull2.closestPointTo(new Pose(108, 36)).roughlyEquals(new Pose(84, 12), 1e-9), "Out of edge returns closests on closest edge");
        assertTrue(hull2.closestPointTo(new Pose(50, -20)).roughlyEquals(new Pose(50, 0), 1e-9), "Out of edge returns closests on closest edge");
    }
    
    @DisplayName("ClosestPoint get vertices")
    @Test
    void closestGetsVertices() {

        final ConvexHull hull1 = ConvexHull.of(new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        });

        assertTrue(hull1.closestPointTo(new Pose(72, 72)).roughlyEquals(new Pose(72, 72), 1e-9), "Vertex returns itself");
        assertTrue(hull1.closestPointTo(new Pose(-10, 144)).roughlyEquals(new Pose(0, 144), 1e-9), "Out of edge returns closests on closest vertex");
        assertTrue(hull1.closestPointTo(new Pose(160, 160)).roughlyEquals(new Pose(144, 144), 1e-9), "Out of edge returns closests on closest vertex");
        assertTrue(hull1.closestPointTo(new Pose(144, 0)).roughlyEquals(new Pose(72, 72), 1e-9), "Out of edge returns closests on closest vertex");
    }
    
    @DisplayName("ClosestPoint gets inside")
    @Test
    void closestGetsInside() {
        final Pose[] points1 = new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        };
        final ConvexHull hull1 = ConvexHull.of(points1);

        assertTrue(hull1.closestPointTo(new Pose(100, 130)).roughlyEquals(new Pose(100, 130), 1e-9), "Indentity (100, 130)");
        assertTrue(hull1.closestPointTo(new Pose(72, 108)).roughlyEquals(new Pose(72, 108), 1e-9), "Indentity (72, 108)");
        assertTrue(hull1.closestPointTo(new Pose(72, 72)).roughlyEquals(new Pose(72, 72), 1e-9), "Indentity Vertex");
    }

    @DisplayName("Intersects is correct")
    @Test
    void intersectPassesCorrectly() {
        final ConvexHull hull1 = ConvexHull.of(new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        });
        final double size = 8.5;
        final double[] rotations = new double[] {
            Math.toRadians(0),
            Math.toRadians(31.4),
            Math.toRadians(-1),
            Math.toRadians(0),

            Math.toRadians(135),
            Math.toRadians(-110)
        };
        final Pose[] translations = new Pose[]{
            new Pose(84, 80),
            new Pose(100, 93),
            new Pose(72, 120),
            new Pose(72, 65),

            new Pose(20, 20),
            new Pose(24, 96)
        };
        final boolean[] expected = new boolean[]{
            true,  // Center inside
            true,  // Center outside of hull1, but still intersecting
            true,  // fully inside
            true,  // Edge inside, no verts

            false, // Fails aabb
            false, // Inside aabb, but just misses
        };

        for(int i = 0;  i < rotations.length; i++) {
            // Transforming the rectangle
            final double rot = rotations[i];
            final Pose trans = translations[i];
            final boolean exp = expected[i];

            final ConvexHull rect = ConvexHull.of(new Pose[] {
                new Pose(-size, -size).rotate(rot, false).plus(trans),
                new Pose( size, -size).rotate(rot, false).plus(trans),
                new Pose( size,  size).rotate(rot, false).plus(trans),
                new Pose(-size,  size).rotate(rot, false).plus(trans),
            });

            // Doing the comparison
            final String name = String.format("Close goal with rectangle %.1f° + %s", Math.toDegrees(rot), trans);
            assertEquals(exp, hull1.intersects(rect), name);
        }
    }

    @DisplayName("Intersects circle passes when correct")
    @Test
    void intersectCirclePassesCorrectly() {
        
        final ConvexHull hull1 = ConvexHull.of(new Pose[] {
            new Pose(0, 144),
            new Pose(144, 144),
            new Pose(72, 72),
        });
        final double size = 8.5;
        final Pose[] translations = new Pose[]{
            new Pose(84, 80),
            new Pose(100, 93),
            new Pose(72, 120),
            new Pose(72, 65),

            new Pose(20, 20),
            new Pose(24, 96)
        };
        final boolean[] expected = new boolean[]{
            true,  // Center inside
            true,  // Center outside of hull1, but still intersecting
            true,  // fully inside
            true,  // Edge inside, no verts

            false, // Fails aabb
            false, // Inside aabb, but just misses
        };

        for(int i = 0;  i < translations.length; i++) {
            // Transforming the rectangle
            final Pose trans = translations[i];
            final boolean exp = expected[i];

            // Doing the comparison
            final String name = String.format("Close goal with circle %s", trans);
            assertEquals(exp, hull1.intersectsCircle(trans, size), name);
        }
    }
}