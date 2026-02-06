package org.firstinspires.ftc.teamcode.test.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.RippleyColorBlind;
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

import static org.firstinspires.ftc.teamcode.util.Util.*;
import static org.firstinspires.ftc.teamcode.test.TestUtil.*;
import static org.junit.jupiter.api.Assertions.*;

class RippleyColorBlindUnitTest {
    @Test
    @DisplayName("minTravelDist returns the closest point")
    void minTravelDistReturnsClosestPoint() {
        // Blue first shooting
        final BezierLine segment1 = new BezierLine(
            new Pose(0, 144),
            new Pose(72, 72)
        );
        
        final BezierLine segment12 = new BezierLine(
            new Pose(72, 72),
            new Pose(0, 144)
        );
        final Pose p1 = new Pose(24, 84);
        final Pose p2 = new Pose(28, 105);
        final Pose closest1 = RippleyColorBlind.minTravelDist(segment1, p1, p2);
        final Pose closest12 = RippleyColorBlind.minTravelDist(segment12, p1, p2);
        final Pose expected1 = new Pose(35.4893617, 108.510638);

        assertWithin(expected1.getX(), closest1.getX(), 1e-6);
        assertWithin(expected1.getY(), closest1.getY(), 1e-6);
        
        assertWithin(expected1.getX(), closest12.getX(), 1e-6);
        assertWithin(expected1.getY(), closest12.getY(), 1e-6);
        
        // Red first shooting
        final BezierLine segment2 = new BezierLine(
            new Pose(144, 144),
            new Pose(72, 72)
        );
        final Pose p3  = new Pose(120, 84);
        final Pose p4  = new Pose(116, 105);
        final Pose p32 = new Pose(116, 105);
        final Pose p42 = new Pose(120, 84);
        final Pose closest2 = RippleyColorBlind.minTravelDist(segment2, p3, p4);
        final Pose closest22 = RippleyColorBlind.minTravelDist(segment2, p32, p42);
        final Pose expected2 = new Pose(144 - 35.4893617, 108.510638);

        assertWithin(expected2.getX(), closest2.getX(), 1e-6);
        assertWithin(expected2.getY(), closest2.getY(), 1e-6);

        assertWithin(expected2.getX(), closest22.getX(), 1e-6);
        assertWithin(expected2.getY(), closest22.getY(), 1e-6);


        // End point
        final BezierLine segment3 = new BezierLine(
            new Pose(0, 144),
            new Pose(72, 72)
        );
        final Pose p5 = new Pose(108, 24);
        final Pose p6 = new Pose(144, 24);
        final Pose closest3 = RippleyColorBlind.minTravelDist(segment3, p5, p6);
        final Pose expected3 = new Pose(72, 72);

        assertWithin(expected3.getX(), closest3.getX(), 1e-9);
        assertWithin(expected3.getY(), closest3.getY(), 1e-9);
    }
}