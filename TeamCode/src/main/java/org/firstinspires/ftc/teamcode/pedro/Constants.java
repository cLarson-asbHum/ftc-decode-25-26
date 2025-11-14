package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {    
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(22.0 /* kg */) // TODO: Find mass in kg, because I have not put it on the scale

        ;

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .leftFrontMotorName("frontLeft") 
        .leftRearMotorName("backLeft")
        .rightFrontMotorName("frontRight")
        .rightRearMotorName("backRight")
        .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
        .leftRearMotorDirection(DcMotor.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
        .rightRearMotorDirection(DcMotor.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
