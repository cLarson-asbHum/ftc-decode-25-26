package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TODO: Figure out how to make the settings (e.g. max power) able to be confiured and updated in the builder
@Configurable
public class Constants {    
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.616 /* kg */);

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
        

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .distanceUnit(DistanceUnit.INCH)
        .forwardPodY(-0.75)
        .strafePodX(-3.75)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
