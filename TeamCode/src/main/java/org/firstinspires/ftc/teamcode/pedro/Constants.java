package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.util.Util;

// TODO: Figure out how to make the settings (e.g. max power) able to be confiured and updated in the builder
@Configurable
public class Constants {    
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.616 /* kg */)
        .forwardZeroPowerAcceleration(Util.avg(new double[] { -58.85349766006428,  -56.15420688364663, -55.95829214330413, -54.66555398023172, -54.925725637841396 }))
        .lateralZeroPowerAcceleration(Util.avg(new double[] { 
            -72.64969465750373, -79.11049228489789, -82.25602891741310, -73.77492763653832, -75.76698473910751, 
            -77.36451515801400, -69.28048582548735, -83.27346236321836, -72.48919310515740, -77.57216798938164 }))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(
            0.01,   // P
            0.01,   // I
            0.0005, // D
            0.6,    // T
            0.07188 // F
        ))
        .headingPIDFCoefficients(new PIDFCoefficients(
            0.8,    // P
            0.2,    // I
            0.1,    // D,
            0.0254  // F
        ))
        .translationalPIDFCoefficients(new PIDFCoefficients(
            0.1,    // P
            0.0003, // I
            0.01,   // D
            0       // F
        ))
        .centripetalScaling(0.000875)
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
        .rightRearMotorDirection(DcMotor.Direction.FORWARD)
        .xVelocity(Util.avg(new double[] { 60.018213106891324, 59.74428780623308, 59.97250017781894, 59.969493715781894, 59.564458531657536 }))
        .yVelocity(Util.avg(new double[] { 48.713226198211416, 49.35809157964751, 48.47607902466781, 48.779553511011315, 48.89836553138073 }))
        ;
        

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .distanceUnit(DistanceUnit.INCH)
        .forwardPodY(-0.75)
        .strafePodX(-3.75)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
        /* double tValueConstraint, 
        double velocityConstraint, 
        double translationalConstraint, 
        double headingConstraint, 
        double timeoutConstraint, 
        double brakingStrength, 
        int BEZIER_CURVE_SEARCH_LIMIT, 
        double brakingStart */
        0.995,
        10, // in/s
        2, // inches
        Math.toRadians(100), // radians
        100, 
        0.65, 
        10,
        1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
