package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.ConfigPose;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;
import org.firstinspires.ftc.teamcode.util.MotifWebcam;
import org.firstinspires.ftc.teamcode.util.RrCoordinates;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

@Configurable
@Autonomous(name = "Auto2: Blue Rippley", group = "A - Main")
public class Rippley extends LinearOpMode {

    private double ticksPerInches = 10331 / 30;
    private double ticksPerDegree = (-35248 - 12419) / (3 * 360);

    private void Turn(double ang) {
        frontLeft.setPower(Math.signum(ang) * mPWR);
        frontRight.setPower(Math.signum(ang) * -mPWR);
        backLeft.setPower(Math.signum(ang) * mPWR);
        backRight.setPower(Math.signum(ang) * -mPWR);

        startDeer = (backLeft.getCurrentPosition());

        while (!(Math.abs(-backLeft.getCurrentPosition()
                - startDeer - tickToDeg(ang) - 100) < 50)) {
            telemetry.addData("startDeer", startDeer);
            telemetry.addData("CurAngle", imu.getRobotYawPitchRollAngles().getRoll());
            telemetry.addLine("DUCK");
            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backLeft.getCurrentPosition());
            telemetry.addData("Time", (int) sleepTime(mPWR));
            telemetry.update();
        }

        frontLeft.setPower(-mPWR);
        frontRight.setPower(mPWR);
        backLeft.setPower(-mPWR);
        backRight.setPower(mPWR);

        timeUtil.sleep((int) 50);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void MoveForward(double dist) {
        frontLeft.setPower(mPWR);
        frontRight.setPower(mPWR);
        backLeft.setPower(mPWR);
        backRight.setPower(mPWR);
        startDeer = (backLeft.getCurrentPosition());

        while (!(Math.abs(-backLeft.getCurrentPosition() - tickToIn(dist) - startDeer) < 100
        // true
        // was 30 (avg 36)
        // was 27 (avg 34)
        // was 25
        // was 21
        // was 23
        // 29.5 works on 0.3333 speed! :)
        )) {
            telemetry.addLine("Fork Knife");
            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backLeft.getCurrentPosition());
            telemetry.addData("Time", (int) sleepTime(mPWR));
            telemetry.update();
        }

        frontLeft.setPower(-mPWR);
        frontRight.setPower(-mPWR);
        backLeft.setPower(-mPWR);
        backRight.setPower(-mPWR);

        timeUtil.sleep((int) 50);

        frontLeft.setPower(-0);
        frontRight.setPower(-0);
        backLeft.setPower(-0);
        backRight.setPower(-0);
    }

    private ElapsedTime timer = new ElapsedTime();
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private IMU imu = null;
    private double startDeer = 0;

    private double sleepTime(double mPWR) {
        return (200 / 3) * (mPWR - 0.3) + 83;
    }

    private double mPWR = 0.3333;

    public static int GAIN = 200;
    public static int EXPOSURE_MS = 33;

    // TODO: find the robot width and length
    public static double ROBOT_LENGTH = 17; // Inches parallel to the robot's forward-facing axis
    public static double ROBOT_WIDTH = 17; // Inches perpendicular to the robot's forward-facing axis 

    public static double CAMERA_YAW_OFFSET = 0; // In radians

    public static ConfigPose START_POSITION = new ConfigPose(
        // In Inches. Robot is touching the Blue goal
        goalEdgeXFromY(48 + ROBOT_WIDTH) + ROBOT_LENGTH / 2,

        // In Inches. Robot is overlapping tile edge 1
        48 + ROBOT_WIDTH / 2,

        // In Radians. Robot is facing the Blue Alliance
        0, 
        RrCoordinates.INSTANCE
    );

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(30);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        final DcMotorEx rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        final ColorRangeSensor rightReloadSensor = hardwareMap.get(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = hardwareMap.get(ColorRangeSensor.class, "leftReload");
        final WebcamName obeliskViewerCam = hardwareMap.get(WebcamName.class, "obeliskViewer");
        
        setManualExposure(obeliskViewerCam, GAIN, EXPOSURE_MS);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);

        final ArtifactColorRangeSensor rightReload = new ArtifactColorRangeSensor(rightReloadSensor);
        final ArtifactColorRangeSensor leftReload = new ArtifactColorRangeSensor(leftReloadSensor);
        final FlywheelTubeShooter shooter = new FlywheelTubeShooter.Builder(rightShooterMotor)
                .setLeftFeeder(leftFeeder)
                .setRightFeeder(rightFeeder)
                .setRightReloadClassifier(rightReload)
                .setLeftReloadClassifier(leftReload)
                .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final MotifWebcam motifGetter = new MotifWebcam(obeliskViewerCam, CAMERA_YAW_OFFSET);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        CommandScheduler.getInstance().registerSubsystem(shooter, intake);
        
        waitForStart();

        // Get the motif 
        motifGetter.setGlobalRobotYaw(START_POSITION.yaw);
        Motif motif = motifGetter.getMotif();
        motifGetter.disable(); // Save bandwidth and performance by not accessing the camera

        // If the motif coul dnt be found, use a default
        if(motif == null) {
            motif = Motif.FIRST_GREEN;
        }

        // MoveForward(6);

        // Firing the artifacts we have, using the motif from the april tag
        boolean hasFiredPurple = false;
        for(final ArtifactColor color : motif) {
            runUntilCompleted(shooter.chargeCommand());

            // Firing the indicated color
            switch(color) {
                case GREEN: 
                    shooter.fireGreen();
                    break;
                case PURPLE:
                    hasFiredPurple = true;
                    shooter.firePurple();
                    break;
                default:
                    throw new RuntimeException("Encountered unfirable ArtifactColor: " + color.name());
            }

            // Waiting for the firing to end
            // The shooter is likely to charge after this, but we want to wait until after reloading
            // to do any extra charging (for saving time).
            runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.FIRING));

            // Getting ready for reloading by cycling the next artifact into position
            // and taking note of what colors are already reloaded.
            ArtifactColor rightColor = null;
            ArtifactColor leftColor = null;

            if(hasFiredPurple) {
                intake.intakeGamePieces();
                rightColor = rightReload.getColor();
                leftColor = leftReload.getColor();
            }

            // Reload both sides if both are empty
            // We do this to ensure *something* is reloaded
            if(hasFiredPurple && leftColor == ArtifactColor.UNKNOWN && rightColor == ArtifactColor.UNKNOWN) {
                shooter.reload();
            }

            // Reload the left if it is empty and the other is green
            if(hasFiredPurple && leftColor == ArtifactColor.UNKNOWN && rightColor == ArtifactColor.GREEN ) {
                shooter.reloadLeft();
            }
            
            // Reload the right if it is empty and the other is green
            if(hasFiredPurple && leftColor == ArtifactColor.GREEN && rightColor == ArtifactColor.UNKNOWN) {
                shooter.reloadRight();
            }

            // Wait for the shooter to finish reloading and become charged again
            // Reloading naturally will cause the shooter to charge again, so this 
            // covers in case enough shooter velocity was lost when shooting
            runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.CHARGING
                    && shooter.getStatus() != Status.RELOADING));

            // If the charging failed, just tell it that it is charged, and move on
            if(shooter.getStatus() == Status.UNCHARGING) {
                shooter.forceCharged();
            }
        } 
        hasFiredPurple = false; // This will save us if we copy and paste our stuff above.

        // MoveForward(21);
        // Turn(90);
        // MoveForward(45);
        // Turn(90);
        // MoveForward(30);
        // Turn(-100);

    }

    private double tickToDeg(double ticks) {
        return ticks * ticksPerDegree;
    }

    private double tickToIn(double num) {
        return num * ticksPerInches;
    }

    private void runUntilCompleted(Command command) {
        CommandScheduler.getInstance().schedule(command);
        while(!command.isFinished() && opModeIsActive()) {
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
    }

    private void setManualExposure(WebcamName camera, int gain, int exposure) {}

    /**
     * Calculates the x so that the point (x, y) is along the edge of the goal.
     * This is used to find the x-coordinate a robot resting against the goal.
     * 
     * @param y The y coordinate that is along the edge of the goal.
     * @return The x coordinate so that (x, y) is along the edge of the goal.
     */
    private static double goalEdgeXFromY(double y) {
        final double RAMP_WIDTH = 6.75; // Inches
        final double GOAL_LENGTH_Y = 21.75; // Inches along the field y axis, up to the archway
        final double GOAL_LENGTH_X = 22.75; // Inches along the field x axis
        return Util.lerp(
            RAMP_WIDTH, 
            Util.invLerp(72 - GOAL_LENGTH_Y, y, 72), 
            RAMP_WIDTH + GOAL_LENGTH_X
        );
    }

    private final TimeInjectionUtil timeUtil = new TimeInjectionUtil(this);
}
