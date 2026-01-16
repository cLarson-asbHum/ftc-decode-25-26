package org.firstinspires.ftc.teamcode.teleop;

// import com.qualcomm.robotcore.hardware
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.hardware.subsystem.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;

@Configurable
@TeleOp(group="A - Main")
public class CompetitionTeleop extends OpMode {
    public static double TRIGGER_PRESSED = 0.1;

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private List<LynxModule> lynxModules = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private LinearHingePivot rampPivot = null;
    private Follower follower = null;

    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;
    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;
    private ArtifactColorLed rightLed = null;
    private ArtifactColorLed leftLed = null;

    private boolean autoReloadEnabled = true;
    private boolean showExtraTelemetry = true;

    private boolean wasPressingRightTrigger = false;
    private boolean wasPressingLeftTrigger = false;
    private boolean wasPressingLeftBumper = false;
    private boolean wasTogglingAutoReload = false;
    private boolean wasPressingIsRed = false;
    private boolean wasPressingX = false;
    private boolean wasPressingA = false;
    private boolean wasPressingPark = false;

    private Pose startPosition = null;
    private boolean isRed = false;

    /**
     * Initializing the opmode. This is not expected to be HardwareFaker 
     * compatible.
     */
    @Override
    public void init() {
        final Robot robot = new Robot(hardwareMap, Robot.teleopDevices());
        shooter      = robot.getShooter();
        intake       = robot.getIntake();
        drivetrain   = robot.getDrivetrain();
        leftBlocker  = robot.getLeftBlocker();
        rightBlocker = robot.getRightBlocker();
        rampPivot    = robot.getRampPivot();
        leftReload   = robot.getLeftReload();
        rightReload  = robot.getRightReload();

        // Doing some work with the subsystems
        shooter.setTelemetry(telemetry);
        CommandScheduler.getInstance().reset(); // Clear anything from before
        CommandScheduler.getInstance().registerSubsystem(robot.getAllSubsystems());

        // Creating the PerdoPathing path follower
        follower = Constants.createFollower(hardwareMap);

        // Bulk caching
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for(final LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        
        rightReload.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        leftReload.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Finishing up
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void adjustSettings(Gamepad gamepad) {
        // Adjusting whether we are blue or red
        final Boolean blackboardIsRed = (Boolean) blackboard.get("isRed");
        isRed = blackboardIsRed == null ? false : blackboardIsRed.booleanValue();

        if(gamepad.aWasPressed()) {
            isRed = !isRed;
        }

        blackboard.put("isRed", isRed);
        
        // Adjusting where we start the opmode
        startPosition = (Pose) blackboard.get("startPosition");
        if(startPosition == null) {
            startPosition = new Pose(72, 72, 0);
        }
        
        // X
        final double X_STEP = 6;
        if(gamepad.dpadUpWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() + X_STEP, 144));
        }

        if(gamepad.dpadDownWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() - X_STEP, 144));
        }
        
        
        // Y
        final double Y_STEP = 6;
        if(gamepad.dpadRightWasPressed()) {
            startPosition = startPosition.withY(Util.clamp(0, startPosition.getY() + Y_STEP, 144));
        }
        
        
        if(gamepad.dpadLeftWasPressed()) {
            startPosition = startPosition.withY(Util.clamp(0, startPosition.getY() - Y_STEP, 144));
        }

        // θ
        final double YAW_STEP = Math.toRadians(15);
        if(gamepad.leftBumperWasPressed()) {
            final double newYaw = AngleUnit.normalizeRadians(startPosition.getHeading() + YAW_STEP);
            startPosition = startPosition.withHeading(newYaw);
        }
        
        
        if(gamepad.rightBumperWasPressed()) {
            final double newYaw = AngleUnit.normalizeRadians(startPosition.getHeading() - YAW_STEP);
            startPosition = startPosition.withHeading(newYaw);
        }

        blackboard.put("startPosition", startPosition);
    }

    public void displaySettings() {
        final String format = Util.lines(
            "",
            "========== Settings ==========",
            "",
            "isRed: %b",
            "",
            "startPos", 
            "    x: %.2f;",
            "    y: %.2f;",
            "    θ: %.2f;°",
            "",
            "========== Controls ==========",
            "",
            "Toggle isRed: A",
            "",
            "Add   X: dpad_up",
            "minus X: dpad_down",
            "",
            "Add   Y: dpad_up",
            "Minus Y: dpad_left",
            "",
            "Add   θ: left_bumper",
            "Minus θ: right_bumper",
            ""
        );

        telemetry.addLine(String.format(
            format, 
            isRed,
            startPosition.getX(),
            startPosition.getY(),
            Math.toDegrees(startPosition.getHeading())
        ));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        adjustSettings(gamepad1);
        displaySettings();
    }

    @Override
    public void start() {
        rampPivot.runToAngle(rampPivot.convertFromPosition(0.66));

        // Calling this method multiple times on the same reference will
        // cause any calls after the first one to be ignored.
        follower.setStartingPose(startPosition);
        timer.reset();
    }

    @Override
    public void loop() {
        final double timestamp = timer.seconds();
        final double deltaTime = timestamp - lastTime;
        lastTime = timestamp;

        // MANUAL DRIVING            
        drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        changeDrivetrainSpeed(
            gamepad1.left_trigger > TRIGGER_PRESSED || gamepad1.right_trigger > TRIGGER_PRESSED, // Fast
            gamepad1.left_bumper || gamepad1.right_bumper, // Slow
            gamepad1.left_bumper && gamepad1.right_bumper // Slowest, has precedent over the above
        );      
        reverseDrivingDirection(gamepad1.bWasPressed());
        
        // Team change
        toggleIsRed(gamepad1.back && gamepad1.a && !wasPressingIsRed);   

        // Automatic driving
        // goToShootingZone(gamepad1.x && !wasPressingX, gamepad1.x, !gamepad1.x && wasPressingX);
        // goToLoadingZone(gamepad1.a && !wasPressingA, gamepad1.a, !gamepad1.a && wasPressingA);
        goParkForthwith(
            gamepad1.dpad_up && gamepad1.y && !wasPressingPark,
            gamepad1.dpad_up && gamepad1.y, 
            !(gamepad1.dpad_up && gamepad1.y) && wasPressingPark
        );

        // FIRING: color, side, and multi
        fireBasedOffColor(gamepad2.aWasPressed() /* Green */, gamepad2.xWasPressed() /* Purple */);
        fireBasedOffSide(
            gamepad2.rightStickButtonWasPressed(), // Right
            gamepad2.leftStickButtonWasPressed() // Left
        );
        fireIndiscriminantly(
            gamepad2.right_trigger > TRIGGER_PRESSED && !wasPressingRightTrigger, // Begin
            !(gamepad2.right_trigger > TRIGGER_PRESSED) && wasPressingRightTrigger // Cancel
        );

        // MANUAL RELOAD
        if(autoReloadEnabled) {
            // Because we assume that if we're reloading manually, something's wrong
            reloadBothSides(gamepad2.yWasPressed());
        } else {
            manualReloadEmptySides(gamepad2.yWasPressed());
        }

        // AUTO RELOAD
        // Performing the auto reload if allowed to AND we are fully charged
        // We check that we are charged so that we don't automaticcally exit, say, 
        // `UNCHARGING` and cause the shooter to instantly speed up again.
        autoReloadEmptySides(autoReloadEnabled && shooter.getStatus() == ShooterSubsystem.Status.CHARGED);
        toggleAutoReload(gamepad2.back && gamepad2.y && !wasTogglingAutoReload);

        // CHARGE / UNCHARGE
        chargeShooter(gamepad2.dpadUpWasPressed());
        unchargeShooter(gamepad2.dpadDownWasPressed());

        // ABORT
        bringArtifactsOutOfShooter(gamepad2.bWasPressed());

        // INTAKE / EJECT
        intakeFromFloor(
            gamepad2.left_trigger > TRIGGER_PRESSED && !wasPressingLeftTrigger, // Begin
            !(gamepad2.left_trigger > TRIGGER_PRESSED) && wasPressingLeftTrigger // End
        );
        ejectToFloor(
            gamepad2.left_bumper && !wasPressingLeftBumper, // Begin
            !gamepad2.left_bumper && wasPressingLeftBumper // End
        );

        // BLOCKING
        if(shooter.getFiringState() == FlywheelTubeShooter.FiringState.UNKNOWN) {
            closeBlockers();
        }

        // LEDs
        final ArtifactColor artifactcolorL = leftReload.lastColor();
        final ArtifactColor artifactcolorR = rightReload.lastColor();
        colorLed(rightLed, artifactcolorR);
        colorLed(leftLed, artifactcolorL);

        // FINISHING UP
        updateButtonPresses();
        logTelemetry(deltaTime, artifactcolorL, artifactcolorR);
        leftReload.clearBulkCache();
        rightReload.clearBulkCache();
        CommandScheduler.getInstance().run();
    }

    private void updateButtonPresses() {
        wasPressingRightTrigger = gamepad2.right_trigger > TRIGGER_PRESSED;
        wasPressingLeftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;
        wasPressingLeftBumper = gamepad2.left_bumper;
        wasTogglingAutoReload = gamepad2.back && gamepad2.y;
        wasPressingIsRed = gamepad1.back && gamepad1.a;
        wasPressingX = gamepad1.x;
        wasPressingA = gamepad1.a;
        wasPressingPark = gamepad1.dpad_up && gamepad1.y;
    }

    private void logTelemetry(double deltaTime, ArtifactColor l, ArtifactColor r) {
        telemetry.clearAll();
        telemetry.addData("deltaTime (seconds)", deltaTime);
        telemetry.addLine();
        telemetry.addLine(Util.header("Settings"));
        telemetry.addLine();
        telemetry.addData("autoReloadEnabled (Back2 + Y)", autoReloadEnabled);
        telemetry.addLine();
        telemetry.addData("isRed (Back1 + A)", isRed);
        telemetry.addData("startPosition", startPosition);
        
        if(showExtraTelemetry) {
            telemetry.addLine();
            telemetry.addLine(Util.header("Sensors"));
            telemetry.addLine();
            telemetry.addData("leftReload Artifact", l);
            telemetry.addData("rightReload Artifact", r);
            telemetry.addData("currentPose", () -> {
                follower.updatePose();
                return follower.getPose();
            });
            telemetry.addLine();
        }
    }

    public boolean toggleIsRed(boolean doToggle) {
        if(doToggle) {
            isRed = !isRed;
            return true;
        }

        return false;
    }

    /**
     * Sets the drivetrain speed to be abnormal when eithe of the 
     * parameters are true. If both `goFast` and `goSlow` are true, this 
     * goes fast. This goes at normal speed if ano only if both `goFast` and 
     * `goSlow` are false.
     * 
     * @param goFast The drivetrain goes fast only when this is true
     * @param goSlow The drivetrain goes fast only when this is false
     * @return Whether the drivetrain speed was not at middle (normal) speed
     */
    public boolean changeDrivetrainSpeed(boolean goFast, boolean goSlow, boolean goSnail) {
        if(goSnail) {
            drivetrain.engageSlowestMode();
            return false;
        }

        if(goFast) {
            drivetrain.engageFastMode();
            return false;
        }
        
        if(goSlow) {
            drivetrain.engageSlowMode();
            return false;
        }
        
        // Normal Speed
        drivetrain.engageMiddleMode();
        return true;
    }

    public boolean goParkForthwith(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.base(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean goToShootingZone(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.shooting(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean goToLoadingZone(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.loading(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean openBlockers(FlywheelTubeShooter.FiringState firingState) {
        switch(firingState) {
            case FIRING_BOTH:
                leftBlocker.open();
                rightBlocker.open();
                return true;

            case FIRING_LEFT:
                leftBlocker.open();
                rightBlocker.close();
                return true;

            case FIRING_RIGHT:
                leftBlocker.close();
                rightBlocker.open();
                return true;

            default:
                return false;
        }
    }

    private void asyncSleep(Runnable command, double ms) {
        final ElapsedTime timer = new ElapsedTime() ;// FIXME: timeutil
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new Command() {
                @Override
                public Set<Subsystem> getRequirements() {
                    return new HashSet<Subsystem>();
                }

                @Override
                public boolean isFinished() {
                    return timer.seconds() * 1000 > ms;
                }

                @Override
                public void initialize() {
                    timer.reset();
                }
            },
            new InstantCommand(command) // Quote from an artifact: {d}
        ));
    }

    public boolean closeBlockers() {
        final boolean closeLeft = leftBlocker.close();
        final boolean closeRight = rightBlocker.close();

        return closeLeft || closeRight;
    }

    public boolean fireBasedOffColor(boolean fireGreen, boolean firePurple) {
        if(fireGreen) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            asyncSleep(() -> {
                shooter.fireGreen();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }
        
        if(firePurple) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            asyncSleep(() -> {
                shooter.firePurple();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireBasedOffSide(boolean fireRight, boolean fireLeft) {
        if(fireRight) { // Quote from the artifact that hit my keyboard: 12erre
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            asyncSleep(() -> {
                shooter.fireRight();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }
        if(fireLeft) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            asyncSleep(() -> {
                shooter.fireLeft();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireIndiscriminantly(boolean startFiring, boolean cancelFiring) {
        if(startFiring) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            asyncSleep(() -> {
                shooter.multiFire();
                intake.intakeGamePieces();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        if(cancelFiring) {
            shooter.charge();
        } 

        return false;
    }

    public boolean reloadBothSides(boolean doReload) {
        if(doReload) {
            shooter.reload();
            return true;
        }

        return false;
    }

    public boolean manualReloadEmptySides(boolean doReload) {
        if(doReload) {
            shooter.reloadEmpty();
            return true;
        }

        return false;
    }

    public boolean toggleAutoReload(boolean doToggle) {
        if(doToggle) {
            autoReloadEnabled = !autoReloadEnabled;
            return true;
        }

        return false;
    }

    public boolean autoReloadEmptySides(boolean doReload) {
        if(doReload) {
            shooter.autoReload();
            return true;
        }

        return false;
    }

    public boolean chargeShooter(boolean doCharge) {
        if(doCharge) {
            shooter.charge();
            return true;
        }

        return false;
    }

    public boolean unchargeShooter(boolean doUncharge) {
        if(doUncharge) {
            shooter.uncharge();
            return true;
        } 

        return false;
    }

    public boolean bringArtifactsOutOfShooter(boolean doBringBack) {
        if(doBringBack) {
            shooter.abort();
            return true;
        }

        return false;
    }

    public boolean intakeFromFloor(boolean startIntake, boolean cancelIntake) {
        if(startIntake) {
            intake.intakeGamePieces();
            return true;
        }
        
        if(cancelIntake) {
            intake.holdGamePieces();
        } 

        return false;
    }

    public boolean ejectToFloor(boolean startEject, boolean cancelEject) {
        if(startEject) {
            intake.ejectGamePieces();
            return true;
        } 
        
        if(cancelEject) {
            intake.holdGamePieces();
        }

        return false;
    }

    public boolean reverseDrivingDirection(boolean doReverse) {
        if(doReverse) {
            drivetrain.reverse();
            return true;
        }

        return false;
    }

    public boolean colorLed(ArtifactColorLed led, ArtifactColor color) {
        if(led == null) {
            return false;
        }

        return led.color(color);
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}