package org.firstinspires.ftc.teamcode.teleop;

// import com.qualcomm.robotcore.hardware
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArc;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection.Criterion;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.hardware.subsystem.*;
import org.firstinspires.ftc.teamcode.hardware.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.res.R;
import org.firstinspires.ftc.teamcode.util.AimbotManager;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ConvexHull;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.OpModeData;
import org.firstinspires.ftc.teamcode.util.Util;

@Configurable
@TeleOp(group="A - Main")
public class CompetitionTeleop extends OpMode {
    public static double TRIGGER_PRESSED = 0.1;
    
    // Auto aiming constants
    public static final double DISTANCE_OFFSET = -8; // Odom measures from center, but it should be from back of bot
    public static final double DIST_TOLERANCE = 0.4; // inches
    public static final double MIN_ANGLE = Robot.positionToRadians(0);
    public static final double MAX_ANGLE = Math.toRadians(62.5); // Any higher, and the shooting is inaccurate
    public static final double MAX_SPEED = Robot.ticksToInches(2400);

    // Auto firing constants
    public static final double ROBOT_WIDTH = 16; // In inches
    public static final double ROBOT_LENGTH = 18; // In inches
    public static final double MAX_DISPLACEMENT = 5; // In inches

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private Command fireAfterBlockers = null;

    private final ArrayList<String> nullDeviceNames = new ArrayList<>();
    private final ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private List<LynxModule> lynxModules = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private LinearHingePivot rampPivot = null;

    private Follower follower = null;
    private AimbotManager aimbot = null;
    private ConvexHull closeShootingZone = ConvexHull.of(new Pose[] {
        new Pose(0, 144),
        new Pose(72, 72),
        new Pose(144, 144)
    });
    private ConvexHull farShootingZone = ConvexHull.of(new Pose[] {
        new Pose(48, 0),
        new Pose(72, 24),
        new Pose(96, 0)
    });

    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;
    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;
    private ArtifactColorLed rightLed = null;
    private ArtifactColorLed leftLed = null;

    private boolean autoReloadEnabled = false;
    private boolean autoAimEnabled = true;
    private boolean autoFiringEnabled = true;
    private boolean showExtraTelemetry = true;

    private boolean wasPressingRightTrigger = false;
    private boolean wasPressingLeftTrigger = false;
    private boolean wasPressingLeftBumper = false;
    private boolean wasTogglingAutoReload = false;
    private boolean wasTogglingAimbot = false;
    private boolean wasTogglingAutoFiring = false;
    private boolean wasPressingIsRed = false;
    private boolean wasPressingX = false;
    private boolean wasPressingY = false;
    private boolean wasPressingA = false;
    private boolean wasPressingPark = false;
    private boolean autoFiringWasFeasible = false;

    private boolean updateStartPosition = false;
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
        showExtraTelemetry = !OpModeData.inCompetitonMode;
        CommandScheduler.getInstance().reset(); // Clear anything from before
        CommandScheduler.getInstance().registerSubsystem(robot.getAllSubsystems());

        // Creating the PedroPathing path follower
        if(OpModeData.follower == null) {
            follower = Constants.createFollower(hardwareMap);
            updateStartPosition = true;

            // Create a start position if we need one
            if(startPosition == null) {
                startPosition = new Pose(72, 72, 0);
            }
        } else {
            follower = OpModeData.follower;
            startPosition = follower.getPose();
            updateStartPosition = false; // Setting it twice can do weird things
        }

        // Bulk caching
        rightReload.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        leftReload.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for(final LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        
        // Finishing up
        aimbot = new AimbotManager(shooter, rampPivot, OpModeData.selection);
        if(OpModeData.selection == null) {
            try {
                aimbot.init(R.raw.arcs, this::filterArc, telemetry);
            } catch(IOException exc) {
                throw new RuntimeException(exc);
            }
        }
    }

    public void adjustSettings(Gamepad gamepad) {
        // Adjusting whether we are blue or red
        final boolean opModeDataIsRed = OpModeData.isRed;

        if(gamepad.aWasPressed()) {
            isRed = !isRed;
        }

        
        if(gamepad.bWasPressed()) {
            showExtraTelemetry = !showExtraTelemetry;
        }

        OpModeData.inCompetitonMode = !showExtraTelemetry;
        OpModeData.isRed = isRed;
        
        // Adjusting where we start the opmode
        if(updateStartPosition) {
            adjustStartPosition(gamepad);
        }
    }

    private void adjustStartPosition(Gamepad gamepad) {
        // X
        final double X_STEP = 6;
        if(gamepad.dpadRightWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() + X_STEP, 144));
        }

        if(gamepad.dpadLeftWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() - X_STEP, 144));
        }
        
        // Y
        final double Y_STEP = 6;
        if(gamepad.dpadUpWasPressed()) {
            startPosition = startPosition.withY(Util.clamp(0, startPosition.getY() + Y_STEP, 144));
        }
        
        
        if(gamepad.dpadDownWasPressed()) {
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

        OpModeData.startPosition = startPosition;
    }

    public void displaySettings() {
        final String format = Util.lines(
            "========== Settings ==========",
            "",
            "isRed: %b",
            "isCompetitionMode: %b",
            "",
            "startPos", 
            "    x: %.2f;",
            "    y: %.2f;",
            "    θ: %.2f;°",
            "",
            "========== Controls ==========",
            "",
            "Toggle isRed: A",
            "Toggle isCompetitionMode: B",
            "",
            "Add   X: →",
            "Minus X: ←",
            "",
            "Add   Y: ↑",
            "Minus Y: ↓",
            "",
            "Add   θ: LB",
            "Minus θ: RB",
            ""
        );

        telemetry.addLine(String.format(
            format, 
            isRed,
            !showExtraTelemetry,
            startPosition.getX(),
            startPosition.getY(),
            Math.toDegrees(startPosition.getHeading())
        ));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if(aimbot.isInitialized()) {
            OpModeData.selection = aimbot.getSelection();
            telemetry.addData("Status", "Initialized");
            telemetry.addLine();
            telemetry.addData("Selection Size", aimbot.getSelection().size());
            telemetry.addLine();
            adjustSettings(gamepad1);
            displaySettings();
            telemetry.update();
        }
    }

    @Override
    public void start() {
        rampPivot.runToAngle(rampPivot.convertFromPosition(0.66));
        
        if(showExtraTelemetry) {
            shooter.setTelemetry(telemetry);
        }

        if(updateStartPosition) {
            follower.setPose(startPosition);
        }

        if(OpModeData.follower == null) {
            OpModeData.follower = follower;
        }

        // follower.breakFollowing();

        timer.reset();
    }

    @Override
    public void loop() {
        final double timestamp = timer.seconds();
        final double deltaTime = timestamp - lastTime;
        lastTime = timestamp;

        // Driving the drivetrain
        if(!follower.isBusy()) {            
            drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        changeDrivetrainSpeed(
            gamepad1.left_trigger > TRIGGER_PRESSED || gamepad1.right_trigger > TRIGGER_PRESSED, // Fast
            gamepad1.left_bumper || gamepad1.right_bumper, // Slow
            gamepad1.left_bumper && gamepad1.right_bumper // Slowest, has precedent over the above
        );      
        
        reverseDrivingDirection(gamepad1.bWasPressed());

        // Team change
        toggleIsRed(gamepad1.back && gamepad1.a && !wasPressingIsRed);   

        // Automatic driving
        final boolean doPark = gamepad1.dpad_up && gamepad1.y;
        goToLoadingZone(gamepad1.a && !wasPressingA, gamepad1.a, !gamepad1.a && wasPressingA);
        goToShootingZone(gamepad1.x && !wasPressingX, gamepad1.x, !gamepad1.x && wasPressingX);
        faceGoal(
            gamepad1.y && !wasPressingY && !doPark, 
            gamepad1.y && !doPark, 
            (!gamepad1.y && wasPressingY) || (doPark && !wasPressingPark)
        );
        goParkForthwith(doPark && !wasPressingPark, doPark, !doPark && wasPressingPark);

        // Shooting the given color of artifact
        fireBasedOffColor(gamepad2.aWasPressed() /* Green */, gamepad2.xWasPressed() /* Purple */);

        // Firing the given side if anything goes wrong
        fireBasedOffSide(
            gamepad2.rightStickButtonWasPressed(), // Right
            gamepad2.leftStickButtonWasPressed() // Left
        );

        // MULTIFIRE (hold right trigger)
        fireIndiscriminantly(
            gamepad2.right_trigger > TRIGGER_PRESSED && !wasPressingRightTrigger, // Begin
            !(gamepad2.right_trigger > TRIGGER_PRESSED) && wasPressingRightTrigger // Cancel
        );

        // Firing automatically
        final boolean autoFiringIsFeasible = 
            autoFiringEnabled 
            && insideAnyShootingZone(follower.getPose(), ROBOT_WIDTH, ROBOT_LENGTH)
            && isAccuratelyFacingGoal(follower.getPose(), MAX_DISPLACEMENT);
        fireIndiscriminantly(
            autoFiringIsFeasible && shooter.getStatus() == Status.CHARGED,
            !(autoFiringIsFeasible && shooter.getStatus() == Status.FIRING) && autoFiringWasFeasible
        );
        toggleAutoFiring(gamepad2.back && gamepad2.left_trigger > TRIGGER_PRESSED && !wasTogglingAutoFiring);

        // AUTOAIM
        final BallisticArc arc = getArc(shooter.getStatus());
        followArc(autoAimEnabled && arc != null, arc, shooter.getStatus());
        toggleAutoAim(gamepad2.back && gamepad2.dpad_left && !wasTogglingAimbot);

        // RELOAD
        if(autoReloadEnabled) {
            // Because we assume that if we're reloading manually, something's wrong
            reloadBothSides(gamepad2.yWasPressed() && !gamepad2.back);
        } else {
            manualReloadEmptySides(gamepad2.yWasPressed() && !gamepad2.back);
        }

        // AUTO RELOAD
        // Performing the auto reload if allowed to AND we are fully charged
        // We check that we are charged so that we don't automaticcally exit, say, 
        // `UNCHARGING` and cause the shooter to instantly speed up again.
        autoReloadEmptySides(autoReloadEnabled && shooter.getStatus() == ShooterSubsystem.Status.CHARGED);

        // Toggling auto reload upon p
        toggleAutoReload(gamepad2.back && gamepad2.y && !wasTogglingAutoReload);

        // CHARGE
        chargeShooter(gamepad2.dpadUpWasPressed());
        
        // UNCHARGE
        unchargeShooter(gamepad2.dpadDownWasPressed());

        // ABORT
        bringArtifactsOutOfShooter(gamepad2.bWasPressed() && !gamepad2.start);

        // INATKE (hold)
        intakeFromFloor(
            gamepad2.left_trigger > TRIGGER_PRESSED && !wasPressingLeftTrigger, // Begin
            !(gamepad2.left_trigger > TRIGGER_PRESSED) && wasPressingLeftTrigger // End
        );
        
        ejectToFloor(
            gamepad2.left_bumper && !wasPressingLeftBumper, // Begin
            !gamepad2.left_bumper && wasPressingLeftBumper // End
        );

        // Blocking if we aren't firing
        if(shooter.getFiringState() == FlywheelTubeShooter.FiringState.UNKNOWN) {
            closeBlockers();
        }

        // LEDs
        final ArtifactColor artifactcolorL = leftReload.lastColor();
        final ArtifactColor artifactcolorR = rightReload.lastColor();

        colorLed(rightLed, artifactcolorR);
        colorLed(leftLed, artifactcolorL);

        // BUTTON PRESSES
        wasPressingRightTrigger = gamepad2.right_trigger > TRIGGER_PRESSED;
        wasPressingLeftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;
        wasPressingLeftBumper = gamepad2.left_bumper;
        wasTogglingAutoReload = gamepad2.back && gamepad2.y;
        wasTogglingAimbot     = gamepad2.back && gamepad2.dpad_left; 
        wasTogglingAutoFiring = gamepad2.back && gamepad2.left_trigger > TRIGGER_PRESSED; 
        wasPressingIsRed = gamepad1.back && gamepad1.a;
        wasPressingX = gamepad1.x;
        wasPressingY = gamepad1.y;
        wasPressingA = gamepad1.a;
        wasPressingPark = gamepad1.dpad_up && gamepad1.y;
        autoFiringWasFeasible = autoFiringIsFeasible;

        // TELELEMETRY
        telemetry.clearAll();
        telemetry.addData("deltaTime (seconds)", deltaTime);
        telemetry.addLine();
        telemetry.addLine(Util.header("Settings"));
        telemetry.addLine();
        telemetry.addData("autoReloadEnabled (Back2 + Y)", autoReloadEnabled);
        telemetry.addData("autoAimEnabled (Back2 + ←)", autoAimEnabled);
        telemetry.addLine();
        telemetry.addData("isRed (Back1 + A)", isRed);
        telemetry.addData("startPosition", startPosition);
        
        if(showExtraTelemetry) {
            telemetry.addLine();
            telemetry.addLine(Util.header("Sensors"));
            telemetry.addLine();
            telemetry.addData("leftReload Artifact", artifactcolorL);
            telemetry.addData("rightReload Artifact", artifactcolorR);
            telemetry.addData("currentPose", () -> {
                return follower.getPose();
            });
            telemetry.addData("shooting distance", () -> {
                return getDistance(follower.getPose(), KeyPoses.goalCenter(isRed));
            });
            telemetry.addData("in shooting zone", () -> {
                return insideAnyShootingZone(follower.getPose(), ROBOT_WIDTH, ROBOT_LENGTH);
            });
            
            if(arc != null) {
                telemetry.addLine("arc:");
                telemetry.addData("  |    dist ",  "%.1f in", Criterion.DISTANCE.of(arc));
                telemetry.addData("  |    angle",  "%.1f°", Math.toDegrees(Criterion.ANGLE.of(arc)));
                telemetry.addData("  |    speed",  "%.1f in s⁻¹", Criterion.SPEED.of(arc));
                telemetry.addData("  \\    time ", "%.3f s", arc.getElapsedTime());
            } else {
                telemetry.addData("arc", "null");
            }

            telemetry.addLine();
        }

        // COMMAND SCHEDULER
        follower.updatePose();
        leftReload.clearBulkCache();
        rightReload.clearBulkCache();
        for(final LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
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

    public Pose getClosestShootingPoint(Pose currentPose) {
        final Pose closeSide = closeShootingZone.closestPointTo(currentPose);
        final Pose farSide = farShootingZone.closestPointTo(currentPose);

        if(closeSide.distSquared(currentPose) <= farSide.distSquared(currentPose)) {
            // Equality is included, as we would rather should close in case of tie
            return closeSide;
        } else {
            return farSide;
        }
    }

    public double shootingAngleToGoal(Pose currentPose) {
        return AngleUnit.normalizeRadians(Math.PI + Math.atan2(
            KeyPoses.goalCenter(isRed).getY() - currentPose.getY(),
            KeyPoses.goalCenter(isRed).getX() - currentPose.getX()
        ));
    }

    public boolean goToShootingZone(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            // We check auto aim because having it disabled means we have to 
            // shoot from the same spot (or, more accurately, distance)
            Path path = null; // Placeholder

            if(autoAimEnabled) {
                // Getting the closest shooting location
                // The robot will choose between close-side and far-side (relative to the goal)
                final Pose currentPose = follower.getPose();
                final Pose closest = getClosestShootingPoint(currentPose);
                final double targetHeading = shootingAngleToGoal(closest);
                path = new Path(new BezierLine(currentPose, closest.withHeading(targetHeading))); 
                path.setConstantHeadingInterpolation(targetHeading);
            } else {
                path = new Path(new BezierLine(follower.getPose(), KeyPoses.shooting(isRed)));
                path.setConstantHeadingInterpolation(KeyPoses.shooting(isRed).getHeading());
            }
            
            follower.followPath(path, false);
        }
        
        final boolean continueFollowing = (doStart || doFollow) && follower != null;

        // Follow the current path, be it a line or just turning
        if(continueFollowing) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean faceGoal(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Pose currentPose = follower.getPose();
            follower.holdPoint(
                new BezierPoint(currentPose),
                shootingAngleToGoal(currentPose),
                false
            );
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
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.loading(isRed)));
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

    private Command asyncSleep(Runnable command, long ms) {
        final Command sleepCommand = new SequentialCommandGroup(
            new WaitCommand(ms), // FIXME: Time util?
            new InstantCommand(command) // Quote from an artifact: {d}
        );
        CommandScheduler.getInstance().schedule(sleepCommand);
        return sleepCommand;
    }

    public boolean closeBlockers() {
        final boolean closeLeft = leftBlocker.close();
        final boolean closeRight = rightBlocker.close();

        return closeLeft || closeRight;
    }

    public boolean fireBasedOffColor(boolean fireGreen, boolean firePurple) {
        if((fireGreen || firePurple) && fireAfterBlockers != null) {
            CommandScheduler.getInstance().cancel(fireAfterBlockers);
        }

        if(fireGreen) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            fireAfterBlockers = asyncSleep(() -> {
                shooter.fireGreen();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }
        
        if(firePurple) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            fireAfterBlockers = asyncSleep(() -> {
                shooter.firePurple();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireBasedOffSide(boolean fireRight, boolean fireLeft) {
        if((fireRight || fireLeft) && fireAfterBlockers != null) {
            CommandScheduler.getInstance().cancel(fireAfterBlockers);
        }

        if(fireRight) { // Quote from the artifact that hit my keyboard: 12erre
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            fireAfterBlockers = asyncSleep(() -> {
                shooter.fireRight();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        if(fireLeft) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            fireAfterBlockers = asyncSleep(() -> {
                shooter.fireLeft();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireIndiscriminantly(boolean startFiring, boolean cancelFiring) {
        if(startFiring && fireAfterBlockers != null) {
            CommandScheduler.getInstance().cancel(fireAfterBlockers);
        }

        if(startFiring) {
            openBlockers(FlywheelTubeShooter.FiringState.FIRING_BOTH);
            fireAfterBlockers = asyncSleep(() -> {
                shooter.multiFire();
                intake.intakeGamePieces();
                openBlockers(shooter.getFiringState());
            }, 500);
            return true;
        }

        if(cancelFiring) {
            intake.holdGamePieces();
            shooter.charge();
            CommandScheduler.getInstance().cancel(fireAfterBlockers);
        } 

        return false;
    }

    public double getDistance(Pose currentPose, Pose goal) {
        return DISTANCE_OFFSET + Math.hypot(
            currentPose.getX() - goal.getX(), 
            currentPose.getY() - goal.getY()
        );
    }

    public BallisticArc getArc(ShooterSubsystem.Status status) {
        switch(status) {
            case FIRING:
            case CHARGING:
            case RELOADING:
            case CHARGED: 
                // Getting the arc as a function of distance
                return aimbot.selectArcByDistance(
                    getDistance(follower.getPose(), KeyPoses.goalCenter(isRed)), 
                    DIST_TOLERANCE
                );
            
            default:
                // Nothing is returned
                return null;
        }
    }

    public boolean followArc(boolean doFollow, BallisticArc arc, ShooterSubsystem.Status status) {
        if(!doFollow) {
            return false;
        }

        switch(status) {
            case FIRING:
            case RELOADING:
            case CHARGED:
            case CHARGING:
                // Changing the pivot and the shooter speed
                aimbot.followArc(arc);
                return true;

            default: 
                // Autoaim is not allowed in the current state
                return false;
        }
    }

    public boolean insideAnyShootingZone(Pose currentPose, double robotWidth, double robotLength) {
        // Getting the profile of the robot.
        final double hWidth = 0.5 * robotWidth; // Half Width
        final double hLength = 0.5 * robotLength; // Half Length
        final ConvexHull robot = ConvexHull.of(new Pose[] {
            new Pose(-hWidth, -hLength).rotate(currentPose.getHeading(), false).plus(currentPose),
            new Pose( hWidth, -hLength).rotate(currentPose.getHeading(), false).plus(currentPose),
            new Pose( hWidth, hLength) .rotate(currentPose.getHeading(), false).plus(currentPose),
            new Pose(-hWidth, hLength) .rotate(currentPose.getHeading(), false).plus(currentPose)
        });

        // Checking if this is touching either shooting zone
        return closeShootingZone.intersects(robot) || farShootingZone.intersects(robot);
    }

    public boolean isAccuratelyFacingGoal(Pose currentPose, double maxDisplacement) {
        final double distance = currentPose.distSquared(KeyPoses.goalCenter(isRed));

        // The the difference in angle measure (guaranteed to be in range [0, pi])
        final double targetAngle = shootingAngleToGoal(currentPose);
        final double unnormError = currentPose.getHeading() - targetAngle;
        final double angleErr = Math.abs(AngleUnit.normalizeRadians(unnormError));

        // Using the law of cosines to get the displacement to the goal
        // The known side lengths are both equal to distance, i.e: a = b = distance
        // 
        // NOTE: The actual formula used is equivalent to the law of cosines,
        //       mathematically but is better for small angles 
        //       (refer to https://en.wikipedia.org/wiki/Law_of_cosines#Version_suited_to_small_angles)
        return 2 * distance * Math.sin(0.5 * angleErr) <= maxDisplacement;
    }

    public boolean toggleAutoFiring(boolean doToggle) {
        if(doToggle) {
            autoFiringEnabled = !autoFiringEnabled;
        }

        return false;
    }

    public boolean toggleAutoAim(boolean doToggle) {
        // Resetting a few parameters if we disable aimbot
        if(doToggle && autoAimEnabled) {
            autoAimEnabled = false;
            rampPivot.runToAngle(rampPivot.convertFromPosition(0.66));
            switch(shooter.getStatus()) {
                case UNCHARGED:
                case UNCHARGING:
                    // Keep the shooter uncharged
                    break;
                
                default:
                    // charge the shooter
                    shooter.charge();
                    break;
            }
            return true;
        }

        // Enabling auto reload. 
        if(doToggle && !autoAimEnabled) {
            autoAimEnabled = true;
            return true;
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

        try {
            aimbot.close();
        } catch(IOException exc) {
            // If we get an exception... that sucks! We would rather continue to clean up
        } 

        aimbot = null;
    }

    private boolean filterArc(BallisticArc arc) {
        final double theta = Criterion.ANGLE.of(arc);
        
        // Filter based off angle
        if(!(MIN_ANGLE <= theta && theta <= MAX_ANGLE)) {
            return false;
        }

        final double speed = Criterion.SPEED.of(arc);
        return speed <= MAX_SPEED;
    }
}