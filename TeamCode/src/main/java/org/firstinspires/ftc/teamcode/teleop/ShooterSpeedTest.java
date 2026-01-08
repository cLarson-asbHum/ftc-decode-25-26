package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.subsystem.PivotSubsystem;

@TeleOp(group="B - Testing")
public class ShooterSpeedTest extends OpMode {

    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();    

    private FlywheelTubeShooter shooter = null;
    private DoubleUnaryOperator inchesToTicks = null;
    private PivotSubsystem pivot = null;

    private boolean areControlsHidden = true;

    private double targetSpeed = 0;
    private double newTargetSpeed = 0;
    private DcMotorEx shootingMotor = null;

    /**
     * Attempts to get the given hardware from the hardwareMap. If it cannot be 
     * found, then it returns null without finding an error.
     * 
     * This method should be used instead of hardwareMap.get() because it allows
     * us to see **all** the hardware that we cannot find.
     * 
     * @return The hardware with that name, or null if it cannot be found.
     */
    private <T extends HardwareDevice> T findHardware(Class<T> hardwareType, String name) {
        final T result = hardwareMap.tryGet(hardwareType, name);

        // Adding it to the list if null
        if(result == null) {
            nullDeviceNames.add(name);
            nullDeviceTypes.add(hardwareType);
        }

        return result;
    }

    /**
     * Throws an exception if any devices are in the nullDeviceNames or 
     * nullDeviceTypes lists. The thrown exception contains the names and types 
     * of all null hardware devices. 
     */
    private void throwAFitIfAnyHardwareIsNotFound() {
        if(nullDeviceNames.size() != 0 || nullDeviceTypes.size() != 0) {
            String concat = "";

            for(int i = 0; i < nullDeviceNames.size() || i < nullDeviceNames.size(); i++) {
                final String name = nullDeviceNames.get(i);
                final Class type = nullDeviceTypes.get(i); 
                concat += "\n    ";

                if(name != null) {
                    concat += '"' + name + '"';
                } else {
                    concat += "[null]";
                }

                concat += " with type ";
                
                if(type != null) {
                    concat += type.getName() + ".class";
                } else {
                    concat += "[null]";
                }
            }

            throw new RuntimeException("Cannot find hardware:" + concat);
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        // Find and create all of the hardware. This uses the hardware map. 
        // When using unit tests, the `hardwareMap` field can be set for dependency injection.
        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");

        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        final DistanceSensor rightDistanceSensor = findHardware(DistanceSensor.class, "rightDistance");
        final DistanceSensor leftDistanceSensor = findHardware(DistanceSensor.class, "leftDistance");
        
        final ServoImplEx rampPivotServo = (ServoImplEx) findHardware(Servo.class, "rampPivot");

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFeederServo.setDirection(DcMotor.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        rampPivotServo.setPwmRange(new PwmRange(1050, 1950));

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        // (even if nothing is achieved, per se).
        final ArtifactColorRangeSensor rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor,
            rightDistanceSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.60, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}
        );
        final ArtifactColorRangeSensor leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            leftDistanceSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.40, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}

        );

        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor/* , rightShooterMotor */);
        shootingMotor = flywheels;
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .setTicksToInches(this::ticksToInches)
            .setInchesToTicks(inchesToTicks = this::inchesToTicks)
            .build();

        this.shooter = rightShooter;

        
        // NOTE: This is a default tuning, from 27 Dec 2025 at 1:28 PM
        final HashMap<Double, Double> pretunedData = new HashMap<>() {{
            final double[] dists = new double[] {
                0.00, 0.03, 0.06, 0.08,   0.11, 0.13, 0.16, 0.18, 
                0.20, 0.24, 0.26, 0.29,   0.31, 0.36, 0.40, 0.42, 
                0.46, 0.49, 0.52, 0.54,   0.56, 0.58, 0.60, 0.62, 
                0.65, 0.70, 0.72, 0.75,   0.77, 0.80, 0.83, 0.86,
                0.88, 0.90, 0.94, 0.98,   1.00
            };
            
            double angle = 37.0;
            for(final double dist : dists) {
                put(dist, Math.toRadians(angle));
                angle++;
            }
        }};
        final LinearInterpolator positionToRadians = new LinearInterpolator(pretunedData);
        final LinearInterpolator radiansToPosition = positionToRadians.inverse();
        pivot = new LinearHingePivot.Builder(rampPivotServo)
            .setPositionToRadians((pos) -> positionToRadians.clampedCalculate(pos))
            .setRadiansToPosition((ang) -> radiansToPosition.clampedCalculate(ang))
            .build();

        telemetry.setMsTransmissionInterval(33);

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().registerSubsystem(rightShooter, pivot);
    }

    public double ticksToInches(double ticks) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return K + B * Math.pow(Math.log(ticks - H), A);
    }

    public double inchesToTicks(double inches) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return H + Math.exp(Math.pow((inches - K) / B, 1 / A));
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Intialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Not much to do
    }

    private void changePivotAngle() {
        double newPosition = pivot.getCurrentAngle();

        if(gamepad1.dpadUpWasPressed()) {
            newPosition += Math.toRadians(1);
        }

        if(gamepad1.dpadDownWasPressed()) {
            newPosition -= Math.toRadians(1);
        }

        newPosition = Util.clamp(Math.toRadians(37.0), newPosition, Math.toRadians(73.0));
        pivot.runToAngle(newPosition, Math.toRadians(0.25));
    }

    @Override
    public void loop() {
        // Handling button presses
        if(gamepad1.backWasPressed()) {
            areControlsHidden = !areControlsHidden;
        }

        double increment = 20;
        if(gamepad1.left_trigger > 0.1) {
            increment *= 0.5;
        }

        if(gamepad1.right_trigger > 0.1) {
            increment *= 2;
        }

        if(gamepad1.rightBumperWasPressed()) {
            newTargetSpeed += increment;
        }
        
        if(gamepad1.leftBumperWasPressed()) {
            newTargetSpeed -= increment;
        }

        if(gamepad1.yWasPressed()) {
            targetSpeed = newTargetSpeed;
            shooter.charge(targetSpeed, true);
        }

        if(gamepad1.xWasPressed()) {
            shooter.fire();
        }
        
        if(gamepad1.bWasPressed()) {
            shooter.uncharge();
        }

        changePivotAngle();
        
        // Controls
        if(!areControlsHidden) {
            telemetry.addLine(Util.header("Controls (Back to hide)"));
            telemetry.addData("Accel shooter", "RB");
            telemetry.addData("Decel shooter", "LB");
            telemetry.addLine();
            telemetry.addData("Fire", "X");
            telemetry.addData("Stop", "B");
            telemetry.addData("Update accel", "Y");
            telemetry.addLine();
            telemetry.addData("Fast mode", "RT");
            telemetry.addData("Slow mode", "LT");
            telemetry.addLine();
            telemetry.addData("Raise pivot", "↑");
            telemetry.addData("Lower pivot", "↓");
        } else {
            telemetry.addLine(Util.header("Controls (Back to show)"));
        }

        telemetry.addLine();
        telemetry.addLine(Util.header("Shooting"));
        telemetry.addLine();

        if(newTargetSpeed != targetSpeed) {
            telemetry.addData(
                "New target", "%.1f in s⁻¹ (%.0f ticks s⁻¹)", 
                newTargetSpeed, 
                inchesToTicks.applyAsDouble(newTargetSpeed)
            );
            telemetry.addLine();
        }

        telemetry.addData("Pivot angle", Math.toDegrees(pivot.getCurrentAngle()));
        telemetry.addData("Target speed", "%.1f in s⁻¹", targetSpeed);
        telemetry.addLine();
        telemetry.addData("Measured speed", "%.1f in s⁻¹", shooter.getSpeed());
        telemetry.addData("Actual speed", "%.1f ticks s⁻¹", shootingMotor.getVelocity());

        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
