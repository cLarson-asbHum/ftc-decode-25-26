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
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.teamcode.hardware.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Device;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.PivotSubsystem;

@TeleOp(group="B - Testing")
public class ShooterSpeedTest extends OpMode {

    private FlywheelTubeShooter shooter = null;
    private DoubleUnaryOperator inchesToTicks = null;
    private PivotSubsystem pivot = null;

    private boolean areControlsHidden = true;

    private double targetSpeed = 0;
    private double newTargetSpeed = 0;
    private DcMotorEx shootingMotor = null;

    @Override
    public void init() {
        final Robot robot = new Robot(hardwareMap, Set.of(Device.LEFT_SHOOTER, Device.RAMP_PIVOT));
        shooter = robot.getShooter();
        pivot = robot.getRampPivot();
        shootingMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftShooter");
        
        telemetry.setMsTransmissionInterval(33);
        
        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(shooter, pivot);
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
