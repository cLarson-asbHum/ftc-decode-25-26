package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="B - Testing")
public final class PidfTest extends OpMode {
    private ElapsedTime timer = new ElapsedTime();

    private DcMotorImplEx rightMotor = null;
    private DcMotorImplEx leftMotor = null;
    private DcMotorImplEx shootingMotor = rightMotor;

    private boolean areControlsHidden = true;
    
    private Target mode = Target.SPEED;
    private Cursor cursor = Cursor.P;

    private PIDFCoefficients target = new PIDFCoefficients(0, 0, 0, 0);
    private PIDFCoefficients newTarget = new PIDFCoefficients(0, 0, 0, 0);
    private double coeffIncrement = 1;
    
    private double targetSpeed = 0;
    private double newTargetSpeed = 0;
    private final double speedIncrement = 100;

    private boolean wasPressingRightTrigger = false;
    private boolean wasPressingLeftTrigger = false;

    @Override
    public void init() {
        rightMotor = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "rightShooter");
        leftMotor = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "leftShooter");
        shootingMotor = leftMotor;
        newTarget = target = shootingMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
    }
    
    @Override
    public void start() {
        timer.reset();
    }

    private static enum Target {
        SPEED,
        PIDF
    }
    
    @Override
    public void loop() {
        // Handling button presses
        if(gamepad1.backWasPressed()) {
            areControlsHidden = !areControlsHidden;
        }

        if(mode == Target.SPEED) {
            adjustSpeedTarget(gamepad1);
        }

        if(mode == Target.PIDF) {
            adjustPidfTarget(gamepad1);
        }
        
        if(gamepad1.bWasPressed()) {
            shootingMotor.setVelocity(0);
        }

        if(gamepad1.aWasPressed()) {
            switchMotor();
        }

        // Controls
        if(!areControlsHidden) {
            telemetry.addLine(Util.header("Controls (Back to hide)"));
            telemetry.addData("Increase speed/coeff", "RB");
            telemetry.addData("Decrement speed/coeff", "LB");
            telemetry.addLine();
            telemetry.addData("Switch target", "A");
            telemetry.addData("Update speed/coeff", "Y");
            telemetry.addLine();
            telemetry.addData("Fast mode", "RT");
            telemetry.addData("Slow mode", "LT");
            telemetry.addLine();
            telemetry.addData("Stop", "B");
            telemetry.addData("Switch motor", "A");
            telemetry.addLine();
            telemetry.addData("Move cursor up", "↑");
            telemetry.addData("Move cursor down", "↓");
        } else {
            telemetry.addLine(Util.header("Controls (Back to show)"));
        }

        wasPressingRightTrigger = gamepad1.right_trigger > 0.1;
        wasPressingLeftTrigger = gamepad1.left_trigger > 0.1;

        telemetry.addLine();
        telemetry.addLine(Util.header("Motor Speed"));
        telemetry.addLine();

        if(!equalPidfs(newTarget, target) && mode == Target.PIDF) {
            displayNewPidfTarget();
        }

        if(newTargetSpeed != targetSpeed && mode == Target.SPEED) {
            telemetry.addData("New target", "%.0f ticks s⁻¹", newTargetSpeed);
            telemetry.addLine();
        }

        telemetry.addData("Time", "%.2f s", timer.seconds());
        telemetry.addData("Target speed", "%.0f ticks s⁻¹", targetSpeed);
        telemetry.addData("Current speed", "%.0f ticks s⁻¹", shootingMotor.getVelocity());
        telemetry.addLine();
        telemetry.addLine("Coefficients");
        telemetry.addData( "    | p", target.p);
        telemetry.addData( "    | i", target.i);
        telemetry.addData( "    | d", target.d);
        telemetry.addData("    \\ f", target.f);


        CommandScheduler.getInstance().run();
    }

    private static enum Cursor {
        P,
        I,
        D,
        F
    }

    private void displayNewPidfTarget() {
        telemetry.addLine(Util.header("PIDF"));
        telemetry.addLine();
        telemetry.addLine("New Coefficients");

        final String pCursor = cursor == Cursor.P ? ">" : " ";
        final String iCursor = cursor == Cursor.I ? ">" : " ";
        final String dCursor = cursor == Cursor.D ? ">" : " ";
        final String fCursor = cursor == Cursor.F ? ">" : " ";

        telemetry.addData(String.format(" %s   | p",  pCursor), newTarget.p);
        telemetry.addData(String.format(" %s   | i",  iCursor), newTarget.i);
        telemetry.addData(String.format(" %s   | d",  dCursor), newTarget.d);
        telemetry.addData(String.format(" %s   \\ f", fCursor), newTarget.f);
        telemetry.addLine();
    }

    private void adjustPidfTarget(Gamepad gamepad) {
        if(gamepad.y) {
            target.p = newTarget.p;
            target.i = newTarget.i;
            target.d = newTarget.d;
            target.f = newTarget.f;
            shootingMotor.setVelocityPIDFCoefficients(target.p, target.i, target.d, target.f);
        }

        // Changing the increment
        if(gamepad.right_trigger > 0.1 && !wasPressingRightTrigger) {
            coeffIncrement *= 10;
        }
        
        if(gamepad.left_trigger > 0.1 && !wasPressingLeftTrigger) {
            coeffIncrement *= 0.1;
        }

        // Updating the value
        double delta = 0.0;
        if(gamepad.rightBumperWasPressed()) {
            delta += coeffIncrement;
        }
        
        if(gamepad.leftBumperWasPressed()) {
            delta -= coeffIncrement;
        }

        switch(cursor) {
            case P:
                newTarget.p += delta;
                break;

            case I:
                newTarget.i += delta;
                break;

            case D:
                newTarget.d += delta;
                break;

            case F:
                newTarget.f += delta;
                break;
        }
    }

    private void adjustSpeedTarget(Gamepad gamepad) {
        if(gamepad.y) {
            targetSpeed = newTargetSpeed;
            shootingMotor.setVelocity(targetSpeed);
        }

        // Changing the increment
        double increment = 1;
        if(gamepad.right_trigger > 0.1) {
            increment *= 2.5;
        }
        
        if(gamepad.left_trigger > 0.1) {
            increment *= 0.5;
        }

        // Updating the value
        double delta = 0.0;
        if(gamepad.rightBumperWasPressed()) {
            delta += increment;
        }
        
        if(gamepad.leftBumperWasPressed()) {
            delta -= increment;
        }

        newTargetSpeed += delta;
    
    }

    private void switchMotor() {
        // Disabling the old motor
        shootingMotor.setPower(0);
        shootingMotor.setMotorDisable();

        // Swith=ching to the new motor
        if(shootingMotor == leftMotor) {
            shootingMotor = rightMotor;
        } else {
            shootingMotor = leftMotor;
        }

        target = shootingMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }


    private boolean equalPidfs(PIDFCoefficients p1, PIDFCoefficients p2) {
        return p1.p == p2.p
            && p1.i == p2.i
            && p1.d == p2.d
            && p1.f == p2.f
            && p1.algorithm == p2.algorithm;
    }
}