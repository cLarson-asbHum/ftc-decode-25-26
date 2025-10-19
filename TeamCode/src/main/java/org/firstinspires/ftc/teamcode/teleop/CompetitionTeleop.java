package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.bylazar.configurables.annotations.Configurable;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.ArrayList;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.util.Util;

@Configurable
@TeleOp(group="A") // Used for the main opmodes
public class CompetitionTeleop extends CommandOpMode {
    public static double TRIGGER_PRESSED = 0.1f;

    private boolean wasInjected = false;
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

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
                concat += "    ";

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

            throw new RuntimeException("Cannot find hardware: \n" + concat);
        }
    }

    private Subsystem[] createSubsystems(HardwareMap hardwareMap) {
        // Find and create all of the hardware. This uses the hardware map. 
        // When using unit tests, the `hardwareMap` field can be set for dependency injection.
        final DcMotorEx frontLeftMotor  = (DcMotorEx) findHardware(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeftMotor   = (DcMotorEx) findHardware(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRightMotor = (DcMotorEx) findHardware(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRightMotor  = (DcMotorEx) findHardware(DcMotor.class, "backRight"); // Null if not found

        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter(rightShooterMotor);
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final BasicMecanumDrive drivetrain = new BasicMecanumDrive(
            frontLeftMotor, 
            backLeftMotor,
            frontRightMotor,
            backRightMotor
        );

        // This means that no command will use the same subsystem at the same time.
        register(rightShooter, intake, drivetrain);

        // Return a list of every subsystem that we have created
        return new Subsystem[] { rightShooter, intake, drivetrain };
    }
    
    @Override
    public void initialize() {
        final Subsystem[] subsystems = createSubsystems(hardwareMap);
        final FlywheelTubeShooter shooter = (FlywheelTubeShooter) subsystems[0];
        final CarwashIntake intake = (CarwashIntake) subsystems[1];
        final BasicMecanumDrive drivetrain = (BasicMecanumDrive) subsystems[2];

        // Creating GamepadEx's
        // These are special versions of gamepads that have added functionality
        final Gamepad driverPad = gamepad1; // Just renaming the old gamepad
        final Gamepad shooterPad = gamepad2; // Just renaming the old gamepad
        final GamepadEx driverPadEx = new GamepadEx(driverPad);
        final GamepadEx shooterPadEx = new GamepadEx(shooterPad);

        // Creating commands and making them happen on 
        // Commands are actions that can be done on button presses
        createShooterCommands(shooterPad, shooterPadEx, shooter, intake);
        createIntakeCommands(shooterPad, shooterPadEx, shooter, intake);
        createDriverCommands(driverPad, driverPadEx, drivetrain);

        // Having the intake hold pieces by default.
        schedule(new InstantCommand(() -> intake.holdGamePieces()));
    }

    private void createShooterCommands(
        Gamepad shooterPad, 
        GamepadEx shooterPadEx, 
        FlywheelTubeShooter shooter,
        CarwashIntake intake
    ) {
        // UpgradeShootingState transitions from UNCAHRGED to CHARGED TO FIRING 
        final Command upgradeShootingState = new SelectCommand(
            new HashMap<Object, Command>() {{
                put(ShooterSubsystem.Status.UNCHARGED, shooter.chargeCommand());
                put(ShooterSubsystem.Status.RELOADED_CHARGED, shooter.fireCommand());
                put(ShooterSubsystem.Status.EMPTY_CHARGED, new SequentialCommandGroup(
                    shooter.reloadCommand(),
                    shooter.fireCommand()
                ));
            }},
            shooter::getStatus
        );

        // Both the triggers do the same thing: charge is uncharged; otherwise, reload if necessary and fire
        new Trigger(() -> shooterPad.left_trigger > TRIGGER_PRESSED)
            .whenActive(upgradeShootingState);
            
        new Trigger(() -> shooterPad.right_trigger > TRIGGER_PRESSED)
            .whenActive(upgradeShootingState);

        // Holding up on the dpad shoots as many projectiles as possible
        final Command chargeIfNecessaryMultiFire = new ConditionalCommand(
            // new InstantCommand(() -> shooter.multiFire()), // Begin the mutli fire if charged
            null,
            shooter.chargeCommand(), // Charge the shooter if uncharged
            () -> shooter.shouldBeAbleToFire() // Detected whether we are charged and ready
        );

        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(chargeIfNecessaryMultiFire) // Start the mutli-fire
            .whenReleased(shooter.chargeCommand()); // End multi fire and go back to charged/charging

        // Allowing for uncharging (aka [Down]grading) upon the down button
        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(shooter.unchargeCommand());

        // [A]bort a fire with the A button
        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(shooter.abortCommand());

        // TODO: Variable fire power
    }

    private void createIntakeCommands(
        Gamepad shooterPad, 
        GamepadEx shooterPadEx, 
        FlywheelTubeShooter shooter,
        CarwashIntake intake
    ) {
        // Bumpers enable (right) or disable (left) the intake
        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(new InstantCommand(() -> intake.intakeGamePieces()));

        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(new InstantCommand(() -> intake.holdGamePieces()));

        // E[x]pell any mistaken pieces with the X button
        shooterPadEx
            .getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(() -> intake.ejectGamePieces()));

        // Transfer any held game pieces with the 
        // TODO: Transfer held pieces to the shooter mover
    }

    private void createDriverCommands(
        Gamepad driverPad,
        GamepadEx drivePadEx,
        BasicMecanumDrive drivetrain
    ) {
        // Settings the default command to be driving with the joysticks
        // A default command is something that a subsystem does when no other 
        // command is being done.
        drivetrain.setDefaultCommand(new RunCommand(
            // We do mecanumDrive() by default, which is what drives the chasis
            () -> drivetrain.mecanumDrive(
                -driverPad.left_stick_y, 
                driverPad.left_stick_x, 
                driverPad.right_stick_x
            ),
            // We mark drivetrain as the subsystem so no one else uses it
            drivetrain
        ));

        // Hold either trigger for fast mode.
        // Fast mode ends when we let go.
        new Trigger(() -> driverPad.left_trigger > TRIGGER_PRESSED || driverPad.right_trigger > TRIGGER_PRESSED)
            .whenActive(new InstantCommand(() -> drivetrain.engageFastMode()))
            .whenInactive(new InstantCommand(() -> drivetrain.engageMiddleMode()));

            
        // Hold either trigger for slow mode
        // Slow mode ends when we let go.
        new Trigger(() -> driverPad.left_bumper || driverPad.right_bumper)
            .whenActive(new InstantCommand(() -> drivetrain.engageFastMode()))
            .whenInactive(new InstantCommand(() -> drivetrain.engageMiddleMode()));
    }
}