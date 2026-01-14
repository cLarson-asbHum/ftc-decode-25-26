package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.teamcode.hardware.subsystem.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;

public class Robot {
    @FunctionalInterface
    private static interface DeviceInit {
        boolean run(Robot robot);
    }

    public static enum Device {
        DRIVETRAIN (robot -> robot.initDrivetrain()),
        INTAKE     (robot -> robot.initIntake()),
        RAMP_PIVOT (robot -> robot.initRampPivot()),

        SHOOTER       (robot -> robot.initShooter()),

        /**
         * Should never be used in conjunction with SHOOTER or RIGHT_SHOOTER
         */
        LEFT_SHOOTER  (robot -> robot.initLeftShooter()),
        
        /**
         * Should never be used in conjunction with SHOOTER or LEFT_SHOOTER
         */
        RIGHT_SHOOTER (robot -> robot.initRightShooter()),

        LEFT_BLOCKER  (robot -> robot.initLeftBlocker()),
        RIGHT_BLOCKER (robot -> robot.initRightBlocker()),
        
        LEFT_RELOAD  (robot -> robot.initLeftReload()),
        RIGHT_RELOAD (robot -> robot.initRightReload()),

        MOTIF_WEBCAM          (robot -> robot.initMotifWebcam()),
        ULTIMATE_POINT_EARNER (robot -> robot.initDuckSpinner()),

        LEFT_LED  (robot -> robot.initLeftLed()),
        RIGHT_LED (robot -> robot.initRightLed());

        private final DeviceInit init;
        private Device(DeviceInit init) {
            this.init = init;
        }

        public void init(Robot robot) {
            this.init.run(robot);
        }
    
        public void requestHardware(HardwareMap hardwareMap) {}
    }

    public static final Set<Device> teleopDevices() {
        final Set<Device> result = new LinkedHashSet(Device.values().length);
        for(final Device device : Device.values()) {
            result.add(device);
        }

        result.remove(Device.MOTIF_WEBCAM);
        result.remove(Device.LEFT_SHOOTER);
        result.remove(Device.RIGHT_SHOOTER);
        return result;
    }

    public static final Set<Device> autoDevices() {
        final Set<Device> result = new LinkedHashSet(Device.values().length);
        for(final Device device : Device.values()) {
            result.add(device);
        }

        result.remove(Device.DRIVETRAIN);
        result.remove(Device.LEFT_SHOOTER);
        result.remove(Device.RIGHT_SHOOTER);
        return result;
    }

    private final HardwareMap hardwareMap;
    private final LinkedHashMap<String, Class<?>> nullDevices = new LinkedHashMap<>();

    private BasicMecanumDrive drivetrain = null;
    private CarwashIntake intake = null;
    private LinearHingePivot rampPivot = null;
    private FlywheelTubeShooter shooter = null;
    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;
    private ArtifactColorRangeSensor leftReload = null;
    private ArtifactColorRangeSensor rightReload = null;
    private MotifWebcam motifWebcam = null;
    private CRServo duckSpinner = null;
    private ArtifactColorLed leftLed = null;
    private ArtifactColorLed rightLed = null;

    public Robot(HardwareMap hardwareMap, Set<Device> devicesToInit) {
        this.hardwareMap = hardwareMap;
        for(final Device device : devicesToInit) {
            try {
                device.init(this);
            } catch(DeviceNotFoundException exception) {
                // This is expected, and will be thrown at the end of construction
                // We wait before throwing because we want to aggregate all the names
                // of the devices that we not found, as this is more summarative
            }
        }

        // Alerting of any unfound hardware
        throwAFitIfAnyHardwareIsNotFound();
    }

    /**
     * Attempts to get the given hardware from the hardwareMap. If it cannot be 
     * found, then it returns null without finding an error.
     * 
     * This method should be used instead of hardwareMap.get() because it allows
     * us to see **all** the hardware that we cannot find.
     * 
     * @return The hardware with that name, or null if it cannot be found.
     */
    private <T> T findHardware(Class<T> hardwareType, String name) {
        final T result = hardwareMap.tryGet(hardwareType, name);

        // Adding it to the list if null
        if(result == null) {
            nullDevices.put(name, hardwareType);
        }

        return result;
    }

    /**
     * Throws an exception if any devices are in the nullDeviceNames or 
     * nullDeviceTypes lists. The thrown exception contains the names and types 
     * of all null hardware devices. 
     */
    private void throwAFitIfAnyHardwareIsNotFound() {
        if(nullDevices.size() != 0) {
            String concat = "";

            final Iterator<String> nullDeviceNamesIter = nullDevices.keySet().iterator();
            final Iterator<Class<?>> nullDeviceTypesIter = nullDevices.values().iterator();

            for(int i = 0; i < nullDevices.size(); i++) {
                final String name = nullDeviceNamesIter.next();
                final Class type = nullDeviceTypesIter.next(); 
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

            throw new DeviceNotFoundException("Cannot find hardware:" + concat);
        }
    }

    private boolean initDrivetrain() {
        if(drivetrain != null) {
            return false;
        }

        // Getting all the necessary motors
        final DcMotorEx frontLeftMotor  = (DcMotorEx) findHardware(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeftMotor   = (DcMotorEx) findHardware(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRightMotor = (DcMotorEx) findHardware(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRightMotor  = (DcMotorEx) findHardware(DcMotor.class, "backRight"); // Null if not found

        throwAFitIfAnyHardwareIsNotFound();

        // Creating the drivetrain
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.drivetrain = new BasicMecanumDrive(
            frontLeftMotor, 
            backLeftMotor,
            frontRightMotor,
            backRightMotor
        );
        return true;
    }

    private boolean initIntake() {
        if(intake != null) {
            return false;
        }

        // Getting the motors
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");
        throwAFitIfAnyHardwareIsNotFound();

        // Creating the intake subsystem
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        this.intake = new CarwashIntake(intakeMotor);
        return true;
    }

    private Map<Double, Double> getRampPivotTuning() {
        // These magic numbers were found experimentally 
        // NOTE: This is a default tuning, from 27 Dec 2025 at 1:28 PM
        return new HashMap<>() {{
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
            
            // The following values are just in case we get out of bound values
            put(-0.1, Math.toRadians(37.001)); // Minimum possible angle
            put(1.1, Math.toRadians(73.001));  // Maximum possible angle
        }};
    }

    private boolean initRampPivot() {
        if(rampPivot != null) {
            return false;
        }

        // Getting all the necessary hardware (a servo)
        final ServoImplEx rampPivotServo = (ServoImplEx) findHardware(Servo.class, "rampPivot");
        throwAFitIfAnyHardwareIsNotFound();

        // Creating the subsystem
        final LinearInterpolator positionToRadians = new LinearInterpolator(getRampPivotTuning());
        rampPivotServo.setPwmRange(new PwmRange(1050, 1950));
        this.rampPivot = new LinearHingePivot.Builder(rampPivotServo)
            .setPositionToRadians(positionToRadians)
            .setRadiansToPosition(positionToRadians.inverse())
            .build();
        return true;
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

    private boolean initShooter() {
        if(shooter != null) {
            return false;
        }

        // Getting the necessary hardware: motors, servos, and sensors
        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");
        
        final ArtifactColorRangeSensor rightReload = getRightReload();
        final ArtifactColorRangeSensor leftReload = getLeftReload();

        throwAFitIfAnyHardwareIsNotFound();
        
        // Setting the necessary states
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFeederServo.setDirection(DcMotor.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        // Wrapping these with subsystems.
        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor, rightShooterMotor);
        this.shooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .setTicksToInches(this::ticksToInches)
            .setInchesToTicks(this::inchesToTicks)
            .build();
        return true;
    }

    private boolean initLeftShooter() {
        if(shooter != null) {
            return false;
        }

        // Getting the necessary hardware: motors, servos, and sensors
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");
        final ArtifactColorRangeSensor leftReload = getLeftReload();

        throwAFitIfAnyHardwareIsNotFound();
        
        // Setting the necessary states
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        // Wrapping these with subsystems.
        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor);
        this.shooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(null) // FlywheelTubeShooter is null safe for feeders
            .setLeftReloadClassifier(leftReload)
            .setRightReloadClassifier(() -> ArtifactColor.UNKNOWN)
            .setTicksToInches(this::ticksToInches)
            .setInchesToTicks(this::inchesToTicks)
            .build();
        return true;
    }

    private boolean initRightShooter() {
        if(shooter != null) {
            return false;
        }

        // Getting the necessary hardware: motors, servos, and sensors
        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final ArtifactColorRangeSensor rightReload = getRightReload();

        throwAFitIfAnyHardwareIsNotFound();
        
        // Setting the necessary states
        rightShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFeederServo.setDirection(DcMotor.Direction.FORWARD);

        // Wrapping these with subsystems.
        final DcMotorGroup flywheels = new DcMotorGroup(rightShooterMotor);
        this.shooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(null)  // FlywheelTubeShooter is null safe for feeders
            .setRightFeeder(rightFeederServo) 
            .setLeftReloadClassifier(() -> ArtifactColor.UNKNOWN)
            .setRightReloadClassifier(rightReload)
            .setTicksToInches(this::ticksToInches)
            .setInchesToTicks(this::inchesToTicks)
            .build();
        return true;
    }

    private boolean initLeftBlocker() {
        if(leftBlocker != null) {
            return false;
        }

        // Getting the necessary hardware: a servo
        final ServoImplEx leftBlockerServo = (ServoImplEx) findHardware(Servo.class, "leftBlocker");
        throwAFitIfAnyHardwareIsNotFound();

        // Creating the subsystem
        leftBlockerServo.setPwmRange(new PwmRange(500, 2500));
        this.leftBlocker = new BlockerSubsystem(
            leftBlockerServo, 
            BlockerSubsystem.PositionPresets.LEFT
        );
        return true;
    }

    private boolean initRightBlocker() {
        if(rightBlocker != null) {
            return false;
        }

        // Getting the necessary hardware: a servo
        final ServoImplEx rightBlockerServo = (ServoImplEx) findHardware(Servo.class, "rightBlocker");
        throwAFitIfAnyHardwareIsNotFound();

        // Creating the subsystem
        rightBlockerServo.setPwmRange(new PwmRange(500, 2500));
        this.rightBlocker = new BlockerSubsystem(
            rightBlockerServo, 
            BlockerSubsystem.PositionPresets.RIGHT
        );
        return true;
    }

    private boolean initLeftReload() {
        if(leftReload != null) {
            return false;
        }

        // Getting the necessary hardware: a few sensors
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        final DistanceSensor leftDistanceSensor = findHardware(DistanceSensor.class, "leftDistance");

        throwAFitIfAnyHardwareIsNotFound();;

        // Wrapping this with an abstraction class
        this.leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            leftDistanceSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );
        return true;
    }

    private boolean initRightReload() {
        if(rightReload != null) {
            return false;
        }

        // Getting the necessary hardware: a few sensors
        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final DistanceSensor rightDistanceSensor = findHardware(DistanceSensor.class, "rightDistance");

        throwAFitIfAnyHardwareIsNotFound();;

        // Wrapping this with an abstraction class
        this.rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor,
            rightDistanceSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );
        return true;
    }

    // TODO: Webcam
    private boolean initMotifWebcam() {
        return false;
    }

    private boolean initDuckSpinner() {
        if(duckSpinner != null) {
            return false;
        }

        // Getting the hardware
        final CRServoImplEx duckSpinner = (CRServoImplEx) findHardware(CRServo.class, "duckSpinner");
        throwAFitIfAnyHardwareIsNotFound();
        duckSpinner.setPwmRange(new PwmRange(1000, 2000));
        this.duckSpinner = duckSpinner;
        return true;
    }

    private boolean initLeftLed() {
        if(leftLed != null) {
            return false;
        }

        // Getting the red and green components
        final SwitchableLight red = findHardware(SwitchableLight.class, "leftRed");
        final SwitchableLight green = findHardware(SwitchableLight.class, "leftGreen");

        throwAFitIfAnyHardwareIsNotFound();

        // Abstracting this with a wrapper
        leftLed = new ArtifactColorLed(red, green);
        return true;
    }

    private boolean initRightLed() {
        if(rightLed != null) {
            return false;
        }

        // Getting the red and green components
        final SwitchableLight red = findHardware(SwitchableLight.class, "rightRed");
        final SwitchableLight green = findHardware(SwitchableLight.class, "rightGreen");

        throwAFitIfAnyHardwareIsNotFound();

        // Abstracting this with a wrapper
        rightLed = new ArtifactColorLed(red, green);
        return true;
    }

    public BasicMecanumDrive getDrivetrain() {
        if(drivetrain == null) initDrivetrain();
        return drivetrain;
    }
    
    public CarwashIntake getIntake() {
        if(intake == null) initIntake();
        return intake;
    }
    
    public LinearHingePivot getRampPivot() {
        if(rampPivot == null) initRampPivot();
        return rampPivot;
    }
    
    public FlywheelTubeShooter getShooter() {
        if(shooter == null) initShooter();
        return shooter;
    }
    
    public FlywheelTubeShooter getLeftShooter() {
        if(shooter == null) initLeftShooter();
        return shooter;
    }
    
    public FlywheelTubeShooter getRightShooter() {
        if(shooter == null) initRightShooter();
        return shooter;
    }
    
    public BlockerSubsystem getLeftBlocker() {
        if(leftBlocker == null) initLeftBlocker();
        return leftBlocker;
    }
    
    public BlockerSubsystem getRightBlocker() {
        if(rightBlocker == null) initRightBlocker();
        return rightBlocker;
    }
    
    public ArtifactColorRangeSensor getLeftReload() {
        if(leftReload == null) initLeftReload();
        return leftReload;
    }
    
    public ArtifactColorRangeSensor getRightReload() {
        if(rightReload == null) initRightReload();
        return rightReload;
    }
    
    public MotifWebcam getMotifWebcam() {
        return null;
    }
    
    public CRServo getDuckSpinner() {
        if(duckSpinner == null) initDuckSpinner();
        return duckSpinner;
    }
    
    public ArtifactColorLed getLeftLed() {
        if(leftLed == null) initLeftLed();
        return leftLed;
    }
    
    public ArtifactColorLed getRightLed() {
        if(rightLed == null) initRightLed();
        return rightLed;
    }

    /**
     * Obtains every single non-null subsystem. Each subsystem is guaranteed to 
     * appear only once. If no subsystems are initialized (they're all null), 
     * then this returns an empty array.
     * 
     * @return All initialized (non-null) subsystems on this robot. Can be empty but 
     * never null
     */
    public Subsystem[] getAllSubsystems() {
        final LinkedHashSet<Subsystem> possiblyNull = new LinkedHashSet<>() {{
            add(drivetrain);
            add(intake);
            add(rampPivot);
            add(shooter);
            add(leftBlocker);
            add(rightBlocker);
        }};

        // Removing the null subsystems.
        for(final Subsystem sub : possiblyNull) {
            if(sub == null) {
                possiblyNull.remove(sub);
            }
        }

        // Returning what is left
        return possiblyNull.toArray(new Subsystem[0]);
    }
}