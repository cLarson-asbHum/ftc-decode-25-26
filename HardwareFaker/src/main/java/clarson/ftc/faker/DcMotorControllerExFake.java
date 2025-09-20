package clarson.ftc.faker;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Map;
import java.util.HashMap;

public class DcMotorControllerExFake implements DcMotorControllerEx {
    public static LynxModuleHardwareFake createUniqueModule() throws RobotCoreException, InterruptedException {
        final LynxUsbDeviceImplFake device = new LynxUsbDeviceImplFake();
        final LynxModule module = device.getOrAddModule(
            new LynxModuleDescription.Builder(-1, true)
                .setUserModule()
                .build()
        );
        device.armOrPretend();
        return (LynxModuleHardwareFake) module;
    }

    /**
     * Constructs a new DcMotorControllerExFake whose module is null if an error 
     * is thrown while trying to create a unique module for it.
     * 
     * @return A newly constructed controller.
     */
    public static DcMotorControllerExFake createPossiblyWithNullModule() {
        try {
            return new DcMotorControllerExFake();
        } catch(RobotCoreException | InterruptedException err) {
            return new DcMotorControllerExFake(null);
        }
    }

    private boolean shouldReread = false;
    protected LynxModuleHardwareFake module;
    protected Map<Integer, MotorData> motors = new HashMap<>(4, 1.0f);

    public DcMotorControllerExFake() throws RobotCoreException, InterruptedException {
        this(createUniqueModule());
    }

    public DcMotorControllerExFake(LynxModuleHardwareFake module) {
        this.module = module;
    }

    public void setLynxModule(LynxModuleHardwareFake newModule) {
        this.module = newModule;
    }

    public LynxModuleHardwareFake getLynxModule() {
        return this.module;
    }

    /**
     * Determines whether a motor can be connected to the given port. A port can
     * be connected to if it is between 0-3 (inclusive) and the port is not 
     * already occupied.
     * 
     * @param portNumber The port to check.
     * @return Whether the port exists and is unnoccupied.
     */
    public boolean isPortAvailable(int portNumber) {
        return !motors.containsKey(portNumber) && portNumber >= 0 && portNumber <= 3;
    }

    /**
     * Attempts to connect the motor to this controller. The connection can fail 
     * if the motor's port (from `getPort()`) is already occupied. 
     * 
     * @param motorData Metadata for the motor to be connected. The `getPort()` 
     * method should return a value in the range 0-3 (inclusive), or the 
     * connection will fail. 
     * @return True if the the connection was succesfully completed. 
     */
    public boolean connect(MotorData motorData) {
        if(!isPortAvailable(motorData.actuator.getPortNumber())) {
            return false;
        }

        motors.put(motorData.actuator.getPortNumber(), motorData);
        return true;
    }
    /**
     * Attempts to connect the motor to this controller. The connection can fail 
     * if the given port number is occupied. 
     * 
     * @param motorData Metadata for the motor to be connected. Does not need
     * to have any specific `actuator` field value, as long as such `actuator`
     * field is set to the desired motor later.
     * @return True if the the connection was succesfully completed. 
     */
    public boolean connect(MotorData motorData, int portNumber) {
        if(!isPortAvailable(portNumber)) {
            return false;
        }

        motors.put(portNumber, motorData);
        return true;
    }

    /**
     * Gets the motor data at the given port number. If the port is not occupied 
     * or does not exists, the method throws an IllegalArgumentException.
     * 
     * @param port The port number at which the motor was connected.
     * @return The motor data at the given port.
     */
    public MotorData getData(int port) {
        if(!motors.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        return motors.get(port);
    }

    /**
     * Gets the motor at the given port number. If the port is not occupied or 
     * does not exists, the method throws an IllegalArgumentException.
     * 
     * @param port The port number at which the motor was connected.
     * @return The motor at the given port
     */
    public DcMotorImplEx getMotor(int port) {
        if(!motors.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        return motors.get(port).actuator;
    }

    /**
     * Dictates whether bulk cache-accesssing methods are forced instead to 
     * re-read their data. This can be used to get the data of a method like
     * `getCurrentPosition()` for a bulk cache without causing infinite 
     * recursion (because `getCurrentPosition()` sends a command for a new 
     * cache, which requires calling `getCurrentPosition()` to get its data, and 
     * so on).
     * 
     * This **always** takes precedence over the bulk caching mode. Setting 
     * bulk caching to AUTO does (nearly) nothing if this has been set to true 
     * 
     * @param shouldReread True if bulk caching should be ignored. False for 
     * normal operation.
     * @return Whether any change was made. False if was already set to the 
     * given `shouldReread` value
     */
    public boolean setForceReread(boolean shouldReread) {
        final boolean oldValue = this.shouldReread;
        this.shouldReread = shouldReread;
        return oldValue != shouldReread;
    }

    /**
     * Constructs a `LynxGetMotorEncoderCommand` with this module and the given motor
     * 
     * @return A new, untransmitted `LynxGetMotorEncoderPositionCommand`.
     */
    public LynxGetMotorEncoderPositionCommand getEncoderCommand(DcMotorImplExFake motor) {
        return new LynxGetMotorEncoderPositionCommand(this.module, motor.getPortNumber());
    }

    @Override
    public void setMotorEnable(int port) {
        getData(port).isEnabled = true;
    }

    @Override
    public void setMotorDisable(int port) {
        getData(port).isEnabled = false;
    }

    @Override
    public boolean isMotorEnabled(int port) {
        return getData(port).isEnabled;
    }

    @Override
    public void setMotorVelocity(int port, double ticksPerSecond) {
        if(!getData(port).isEnabled) {
            // Cannot send commands to a motor not able to receive them.
            return;
        }

        if(getData(port).mode != DcMotor.RunMode.RUN_USING_ENCODER) {
            // Cannot set velocity to a motor that doesn't not what velocity
            return;
        }

        getData(port).unaffectedVelocity = ticksPerSecond;
    }

    @Override
    public void setMotorVelocity(int port, double velocity, AngleUnit unit) {
        setMotorVelocity(
            port,
            getData(port).ticksPerRev 
                * AngleUnit.DEGREES.fromUnit(unit, velocity) 
                / 360
        );
    }

    @Override
    public double getMotorVelocity(int port) {
        if(!shouldReread && module.getBulkCachingMode() != LynxModule.BulkCachingMode.OFF) {
            final LynxModule.BulkData data = module.recordBulkCachingCommandIntent(
                new LynxGetBulkInputDataCommand(module),
                "motorVelocity" + port
            );
            return data.getMotorVelocity(port);
        }

        // TODO: I don't think this check is necessary nor accurate
        // if(!getData(port).isEnabled) {
        //     // When not enabled, the motor does not give or take any data.
        //     return 0;
        // }

        return getData(port).getActualVelocity();
    }

    @Override
    public double getMotorVelocity(int port, AngleUnit unit) {
        return unit.fromDegrees(getMotorVelocity(port) / getData(port).ticksPerRev * 360.0);
    }

    @Override
    public void setPIDCoefficients(int port, DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        if(!getData(port).pidCoefficients.containsKey(mode)) {
            return;
        }

        getData(port).pidCoefficients.put(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(int port, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        if(!getData(port).pidfCoefficients.containsKey(mode)) {
            return;
        }

        getData(port).pidfCoefficients.put(mode, pidfCoefficients);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(int port, DcMotor.RunMode mode) {
        return getData(port).pidCoefficients.get(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(int port, DcMotor.RunMode mode) {
        return getData(port).pidfCoefficients.get(mode);
    }

    @Override
    public void setMotorTargetPosition(int port, int position, int tolerance) {
        getData(port).targetPosition = position;
        getData(port).tolerance = tolerance;
        getData(port).isTargetSet = true;
    }

    @Override
    public double getMotorCurrent(int port, CurrentUnit unit) {
        // TODO: Make this calculate current drawn based off of velocity and added angular?
        // Just pretending that resistance doesn't matter (it doesn't, so touch 
        // exposed wires, kids!) and so current = voltage
        return unit.convert(Math.abs(13 * getData(port).power), CurrentUnit.AMPS);
    }

    @Override
    public double getMotorCurrentAlert(int port, CurrentUnit unit) {
        return unit.convert(getData(port).currentAlertAmps, CurrentUnit.AMPS);
    }

    @Override
    public void setMotorCurrentAlert(int port, double current, CurrentUnit unit) {
        getData(port).currentAlertAmps = unit.toAmps(current);
    }

    @Override
    public boolean isMotorOverCurrent(int port) {
        if(!shouldReread && module.getBulkCachingMode() != LynxModule.BulkCachingMode.OFF) {
            // Bulk Cache is enabled; get the data from the bulk cache!
            // Record...Intent() also creates new BulkData if the cache is 
            // clear or the same data was requested twice in BulkCachingMode.AUTO
            final LynxModule.BulkData data = module.recordBulkCachingCommandIntent(
                new LynxGetBulkInputDataCommand(this.module),
                "motorOverCurrent" + port
            );
            return data.isMotorOverCurrent(port);
        }

        return getMotorCurrent(port, CurrentUnit.AMPS) > getData(port).currentAlertAmps;
    }

    @Override
    public void setMotorType(int port, MotorConfigurationType motorType) {
        if(isPortAvailable(port)) {
            // FIXME: This is the only method that is this graceful, but we need it to be
            //        DcMotorImplEx calls this method before we can connect anything, soooo... 
            return;
        }

        // This is only useful to DcMotorImplEx, which uses this to determine operational 
        // direction, that is, whether the motor naturally rotates clockwise or counter.
        getData(port).motorType = motorType;
    }
    
    @Override
    public MotorConfigurationType getMotorType(int port) {
        return getData(port).motorType;
    }
    
    @Override
    public void setMotorMode(int port, DcMotor.RunMode mode) {
        if(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            getData(port).power = 0;
            getData(port).unaffectedVelocity = 0;
            getData(port).position = 0;
        }

        if(mode == DcMotor.RunMode.RUN_TO_POSITION && !getData(port).isTargetSet) {
            // If no target has been set, throw an exception
            throw new RuntimeException("No target set before setting runMode to RUN_TO_POSITION");
        }
        
        if(
            getData(port).mode != DcMotor.RunMode.RUN_TO_POSITION 
            || mode != DcMotor.RunMode.RUN_TO_POSITION
        ) {
            // Cleanup for when exiting RUN_TO_POSITION. Going to and from RUN_TO_POSITIOM doesn't count.
            // NOTE: This also executes when going to RUN_TO_POSITION from a non-RUN_TO mode. 
            // getData(port).runToFactor = 1;
            getData(port).isBusy = false;
            getData(port).isTargetSet = false;
        }

        if(mode == DcMotor.RunMode.RUN_TO_POSITION) {
            getData(port).isBusy = true;
            // getData(port).unaffectedVelocity = 0;
        }

        getData(port).mode = mode;
    }
    
    @Override
    public DcMotor.RunMode getMotorMode(int port) {
        return getData(port).mode;
    }
    
    @Override
    public void setMotorPower(int port, double power) {
        if(!getData(port).isEnabled) {
            // Cannot send commands to a motor that isn't listening to them.
            return;
        }

        getData(port).power = power;
        getData(port).unaffectedVelocity = power * getData(port).maxTicksPerSec;
    }
    
    @Override
    public double getMotorPower(int port) {
        return getData(port).power;
    }
    
    @Override
    public boolean isBusy(int port) {
        // Not sure why *this* is one of the methods that is bulk cached... but sure.
        if(!shouldReread && module.getBulkCachingMode() != LynxModule.BulkCachingMode.OFF) {
            final LynxModule.BulkData data = module.recordBulkCachingCommandIntent(
                new LynxIsMotorAtTargetCommand(module, port),
                ""
            );
            return data.isMotorBusy(port);
        }

        return getData(port).isBusy;
    }
    
    @Override
    public void setMotorZeroPowerBehavior(int port, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        getData(port).behavior = zeroPowerBehavior;
    }
    
    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int port) {
        return getData(port).behavior;
    }
    
    @Override
    public boolean getMotorPowerFloat(int port) {
        return getData(port).behavior == DcMotor.ZeroPowerBehavior.FLOAT 
            && getData(port).power == 0;
    }
    
    @Override
    public void setMotorTargetPosition(int port, int position) {
        getData(port).targetPosition = position;
    }
    
    @Override
    public int getMotorTargetPosition(int port) {
        return getData(port).targetPosition;
    }
    
    @Override
    public int getMotorCurrentPosition(int port) {
        if(!shouldReread && module.getBulkCachingMode() != LynxModule.BulkCachingMode.OFF) {
            final LynxModule.BulkData data = module.recordBulkCachingCommandIntent(
                new LynxGetMotorEncoderPositionCommand(module, port),
                "" // Intentionally empty, as the command above is not a LynxGetBulkInputDatatCommand
            );
            return data.getMotorCurrentPosition(port);
        }

        return (int) Math.round(getData(port).position);
    }
    
    @Override
    public void resetDeviceConfigurationForOpMode(int port) {
        getData(port).copyAvailableProperties(new MotorData(null, 0, 0));    
    }

    @Override
    public void close() {
        this.motors = null; // Allow the connections ot be garbage collected
    }

    private String safeGetDeviceName(DcMotorImplEx possiblyNull) {
        return possiblyNull == null ? "null" : possiblyNull.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return String.format(
              "DcMotorControllerExFake Connections:" +
            "\n    [1]: %s" +  
            "\n    [2]: %s" +  
            "\n    [3]: %s" +  
            "\n    [4]: %s",
            safeGetDeviceName(getMotor(0)),
            safeGetDeviceName(getMotor(1)),
            safeGetDeviceName(getMotor(2)),
            safeGetDeviceName(getMotor(3))
        );
    }

    @Override
    public String getDeviceName() {
        return "DcMotorControllerExFake. Hi youtube!";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.motors = new HashMap<>();
    }
}