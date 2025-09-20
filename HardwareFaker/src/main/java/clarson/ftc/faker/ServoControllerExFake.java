package clarson.ftc.faker;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
// import java.util.WeakHashMap;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

public class ServoControllerExFake implements ServoControllerEx {
    protected Map<Integer, ServoData> servos = new HashMap<>(6, 1.0f);
    
    /**
     * Constructs a new ServoControllerExFake with the given servos instantly
     * connected. 
     * 
     * @param servoData The servos and their data to be connected. The actuator
     * field must point to a non-null servo with a valid `getPortNumber()` 
     * return value.
     */
    public ServoControllerExFake(ServoData... servoData) {
        for(final ServoData servoDatum : servoData) {
            this.connect(servoDatum);
        }
    }

    /**
     * Determines whether a servo can be connected to the given port. A port can
     * be connected to if it is between 0-5 (inclusive) and the port is not 
     * already occupied.
     * 
     * @param portNumber The port to check.
     * @return Whether the port exists and is unnoccupied.
     */
    public boolean isPortAvailable(int portNumber) {
        return !servos.containsKey(portNumber) && portNumber >= 0 && portNumber <= 5;
    }

    /**
     * Attempts to connect the servo to this controller. The connection can fail 
     * if the servo's port (from `getPort()`) is already occupied. 
     * 
     * @param servoData Metadata for the servo to be connected. The `getPort()` 
     * method should return a value in the range 0-5 (inclusive), or the 
     * connection will fail. 
     * @return True if the the connection was succesfully completed. 
     */
    public boolean connect(ServoData servoData) {
        int portNumber = -1;
        final HardwareDevice actuator = servoData.actuator;

        if(actuator instanceof CRServoImplEx) {
            portNumber = ((CRServoImplEx) actuator).getPortNumber();
        } else if(actuator instanceof ServoImplEx) {
            portNumber = ((ServoImplEx) actuator).getPortNumber();
        } else {
            throw new ClassCastException("servoData.actuator must be instance of CRServoImplEx or ServoImplEx");
        }

        if(!isPortAvailable(portNumber)) {
            return false;
        }

        servos.put(portNumber, servoData);
        return true;
    }
    
    /**
     * Attempts to connect the servo to this controller. The connection can fail 
     * if the given port number is occupied. 
     * 
     * @param servoData Metadata for the servo to be connected. Does not need
     * to have any specific `actuator` field value, as long as such `actuator`
     * field is set to the desired servo later.
     * @return True if the the connection was succesfully completed. 
     */
    public boolean connect(ServoData servoData, int portNumber) {
        if(!isPortAvailable(portNumber)) {
            return false;
        }

        servos.put(portNumber, servoData);
        return true;
    }

    /**
     * Gets the servo data at the given port number. If the port is not occupied 
     * or does not exists, the method throws an IllegalArgumentException.
     * 
     * @param port The port number at which the servo was connected.
     * @return The servo data at the given port.
     */
    public ServoData getData(int port) {
        if(!servos.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        return servos.get(port);
    }
    
    /**
     * Gets the servo at the given port number. If the port is not occupied or 
     * does not exists, the method throws an IllegalArgumentException. If the 
     * servo at the given port exists but is not a continuous servo, a 
     * ClassCastException will be thrown.
     * 
     * @param port The port number at which the servo was connected.
     * @return The continuous servo at the given port.
     */
    public CRServoImplEx getContinuousServo(int port) {
        if(!servos.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        final ServoData datum = getData(port);
        if(!datum.isPositional() || !(datum instanceof ContinuousServoData)) {
            throw new ClassCastException("Cannot cast non-positional servo to CRServoImplEx");
        }

        return ((ContinuousServoData) datum).actuator;
    }
    
    /**
     * Gets the servo at the given port number. If the port is not occupied or 
     * does not exists, the method throws an IllegalArgumentException. If the 
     * servo at the given port exists but is not a positional servo, a 
     * ClassCastException will be thrown.
     * 
     * @param port The port number at which the servo was connected.
     * @return The positional servo at the given port.
     */
    public ServoImplEx getPositionalServo(int port) {
        if(!servos.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        final ServoData datum = getData(port);
        if(!datum.isPositional() || !(datum instanceof PositionalServoData)) {
            throw new ClassCastException("Cannot cast non-positional servo to ServoImplEx");
        }

        return ((PositionalServoData) datum).actuator;
    }

    @Override
    public void pwmEnable() {
        servos.values().forEach(servo -> {
            servo.isEnabled = true;
        });
    }

    @Override
    public void pwmDisable() {
        servos.values().forEach(servo -> {
            servo.isEnabled = false;
        });
    }

    @Override
    public PwmStatus getPwmStatus() {
        if(servos.size() == 0) {
            return PwmStatus.MIXED;
        }

        // Checking if any of the `isEnabled`s are different from the first.
        final ServoData[] servoData = servos.values().toArray(new ServoData[servos.size()]);
        boolean firstIsEnabled = servoData[0].isEnabled;

        for(int i = 1; i < servoData.length; i++) {
            if(servoData[i].isEnabled != firstIsEnabled) {
                return PwmStatus.MIXED;
            }
        }

        // All the `isEnableds` are the same. Returning whether its enabled or not
        if(firstIsEnabled) {
            return PwmStatus.ENABLED;
        } else {
            return PwmStatus.DISABLED;
        }
    }

    @Override
    public void setServoPosition(int portNumber, double position) {
        if(!getData(portNumber).isEnabled) {
            return;
        }

        if(getData(portNumber).isContinuous()) {
            final ContinuousServoData servo = (ContinuousServoData) getData(portNumber);
            servo.power = Range.clip(2 * pwmPowerFromPosition(servo, position), -1.0, 1.0);
            servo.unaffectedVelocity = servo.power * servo.maxRevsPerSec;
            servo.isTargetSet = true;

            //#region DEV START: Logging the values 
            
            // System.out.println("[set servo position] position: " + position);
            // System.out.println("[set servo position] power: " + servo.power);
            // System.out.println("[set servo position] velocity:" + servo.unaffectedVelocity);
            //#endregion DEV END
        } else if(getData(portNumber).isPositional()) {
            final PositionalServoData servo = (PositionalServoData) getData(portNumber);
            servo.targetPosition = 
                servo.maxPosition 
                * Range.clip(0.5 + pwmPowerFromPosition(servo, position), 0, 1.0);
            servo.isTargetSet = true;
        } else {
            throw new ClassCastException("servoData.actuator must be instance of CRServoImplEx or ServoImplEx");
        }
    }

    @Override
    public double getServoPosition(int portNumber) {
        if(getData(portNumber).isContinuous()) {
            final ContinuousServoData servo = (ContinuousServoData) getData(portNumber);
            return positionFromPwmPower(servo, 0.5 * servo.power);
        } else if(getData(portNumber).isPositional()) {
            final PositionalServoData servo = (PositionalServoData) getData(portNumber);
            return positionFromPwmPower(servo, servo.targetPosition / servo.maxPosition - 0.5);
        } else {
            throw new ClassCastException("servoData.actuator must be instance of CRServoImplEx or ServoImplEx");
        }
    }

    @Override
    public void setServoPwmRange(int portNumber, @NonNull PwmRange range) {
        getData(portNumber).range = range;
    }

    @Override
    @NonNull
    public PwmRange getServoPwmRange(int portNumber) {
        return getData(portNumber).range;
    }

    @Override
    public void setServoPwmEnable(int portNumber) {
        getData(portNumber).isEnabled = true;
    }

    @Override
    public void setServoPwmDisable(int portNumber) {
        getData(portNumber).isEnabled = false;
        getData(portNumber).unaffectedVelocity = 0;
        if(getData(portNumber).isContinuous()) {
            ((ContinuousServoData) getData(portNumber)).power = 0;
        }
    }

    @Override
    public boolean isServoPwmEnabled(int portNumber) {
        return getData(portNumber).isEnabled;
    }

    @Override
    public void setServoType(int portNumber, ServoConfigurationType servoType) {
        if(isPortAvailable(portNumber)) {
            // FIXME: This is the only method that is this graceful, but we need it to be
            //        DcMotorImplEx calls this method before we can connect anything, soooo... 
            return;
        }

        // This isn't usefult for ANYTHING, but someone probably uses it.
        getData(portNumber).servoType = servoType;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "ServoControllerExFake. Hi youtube!";
    }

    private String safeGetDeviceName(HardwareDevice servo) {
        return servo == null ? "null" : servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        
        return String.format(
              "ServoControllerExFake Connections:" +
            "\n    [0]: %s" +  
            "\n    [1]: %s" +  
            "\n    [2]: %s" +  
            "\n    [3]: %s" +  
            "\n    [4]: %s" +  
            "\n    [5]: %s",
            safeGetDeviceName(servos.get(0).actuator),
            safeGetDeviceName(servos.get(1).actuator),
            safeGetDeviceName(servos.get(2).actuator),
            safeGetDeviceName(servos.get(3).actuator),
            safeGetDeviceName(servos.get(4).actuator),
            safeGetDeviceName(servos.get(5).actuator)
        );
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.servos = new HashMap<>();
    }

    @Override
    public void close() {
        this.servos = null; // Allow the connections ot be garbage collected
    }

    /**
     * Converts an abstract position value into a normalized fraction of the max
     * PWM range. This considers the max and current PWM ranges of the given 
     * servo data, so that the return value represents the maximum PWM range,
     * assuming that the position is in terms of the current PWM range. 
     * 
     * @param data Source of the current and max PWM ranges
     * @param position Value on interval [0, 1] where 0 correpsonds to the 
     * `usPulseLower` on the current range and 1 to `usPulseUpper`.
     * @return A value on interval [-0.5, 0.5] where -0.5 corresponds to 
     * `usPulseLower` on the max range and 0.5 to `usPulseUpper`.
     */
    private double pwmPowerFromPosition(ServoData data, double position) {
        final double midpoint = 0.5 * (data.maxPwm.usPulseLower + data.maxPwm.usPulseUpper);
        final double length = data.maxPwm.usPulseUpper - data.maxPwm.usPulseLower;
        final double pwmMicros = Range.scale(
            position, 
            0.0, 
            1.0,
            data.range.usPulseLower, 
            data.range.usPulseUpper
        );

        System.out.println("[pwm power position] midpoint: " + midpoint);
        System.out.println("[pwm power position] length: " + length);
        System.out.println("[pwm power position] pwmMicros: " + pwmMicros);

        return (pwmMicros - midpoint) / length;
    }

    /**
     * Calculates the inverse to `pwmPowerFromPosition()`. The following 
     * examples illustrate what this means:
     * 
     * ```java
     * positionFromPwmPower(data, pwmPowerFromPosition(data, 0.83)) == 0.83 // True
     * pwmPowerFromPosition(data, positionFromPwmPower(data, 0.16)) == 0.16 // True
     * ```
     * 
     * @param data Source of the current and max PWM ranges
     * @param power Value on interval [-0.5, 0.5] where -0.5 correpsonds to the 
     * `usPulseLower` field on the current range and 0.5 to `usPulseUpper`.
     * @return A value on interval [0, 1.0] where 0 corresponds to 
     * `usPulseLower` field on the max range and 1.0 to `usPulseUpper`.
     */
    private double positionFromPwmPower(ServoData data, double power) {
        final double midpoint = 0.5 * (data.maxPwm.usPulseLower + data.maxPwm.usPulseUpper);
        final double length = data.maxPwm.usPulseUpper - data.maxPwm.usPulseLower;
        final double pwmMicros = power * length + midpoint;
        return Range.scale(
            pwmMicros,
            data.range.usPulseLower,
            data.range.usPulseUpper,
            0.0,
            1.0
        );
    }
}
