package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.HashMap;
import java.util.function.Consumer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Wraps an arbitary number of motors and uses the given method on all motors.
 * This is most useful for designs with opposite motors, such as in flywheel 
 * set-ups where both will have the same power, but need to spin in opposite 
 * directions.
 */
public class DcMotorGroup implements DcMotorEx {
    private final DcMotorEx[] motors;

    public DcMotorGroup(DcMotorEx... motors) {
        this.motors = motors;
    }

    private void all(Consumer<DcMotorEx> doable) {
        for(final DcMotorEx motor : motors) {
            doable.accept(motor);
        }
    }

    @Override
    public void setMotorEnable() {
        all((motor) -> motor.setMotorEnable());
    }

    @Override
    public void setMotorDisable() {
        all((motor) -> motor.setMotorDisable());
    }

    /**
     * Determines whether *any* motor in this group is enabled. This only returns
     * false if every motor given at construction is disabled.
     * 
     * @return True if *any* motor is enabled; false if *all* motors are disabled.
     */
    @Override
    public boolean isMotorEnabled() {
        for(final DcMotorEx motor : motors) {
            if(motor.isMotorEnabled()) {
                return true;
            }
        }

        return false;
    }
    
    @Override
    public void setVelocity(double angularRate)  {
        all((motor) -> motor.setVelocity(angularRate));
    }
    
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        all((motor) -> motor.setVelocity(angularRate, unit));
    }
    
    /**
     * Returns the *average* velocity of the motors in ticks per second. 
     */
    @Override
    public double getVelocity() {
        double total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getVelocity();
        }
        return total / motors.length;
    }
    
    /**
     * Returns the *average* velocity of the motors in the given unit
     */
    @Override
    public double getVelocity(AngleUnit unit) {
        double total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getVelocity(unit);
        }
        return total / motors.length;
    }
    
    @Override
    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        all((motor) -> motor.setPIDCoefficients(mode, pidCoefficients));
    }
    
    @Override
    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        all((motor) -> motor.setPIDFCoefficients(mode, pidfCoefficients));
    }
    
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        all((motor) -> motor.setVelocityPIDFCoefficients(p, i, d, f));
    }
    
    @Override
    public void setPositionPIDFCoefficients(double p) {
        all((motor) -> motor.setPositionPIDFCoefficients(p));
    }
    
    /**
     * Gets the *average* PID coeffients. Averaging is done componentwise,
     * so P is the average of all the motors' Ps, I is the avergage of all the
     * motors' Is, etc.
     */
    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode mode) {
        double totalP = 0;
        double totalI = 0;
        double totalD = 0;

        for(final DcMotorEx motor : motors) {
            final PIDCoefficients coef = motor.getPIDCoefficients(mode);
            totalP += coef.p;
            totalI += coef.i;
            totalD += coef.d;
        }

        return new PIDCoefficients(totalP / motors.length, totalI / motors.length, totalD / motors.length);
    }
    
    @Override
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        double totalP = 0;
        double totalI = 0;
        double totalD = 0;
        double totalF = 0;

        for(final DcMotorEx motor : motors) {
            final PIDFCoefficients coef = motor.getPIDFCoefficients(mode);
            totalP += coef.p;
            totalI += coef.i;
            totalD += coef.d;
            totalF += coef.f;
        }

        return new PIDFCoefficients(
            totalP / motors.length, 
            totalI / motors.length, 
            totalD / motors.length,
            totalF / motors.length
        );
    }
    
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        all((motor) -> motor.setTargetPositionTolerance(tolerance));
    }
    
    /**
     * Something something average
     * @return
     */
    @Override
    public int getTargetPositionTolerance()  {
        int total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getTargetPositionTolerance();
        }
        return total / motors.length;
    }
    
    @Override
    public double getCurrent(CurrentUnit unit) {
        
        double total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getCurrent(unit);
        }
        return total / motors.length;
    }
    
    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        
        double total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getCurrentAlert(unit);
        }
        return total / motors.length;
    }
    
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        all((motor) -> motor.setCurrentAlert(current, unit));
    }
    
    /**
     * Determines if **any** motor in this group is over current.
     */
    @Override
    public boolean isOverCurrent() {
        for(final DcMotorEx motor : motors) {
            if(motor.isOverCurrent()) {
                return true;
            }
        }

        return false;
    }

    /**
     * Returns the most common direction.
     */
    @Override
    public DcMotor.Direction getDirection() {
        final HashMap<DcMotor.Direction, Integer> counts = new HashMap<DcMotor.Direction, Integer>(DcMotor.Direction.values().length);

        // Counting
        for(final DcMotorEx motor : motors) {
            final DcMotor.Direction direction = motor.getDirection();
            if(!counts.containsKey(direction)) {
                counts.put(direction, 0);
            } else {
                counts.put(direction, counts.get(direction) + 1);
            }
        }

        // Getting the most common
        DcMotor.Direction maxDirection = null;
        int max = Integer.MIN_VALUE;

        for(final DcMotor.Direction direction : counts.keySet()) {
            if(counts.get(direction) > max) {
                maxDirection = direction;
                max = counts.get(direction);
            }
        }

        return maxDirection;
    } 

    @Override
    public double getPower() {
        double total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getPower();
        }
        return total / motors.length;
    } 
    
    @Override
    public void setDirection(DcMotor.Direction direction) {
        all((motor) -> motor.setDirection(direction));
    } 
    
    @Override
    public void setPower(double power) {
        all((motor) -> motor.setPower(power));
    }

    /**
     * Gets the most common mode.
     */
    @Override
    public DcMotor.RunMode getMode() {
        final HashMap<DcMotor.RunMode, Integer> counts = new HashMap<DcMotor.RunMode, Integer>(DcMotor.RunMode.values().length);

        // Counting
        for(final DcMotorEx motor : motors) {
            final DcMotor.RunMode runmode = motor.getMode();
            if(!counts.containsKey(runmode)) {
                counts.put(runmode, 0);
            } else {
                counts.put(runmode, counts.get(runmode) + 1);
            }
        }

        // Getting the most common
        DcMotor.RunMode maxRunMode = null;
        int max = Integer.MIN_VALUE;

        for(final DcMotor.RunMode runmode : counts.keySet()) {
            if(counts.get(runmode) > max) {
                maxRunMode = runmode;
                max = counts.get(runmode);
            }
        }

        return maxRunMode;
    }

    @Override
    public void setMode(DcMotor.RunMode mode) {
        all((motor) -> motor.setMode(mode));
    }
    
    @Override
    public DcMotorController getController() {
        // FIXME: This returns null. Although I assume this isn't used by user code, you never know!
        return null;
    }
    
    /**
     * Calculates the average position of hte motors in the group.
     */
    @Override
    public int getCurrentPosition() {
        int total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getCurrentPosition();
        }
        return total / motors.length;
    }
    
    @Override
    public MotorConfigurationType getMotorType() {
        // FIXME: I don't assume this is used, but...
        return null;
    }
    
    @Override
    public int getPortNumber() {
        return -1;
    }
    
    /**
     * Determines whether **any** motor is power floating.
     */
    @Override
    public boolean getPowerFloat() {
        for(final DcMotor motor : motors) {
            if(motor.getPowerFloat()) {
                return true;
            }
        }

        return false;
    }
    
    @Override
    public int getTargetPosition() {
        int total = 0;
        for(final DcMotorEx motor : motors) {
            total += motor.getTargetPosition();
        }
        return total / motors.length;
    }
    
    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        
        final HashMap<DcMotor.ZeroPowerBehavior, Integer> counts = new HashMap<DcMotor.ZeroPowerBehavior, Integer>(DcMotor.ZeroPowerBehavior.values().length);

        // Counting
        for(final DcMotorEx motor : motors) {
            final DcMotor.ZeroPowerBehavior zeroPowerBehavior = motor.getZeroPowerBehavior();
            if(!counts.containsKey(zeroPowerBehavior)) {
                counts.put(zeroPowerBehavior, 0);
            } else {
                counts.put(zeroPowerBehavior, counts.get(zeroPowerBehavior) + 1);
            }
        }

        // Getting the most common
        DcMotor.ZeroPowerBehavior maxZeroPowerBehavior = null;
        int max = Integer.MIN_VALUE;

        for(final DcMotor.ZeroPowerBehavior zeroPowerbehavior : counts.keySet()) {
            if(counts.get(zeroPowerbehavior) > max) {
                maxZeroPowerBehavior = zeroPowerbehavior;
                max = counts.get(zeroPowerbehavior);
            }
        }

        return maxZeroPowerBehavior;
    }
    
    /**
     * Determines whether **any** motor in the group is busy.
     */
    @Override
    public boolean isBusy() {
        for(final DcMotor motor : motors) {
            if(motor.isBusy()) {
                return true;
            }
        }

        return false;
    }
    
    @Override
    public void setMotorType(MotorConfigurationType config) {
        all((motor) -> motor.setMotorType(config));
    }
    
    @Override
    public void setPowerFloat() {
        all((motor) -> motor.setPowerFloat());
    }
    
    @Override
    public void setTargetPosition(int target) {
        all((motor) -> motor.setTargetPosition(target));
    }
    
    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        all((motor) -> motor.setZeroPowerBehavior(behavior));
    }

    @Override
    public void close() {
        all((motor) -> motor.close());
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        all((motor) -> motor.resetDeviceConfigurationForOpMode());
    }

    @Override
    public int getVersion() {
        return -1;
    }

    @Override
    public String getConnectionInfo() {
        String result = "";

        for(final DcMotorEx motor : motors) {
            result += motor.getConnectionInfo() + "\n";
        }

        return result;
    }

    @Override
    public String getDeviceName() {
        return "DcMotorGroup";
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
}