package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.configurables.annotations.Configurable;

import static org.firstinspires.ftc.teamcode.util.Util.*;

public class BasicMecanumDrive implements MecanumSubsystem {
    public static final class POWER {
        public double maxForward = 1.0;
        public double maxLateral = 1.0;
        public double maxYaw = 0.75;

        public double middleForward = 0.75 * maxForward;
        public double middleLateral = 0.75 * maxLateral;
        public double middleYaw = 0.75 * maxYaw;
        
        public double minForward = 0.5 * middleForward;
        public double minLateral = 0.5 * middleLateral;
        public double minYaw = 0.5 * middleYaw;

        public double tolerance = 0.05;
    }

    public static POWER POWER = new POWER();

    private final DcMotorEx fl;
    private final DcMotorEx bl;
    private final DcMotorEx fr;
    private final DcMotorEx br;
    
    private double flTarget = 0;
    private double blTarget = 0;
    private double frTarget = 0;
    private double brTarget = 0;

    private boolean hasSetFlTarget = false;
    private boolean hasSetBlTarget = false;
    private boolean hasSetFrTarget = false;
    private boolean hasSetBrTarget = false;

    public double forwardMult = POWER.middleForward;
    public double lateralMult = POWER.middleLateral;
    public double yawMult = POWER.middleYaw;


    public BasicMecanumDrive(DcMotorEx fl, DcMotorEx bl, DcMotorEx fr, DcMotorEx br) {
        this.fl = fl;
        this.bl = bl;
        this.fr = fr;
        this.br = br;

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.FORWARD);
        stop();
    }

    @Override
    public double getForwardMultiplier() {
        return this.forwardMult;
    }

    @Override
    public boolean setForwardMultiplier(double newMultiplier) {
        final boolean didChange = newMultiplier == forwardMult;
        this.forwardMult = newMultiplier;
        return didChange;
    }
    
    @Override
    public double getLateralMultiplier() {
        return this.lateralMult;
    }

    @Override
    public boolean setLateralMultiplier(double newMultiplier) {
        final boolean didChange = newMultiplier == lateralMult;
        this.lateralMult = newMultiplier;
        return didChange;
    }

    @Override
    public double getYawMultiplier() {
        return this.yawMult;
    }

    @Override
    public boolean setYawMultiplier(double newMultiplier) {
        final boolean didChange = newMultiplier == yawMult;
        this.yawMult = newMultiplier;
        return didChange;
    }

    @Override
    public boolean powerMotors(double fl, double bl, double fr, double br) {
        boolean didChange = false;

        if(!near(fl, flTarget, POWER.tolerance)) {
            flTarget = fl;
            didChange = hasSetFlTarget = true;
        }
        
        if(!near(bl, blTarget, POWER.tolerance)) {
            blTarget = bl;
            didChange = hasSetBlTarget = true;
        }
        
        if(!near(fr, flTarget, POWER.tolerance)) {
            frTarget = fr;
            didChange = hasSetFrTarget = true;
        }
        
        if(!near(br, brTarget, POWER.tolerance)) {
            brTarget = br;
            didChange = hasSetBrTarget = true;
        }

        return didChange;
    }

    /**
     * Sets the multipliers to their max values. The effects of this remain 
     * until some other method changes them
     * 
     * @return Whether any multiplier was changed.
     */
    public boolean engageFastMode() {
        return setForwardMultiplier(POWER.maxForward) 
            && setLateralMultiplier(POWER.maxLateral) 
            && setYawMultiplier(POWER.maxYaw);
    }
    
    /**
     * Sets the multipliers to their middle values. The effects of this remain 
     * until some other method changes them.
     * 
     * @return Whether any multiplier was changed.
     */
    public boolean engageMiddleMode() {
        return setForwardMultiplier(POWER.middleForward) 
            && setLateralMultiplier(POWER.middleLateral) 
            && setYawMultiplier(POWER.middleYaw);
    }
    
    /**
     * Sets the multipliers to their middle values. The effects of this remain 
     * until some other method changes them.
     * 
     * @return Whether any multiplier was changed.
     */
    public boolean engageSlowMode() {
        return setForwardMultiplier(POWER.minForward) 
            && setLateralMultiplier(POWER.minLateral) 
            && setYawMultiplier(POWER.minYaw);
    }

    @Override
    public void periodic() {
        if(hasSetFlTarget) {
            fl.setPower(flTarget);
            hasSetFlTarget = false;
        }
        
        if(hasSetBlTarget) {
            bl.setPower(blTarget);
            hasSetBlTarget = false;
        }
        
        if(hasSetFrTarget) {
            fr.setPower(frTarget);
            hasSetFrTarget = false;
        }
        
        if(hasSetBrTarget) {
            br.setPower(brTarget);
            hasSetBrTarget = false;
        }

    }
}