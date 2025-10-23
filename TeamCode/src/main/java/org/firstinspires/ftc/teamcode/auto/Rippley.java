package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;

@Autonomous(name="Auto2: Blue Rippley", group="Linear OpMode")

//12419, -3902, -18093, -32248

public class Rippley extends LinearOpMode {
    
    private double ticksPerInches = 10331 / 30;
    private double ticksPerDegree = (-35248 - 12419) / (3 * 360);
    
    private void Turn(double ang) {
        frontLeft.setPower(Math.signum(ang)*mPWR);
        frontRight.setPower(Math.signum(ang)*-mPWR);
        backLeft.setPower(Math.signum(ang)*mPWR);
        backRight.setPower(Math.signum(ang)*-mPWR);
       
        
        startDeer = (backLeft.getCurrentPosition());
        
         while(
                 !(Math.abs(-backLeft.getCurrentPosition()
                 - startDeer - tickToDeg(ang) - 100) < 50
                )
            )
            {   telemetry.addData("startDeer", startDeer);
                telemetry.addData("CurAngle", imu.getRobotYawPitchRollAngles().getRoll());
                telemetry.addLine("DUCK");
                telemetry.addData("Front Left", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right", frontRight.getCurrentPosition());
                telemetry.addData("Back Left", backLeft.getCurrentPosition());
                telemetry.addData("Back Right", backLeft.getCurrentPosition());
                telemetry.addData("Time", (int) sleepTime(mPWR));
                telemetry.update(); }
    
        frontLeft.setPower(-mPWR);
        frontRight.setPower(mPWR);
        backLeft.setPower(-mPWR);
        backRight.setPower(mPWR);
        
        sleep((int) 50);
        
        frontLeft. setPower(0);
        frontRight.setPower(0);
        backLeft.  setPower(0);
        backRight. setPower(0);
    }
    
    private void MoveForward(double dist) {
        frontLeft.setPower(mPWR);
        frontRight.setPower(mPWR);
        backLeft.setPower(mPWR);
        backRight.setPower(mPWR);
        startDeer = (backLeft.getCurrentPosition());
         
         while(
                 !(Math.abs(-backLeft.getCurrentPosition() - tickToIn(dist) - startDeer) < 100
                 // true
                 // was 30 (avg 36)
                 // was 27 (avg 34)
                 // was 25
                 // was 21
                 // was 23
                 // 29.5 works on 0.3333 speed! :)
                )
            )
            {   telemetry.addLine("Fork Knife");
                telemetry.addData("Front Left", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right", frontRight.getCurrentPosition());
                telemetry.addData("Back Left", backLeft.getCurrentPosition());
                telemetry.addData("Back Right", backLeft.getCurrentPosition());
                telemetry.addData("Time", (int) sleepTime(mPWR));
                telemetry.update(); }
    
        frontLeft. setPower(-mPWR);
        frontRight.setPower(-mPWR);
        backLeft.  setPower(-mPWR);
        backRight. setPower(-mPWR);
        
        sleep((int) 50);
        
        frontLeft. setPower(-0);
        frontRight.setPower(-0);
        backLeft.  setPower(-0);
        backRight. setPower(-0);
    }
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private IMU imu = null;
    private double startDeer = 0;
    private double sleepTime (double mPWR){
        return (200 / 3) * (mPWR-0.3) + 83;
    }
    private double mPWR = 0.3333;
    
    @Override
    public void runOpMode(){
        
        telemetry.setMsTransmissionInterval(30);
        
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        
         frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        
        final DcMotorEx rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);

        final FlywheelTubeShooter shooter = new FlywheelTubeShooter.Builder(rightShooterMotor) 
            .setLeftFeeder(leftFeeder) 
            .setRightFeeder(rightFeeder)
            // .setRightReloadClassifier(rightReload)
            // .setLeftReloadClassifier(leftReload)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        
        
          telemetry.addData("Status", "Initialized");
        telemetry.update();

            waitForStart();
            
            // MoveForward(6);

            shooter.charge();
            
            shooter.setTelemetry(telemetry);
            while(shooter.getStatus()==FlywheelTubeShooter.Status.CHARGING){
                shooter.periodic();
                telemetry.update();
            }
            
            shooter.fireLeft();
            
            while(shooter.getStatus()==FlywheelTubeShooter.Status.FIRING || shooter.getStatus() == FlywheelTubeShooter.Status.CHARGING
            ){
                shooter.periodic();
                telemetry.update();
            }

            shooter.fireRight();
            
            while(shooter.getStatus()==FlywheelTubeShooter.Status.FIRING || shooter.getStatus() == FlywheelTubeShooter.Status.CHARGING
            ){
                shooter.periodic();
                telemetry.update();
            }
            
            intake.intakeGamePieces();
            
            shooter.reload();

            while(shooter.getStatus()==FlywheelTubeShooter.Status.FIRING || shooter.getStatus() == FlywheelTubeShooter.Status.CHARGING
            ){
                shooter.periodic();
                telemetry.update();
            }
            
            shooter.fire();
            
            while(shooter.getStatus()==FlywheelTubeShooter.Status.FIRING || shooter.getStatus() == FlywheelTubeShooter.Status.CHARGING
            ){
                shooter.periodic();
                telemetry.update();
            }
            
    //         MoveForward(21);
            
            

        
    
    // //seperated turning robot clockwise
    // //seperated turning robot clockwise
    // //seperated turning robot clockwise
    
    
    //              Turn(90);
                 
    //              MoveForward(45);
                 
    //              Turn(90);
                 
    //              MoveForward(30);
                 
    //              Turn(-100);
                 
                 
    
    }
     
     private double tickToDeg(double ticks){
        return ticks * ticksPerDegree;
     }
     
     
     
     private double tickToIn(double num){
            return num * ticksPerInches;
        }
    
    
    

        // 1: 30
        // 2: 30
        // 3: 29.5
        // 4: 30.5
        // 5: 30.5
        // 6: 30.5
}
