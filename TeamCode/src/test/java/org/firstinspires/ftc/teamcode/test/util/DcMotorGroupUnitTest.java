package org.firstinspires.ftc.teamcode.test.hardware;

import clarson.ftc.faker.DcMotorImplExFake;
import clarson.ftc.faker.updater.ModularUpdater;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.hardware.DcMotorGroup;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.util.Util.*;
import static org.firstinspires.ftc.teamcode.test.TestUtil.*;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

@DisplayName("DcMotorGroupUnitTes")
class DcMotorGroupUnitTest {
    final DcMotorImplExFake[] motors = {
        new DcMotorImplExFake(312, 28 * 6000 / 312, 314.1592),
        new DcMotorImplExFake(435, 28 * 6000 / 435),
        new DcMotorImplExFake(1620, 28 * 6000 / 1620, 2 * 314.1592),
        new DcMotorImplExFake(6000, 28)
    };

    final DcMotorImplExFake[] ungroupedMotors = {
        new DcMotorImplExFake(312, 28 * 6000 / 312, 314.1592),
        new DcMotorImplExFake(435, 28 * 6000 / 435),
        new DcMotorImplExFake(1620, 28 * 6000 / 1620, 2 * 314.1592),
        new DcMotorImplExFake(6000, 28)
    };

    final ModularUpdater updater = new ModularUpdater();

    @BeforeEach 
    void initializeMotors() {
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].setMotorDisable();
        
        ungroupedMotors[1].setDirection(DcMotor.Direction.REVERSE);
        ungroupedMotors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ungroupedMotors[3].setMotorDisable();

        for(final DcMotorImplExFake motor : motors) {
            updater.register(motor);
        }
    }
    
    @DisplayName("Can construct")
    @Test
    void canConstruct() {
        assertDoesNotThrow(() -> new DcMotorGroup( motors ));
        assertDoesNotThrow(() -> new DcMotorGroup( new DcMotorEx[] {new DcMotorImplExFake(1, 2), new DcMotorImplExFake(3, 4)} ));
        assertDoesNotThrow(() -> new DcMotorGroup( new DcMotorImplExFake(1, 2), new DcMotorImplExFake(3, 4) ));
    }
    
    @Nested
    class PostConstruct {
        final DcMotorGroup group = new DcMotorGroup(motors);

        @Test 
        void setMotorEnable() {
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.isMotorEnabled() || motor == motors[3]));
            group.setMotorEnable();

            for(int i = 0; i < motors.length; i++) {
                assertTrue(motors[i].isMotorEnabled(), "motor " + i);
            }
        }
        
        @Test 
        void setMotorDisable() {
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.isMotorEnabled() || motor == motors[3]));
            group.setMotorDisable();

            for(int i = 0; i < motors.length; i++) {
                assertFalse(motors[i].isMotorEnabled(), "motor " + i);
            }
        }
        
        @Test 
        void setVelocity()  {
            // Making sure they are all RUN_USING_ENCODER
            group.setMotorEnable();
            group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, group.getMode());

            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getVelocity() == 0));
            group.setVelocity(10.0);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setMotorEnable();
                ungroupedMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ungroupedMotors[i].setVelocity(10.0);
                assertEquals(motors[i].getVelocity(), ungroupedMotors[i].getVelocity(), "motor " + i);
                assertEquals(10.0, motors[i].getVelocity(), "motor " + i);
                assertEquals(10.0, ungroupedMotors[i].getVelocity(), "motor " + i);
            }
        }
        
        @Test 
        void setVelocityAngleUnit() {
            group.setMotorEnable();
            group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, group.getMode());

            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getVelocity() == 0));
            group.setVelocity(10.0, AngleUnit.DEGREES);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setMotorEnable();
                ungroupedMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ungroupedMotors[i].setVelocity(10.0, AngleUnit.DEGREES);
                assertEquals(motors[i].getVelocity(), ungroupedMotors[i].getVelocity(), "motor " + i);
                assertFloatEquals(10.0 / 360.0 * motors[i].getData().ticksPerRev, motors[i].getVelocity(), 1e-13);
                assertFloatEquals(10.0 / 360.0 * motors[i].getData().ticksPerRev, ungroupedMotors[i].getVelocity(), 1e-13);
            }
        }
        
        @Test 
        void setPIDCoefficients() {
            // assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPIDCoefficients() ));
            final DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
            final PIDCoefficients coeffients = new PIDCoefficients(1, 2, 3);
            group.setPIDCoefficients(mode, coeffients);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setPIDCoefficients(mode, coeffients);
                assertEquals(motors[i].getPIDCoefficients(mode), ungroupedMotors[i].getPIDCoefficients(mode), "motor " + i);
            }
        }
        
        @Test 
        void setPIDFCoefficients() {
            // assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPIDFCoefficients() ));
            final DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
            final PIDFCoefficients coeffients = new PIDFCoefficients(1, 2, 3, 4);
            group.setPIDFCoefficients(mode, coeffients);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setPIDFCoefficients(mode, coeffients);
                assertEquals(motors[i].getPIDFCoefficients(mode), ungroupedMotors[i].getPIDFCoefficients(mode), "motor " + i);
            }
        }
        
        @Test 
        void setVelocityPIDFCoefficients() {
            
            // assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPIDFCoefficients() ));
            final PIDFCoefficients coeffients = new PIDFCoefficients(1, 2, 3, 4);
            group.setVelocityPIDFCoefficients(coeffients.p, coeffients.i, coeffients.d, coeffients.f);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setVelocityPIDFCoefficients(coeffients.p, coeffients.i, coeffients.d, coeffients.f);
                for(final PIDFCoefficients coef : new PIDFCoefficients[] { coeffients, ungroupedMotors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER) }) {
                    assertEquals(
                        motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p,
                        coef.p, 
                        "motor " + i
                    );
                    assertEquals(
                        motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i,
                        coef.i,
                        "motor " + i
                    );
                    assertEquals(
                        motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d,
                        coef.d ,
                        "motor " + i
                    );
                    assertEquals(
                        motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f,
                        coef.f,
                        "motor " + i
                    );
                }
                
            }
        }
        
        @Test 
        void setPositionPIDFCoefficients() {
            
            // assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPIDFCoefficients() ));
            final PIDFCoefficients coeffients = new PIDFCoefficients(1, 2, 3, 4);
            group.setPositionPIDFCoefficients(coeffients.p);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setPositionPIDFCoefficients(coeffients.p);
                assertEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p,
                    coeffients.p, 
                    "motor " + i
                );
                assertNotEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i,
                    coeffients.i,
                    "motor " + i
                );
                assertNotEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d,
                    coeffients.d ,
                    "motor " + i
                );
                assertNotEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f,
                    coeffients.f,
                    "motor " + i
                );

                assertEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p,
                    ungroupedMotors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p, 
                    "motor " + i
                );
                assertEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i,
                    ungroupedMotors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i,
                    "motor " + i
                );
                assertEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d,
                    ungroupedMotors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d ,
                    "motor " + i
                );
                assertEquals(
                    motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f,
                    ungroupedMotors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f,
                    "motor " + i
                );
            
            }
        }
        
        @Test 
        void setTargetPositionTolerance() {
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getTargetPositionTolerance() != 123));
            group.setTargetPositionTolerance(123);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setTargetPositionTolerance(123);
                assertEquals(motors[i].getTargetPositionTolerance(), ungroupedMotors[i].getTargetPositionTolerance(), "motor " + i);
            }
        }
        
        @Test 
        void setCurrentAlert() {
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getCurrentAlert(CurrentUnit.AMPS) != 123));
            group.setCurrentAlert(123, CurrentUnit.AMPS);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setCurrentAlert(123, CurrentUnit.AMPS);
                assertEquals(motors[i].getCurrentAlert(CurrentUnit.AMPS), ungroupedMotors[i].getCurrentAlert(CurrentUnit.AMPS), "motor " + i);
            }
        }
        
        @Test 
        void setDirection() {
            final DcMotor.Direction dir = DcMotor.Direction.REVERSE;
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getDirection() != dir || motor == motors[1]));
            group.setDirection(dir);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setDirection(dir);
                assertEquals(motors[i].getDirection(), ungroupedMotors[i].getDirection(), "motor " + i);
            }
        }
        
        @Test 
        void setPower() {
            
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPower() != 0.5));
            group.setMotorEnable(); // Make sure all can receive power
            group.setPower(0.5);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setMotorEnable();
                ungroupedMotors[i].setPower(0.5);
                assertEquals(motors[i].getPower(), ungroupedMotors[i].getPower(), "motor " + i);
            }
        }
        
        @Test 
        void setMode() {
            final DcMotor.RunMode dir = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getMode() != dir || motor == motors[1]));
            group.setMode(dir);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setMode(dir);
                assertEquals(motors[i].getMode(), ungroupedMotors[i].getMode(), "motor " + i);
            }
        }
        
        // @Test 
        // void setMotorType(MotorConfigurationType config) {}
        
        // @Test 
        // void setPowerFloat() {}
        
        @Test 
        void setTargetPosition() {
            
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getTargetPosition() != 123));
            group.setTargetPosition(123);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setTargetPosition(123);
                assertEquals(motors[i].getTargetPosition(), ungroupedMotors[i].getTargetPosition(), "motor " + i);
            }
        }
        
        @Test 
        void setZeroPowerBehavior() {
            
            final DcMotor.ZeroPowerBehavior dir = DcMotor.ZeroPowerBehavior.FLOAT;
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getZeroPowerBehavior() != dir || motor == motors[1]));
            group.setZeroPowerBehavior(dir);

            for(int i = 0; i < motors.length; i++) {
                ungroupedMotors[i].setZeroPowerBehavior(dir);
                assertEquals(motors[i].getZeroPowerBehavior(), ungroupedMotors[i].getZeroPowerBehavior(), "motor " + i);
            }
        }
        
        // @Test 
        // void close() {}
        
        // @Test 
        // void resetDeviceConfigurationForOpMode() {}
    
        @Test
        void isMotorEnabled() {
            // Only one is disabled
            assertTrue(group.isMotorEnabled());

            // All are enabled
            group.setMotorEnable();
            assertTrue(group.isMotorEnabled());

            // All are disabled
            group.setMotorDisable();
            assertFalse(group.isMotorEnabled());
        }

        @Test 
        void getVelocity() {
            // Making sure all the motors are enabled
            group.setMotorEnable();
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.isMotorEnabled()));

            // All are 0
            assertEquals(group.getVelocity(), 0);

            // One is 100
            motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[2].setVelocity(100.0);
            assertFloatEquals(25.0, group.getVelocity(), 1e-13);

            // All different numbers
            motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[0].setVelocity(123.0);
            
            motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[1].setVelocity(456.0);
            
            motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[2].setVelocity(678.0);
            
            motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[3].setVelocity(890.0);
            assertFloatEquals((123.0 + 456.0 + 678.0 + 890.0) / 4.0, group.getVelocity(), 1e-13);
        }

        @Test 
        void getTargetPositionTolerance() {
            // Making sure all the motors are enabled
            group.setMotorEnable();
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.isMotorEnabled()));

            // All are the same value
            assertEquals(group.getTargetPositionTolerance(), motors[0].getTargetPositionTolerance());

            // all are 0
            group.setTargetPositionTolerance(0);
            assertTrue(all(Arrays.asList(motors), (motor) -> motor.getTargetPositionTolerance() == 0));
            assertEquals(group.getTargetPositionTolerance(), 0);

            // One is 100
            motors[2].setTargetPositionTolerance(100);
            assertFloatEquals(25.0, group.getTargetPositionTolerance(), 1e-13);

            motors[0].setTargetPositionTolerance(123);
            motors[1].setTargetPositionTolerance(456);
            motors[2].setTargetPositionTolerance(678);
            motors[3].setTargetPositionTolerance(890);
            assertFloatEquals((123 + 456 + 678 + 890) / 4, group.getTargetPositionTolerance(), 1e-13);
        }
    
        @Test 
        void getDirection() {
            // Only one is REVERSE
            assertEquals(DcMotor.Direction.FORWARD, group.getDirection());

            // All are REVERSE
            group.setDirection(DcMotor.Direction.REVERSE);
            assertEquals(DcMotor.Direction.REVERSE, group.getDirection());

            // All are FORWARD
            group.setDirection(DcMotor.Direction.FORWARD);
            assertEquals(DcMotor.Direction.FORWARD, group.getDirection());

            // Three are REVERSE
            group.setDirection(DcMotor.Direction.REVERSE);
            motors[3].setDirection(DcMotor.Direction.FORWARD);
            assertEquals(DcMotor.Direction.REVERSE, group.getDirection());
        }
        
        @Test 
        void getRunMode() {
            // Only one is RUN_USING_ENCODER
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, group.getMode());

            // All are RUN_USING_ENCODER
            group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, group.getMode());

            // All are RUN_WITHOUT_ENCODER
            group.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, group.getMode());

            // Three are RUN_USING_ENCODER
            group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, group.getMode());
        }

        // @Override
        // void isBusy() {
        //     // No motors are busy
        //     assertFalse(group.isBusy());

        //     // All motors are busy
        //     group.setTargetPosition(300);
        //     group.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     group.setPower(1.0);

        //     assertTrue(all(Arrays.asList(motors), (motor) -> motor.getTargetPosition() == 300));
        //     assertTrue(all(Arrays.asList(motors), (motor) -> motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION));
        //     assertTrue(all(Arrays.asList(motors), (motor) -> motor.getPower() == 1.0));

        //     assertTrue(group.);

        //     // Letting the motors get to position
        // }
    }

}