package clarson.ftc.faker.test;

import clarson.ftc.faker.DcMotorControllerExFake;
import clarson.ftc.faker.DcMotorImplExFake;
import clarson.ftc.faker.LynxUsbDeviceImplFake;
import clarson.ftc.faker.MotorData;
import clarson.ftc.faker.UnsupportedLynxUsbCommandException;
import clarson.ftc.faker.updater.ModularUpdater;
import clarson.ftc.faker.updater.Updateable;

import static clarson.ftc.faker.test.TestUtil.*;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import static com.qualcomm.hardware.lynx.commands.standard.LynxNack.StandardReasonCode;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.regex.Pattern;
import java.util.regex.Matcher;

import static java.util.concurrent.TimeUnit.SECONDS;

class LynxUsbDeviceImplFakeUnitTest {
    
    
    @DisplayName("Can construct")
    @Test
    void canConstruct() {
        assertDoesNotThrow(() -> new LynxUsbDeviceImplFake());   
    }

    @DisplayName("Can set Hardware Arrays")
    @Test
    void canSetHardwareArrays() {
        final LynxUsbDeviceImplFake lynx = new LynxUsbDeviceImplFake();
        assertDoesNotThrow(() -> {
            lynx.setMotors(new DcMotorImplExFake[] {
                new DcMotorImplExFake(312, 576.6),
                new DcMotorImplExFake(1600, 100, 123.4),
                new DcMotorImplExFake(312, -576.6, 1000)
            });

            // TODO: Add digital channels and analog inputs!
        });
    }

    @DisplayName("Can arm with no modules")
    @Test
    void canArm() {
        final LynxUsbDeviceImplFake lynx = new LynxUsbDeviceImplFake();
        assertDoesNotThrow(() -> lynx.armOrPretend());
        assertEquals(RobotArmingStateNotifier.ARMINGSTATE.ARMED, lynx.getArmingState());
    }

    @DisplayName("Construction Dependent")
    @Nested
    class PostConstruct {
        LynxUsbDeviceImplFake lynx;
        LynxModule module;
        DcMotorImplExFake[] motors = new DcMotorImplExFake[] {
            new DcMotorImplExFake(312, 576.6),
            new DcMotorImplExFake(1600, 100, 123.4),
            new DcMotorImplExFake(312, -576.6, 1000)
        };

        @BeforeEach
        void createLynxUsbDevice() {
            lynx = new LynxUsbDeviceImplFake();
 
            lynx.setMotors(motors);

            // TODO: Add digital channels and analog inputs!
        }

        @DisplayName("Create LynxModule") 
        @Nested
        class CreateModule {
            @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
            @Test
            void canCreateDescriptionBuilder() {
                assertDoesNotThrow(() -> {
                    new LynxModuleDescription.Builder(-1, true)
                        .setUserModule()
                        // .setSystemSynthetic()
                        .build();
                });
            }

            @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
            @Test
            void canAddLynxModule() {
                // fail("Exit tests");
                try {
                    module = lynx.getOrAddModule(
                        new LynxModuleDescription.Builder(-1, true)
                            .setUserModule()
                            // .setSystemSynthetic()
                            .build()    
                    );
                }  catch (InterruptedException unused) {
                    // Do nothing...
                } catch(RobotCoreException err) {
                    fail("RobotCoreException caught in addLynxModule(): " + err.getMessage());
                }
            }
        
            @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
            @DisplayName("Can arm with module")
            @Test
            void canArmWithModule() {
                final LynxUsbDeviceImplFake lynx = new LynxUsbDeviceImplFake();

                try {
                    module = lynx.getOrAddModule(
                        new LynxModuleDescription.Builder(-1, true)
                            .setUserModule()
                            // .setSystemSynthetic()
                            .build()    
                    );
                }  catch (InterruptedException unused) {
                    // Do nothing...
                } catch(RobotCoreException err) {
                    fail("RobotCoreException caught in addLynxModule(): " + err.getMessage());
                }

                assertDoesNotThrow(() -> lynx.armOrPretend());
                assertEquals(RobotArmingStateNotifier.ARMINGSTATE.ARMED, lynx.getArmingState());
            }
        }

        @DisplayName("Miscellaneous Transmit")
        @Nested 
        class MiscTransmit {
            @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
            @BeforeEach
            void addLynxModule() {
                // fail("Exit tests");
                try {
                    module = lynx.getOrAddModule(
                        new LynxModuleDescription.Builder(-1, true)
                            .setUserModule()
                            // .setSystemSynthetic()
                            .build()    
                    );
                }  catch (InterruptedException unused) {
                    // Do nothing...
                } catch(RobotCoreException err) {
                    fail("RobotCoreException caught in addLynxModule(): " + err.getMessage());
                }
                

                try {
                    lynx.armOrPretend();
                } catch(RobotCoreException | InterruptedException err) {
                    fail(err);
                }
            }

            @DisplayName("Responds with LynxNack on Unrecognized")
            @Test
            void nacksOnUnrecognzed() {
                // Getting the test command. The default response only avoids a NullPointerException
                final LynxMessage defaultResponse = new LynxMessage(module) {
                    @Override public boolean isDangerous() {
                        return false;
                    }

                    @Override public void fromPayloadByteArray(byte[] unused) {}

                    @Override public byte[] toPayloadByteArray() {
                        return null;
                    }

                    @Override public int getCommandNumber() {
                        return 0;
                    }
                };
            
                final LynxCommand<LynxMessage> unrecognized = new LynxCommand<>(module, defaultResponse) {
                    @Override public boolean isDangerous() {
                        return false;
                    }

                    @Override public void fromPayloadByteArray(byte[] unused) {}

                    @Override public byte[] toPayloadByteArray() {
                        return null;
                    }

                    @Override public int getCommandNumber() {
                        return 0;
                    }
                };
            
                // Assert that nothing has been received
                assertTrue(unrecognized.isAckable());
                assertFalse(unrecognized.isNackReceived());
                assertEquals(null, unrecognized.getNackReceived());

                // Making sure that the method doesn't misevaluate this message
                // It should only throw if it cannot be responded to
                assertDoesNotThrow(() -> {
                    lynx.transmit(unrecognized);
                });

                // Assert that a nack has been received and correct
                assertTrue(unrecognized.isNackReceived());
                assertNotEquals(null, unrecognized.getNackReceived());
                assertEquals(
                    StandardReasonCode.COMMAND_ROUTING_ERROR, 
                    unrecognized.getNackReceived().getNackReasonCodeAsEnum()
                );
            }

            @Disabled("Cannot yet craft a supported but unrespondable command")
            @DisplayName("Throws on Unrespondable LynxMessage")
            @Test
            void throwsOnUnrespondable() {
                // Making sure that we can actually do bulk gets and sends 
                final GetBulkData nestedTested = new GetBulkData();
                assumeFalse(doesThrow(nestedTested::getResponse));

                // Creating our command to transmit
                final LynxGetBulkInputDataCommand unrecognized = new LynxGetBulkInputDataCommand(module) {
                    // @Override
                    // public boolean isResponseExpected() {
                    //     return false;
                    // }
                };
            
                Exception e = assertThrowsExactly(UnsupportedLynxUsbCommandException.class, () -> {
                    lynx.transmit(unrecognized);
                });

                // Asserting that the message is the unrespondable message.
                // We test this by matching it against its String.format 
                // template, turned into a Regex 
                assertFalse(Pattern.matches(
                    "^" + LynxUsbDeviceImplFake.UN_NACKABLE_MSG.replace("%s", ".+?") + "$",
                    e.getMessage()
                ));
                assertTrue(Pattern.matches(
                    "^" + LynxUsbDeviceImplFake.UNRESPONDABLE_MSG.replace("%s", ".+?") + "$",
                    e.getMessage()
                ));
            }
            
            @DisplayName("Throws on Unnackable LynxMessage")
            @Test
            void throwsOnUnnackable() {
                final LynxMessage unrecognized = new LynxMessage(module) {
                    @Override
                    public boolean isDangerous() {
                        return false;
                    }

                    @Override 
                    public void fromPayloadByteArray(byte[] unused) {
                        // Do nothing.
                    }

                    @Override 
                    public byte[] toPayloadByteArray() {
                        return null;
                    }

                    @Override
                    public int getCommandNumber() {
                        return 0;
                    }
                };
            
                Exception e = assertThrowsExactly(UnsupportedLynxUsbCommandException.class, () -> {
                    lynx.transmit(unrecognized);
                });

                // Asserting that the message is the unnackable message.
                // We test this by matching it against its String.format 
                // template, turned into a Regex 
                assertTrue(Pattern.matches(
                    "^" + LynxUsbDeviceImplFake.UN_NACKABLE_MSG.replace("%s", ".+?") + "$",
                    e.getMessage()
                ));
                assertFalse(Pattern.matches(
                    "^" + LynxUsbDeviceImplFake.UNRESPONDABLE_MSG.replace("%s", ".+?") + "$",
                    e.getMessage()
                ));
            }
        } 
    
        @DisplayName("Get Bulk Data")
        @Nested
        class GetBulkData {
            @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
            @BeforeEach
            void addLynxModule() {
                // fail("Exit tests");
                try {
                    module = lynx.getOrAddModule(
                        new LynxModuleDescription.Builder(-1, true)
                            .setUserModule()
                            // .setSystemSynthetic()
                            .build()    
                    );
                }  catch (InterruptedException unused) {
                    // Do nothing...
                } catch(RobotCoreException err) {
                    fail("RobotCoreException caught in addLynxModule(): " + err.getMessage());
                }
                
                try {
                    lynx.armOrPretend();
                } catch(RobotCoreException | InterruptedException err) {
                    fail(err);
                }
            }

            // NOTE: Hardware is added in the createLynxUsbDevice() method
            @DisplayName("Bulk Data Command Gets response")
            @Test
            void getResponse() throws InterruptedException, LynxNackException {
                final LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);

                assertDoesNotThrow(() -> module.validateCommand(command));

                assertTrue(command.isResponseExpected());
                assertTrue(command.isAckable());
                assertFalse(command.isAckOrResponseReceived());

                final LynxGetBulkInputDataResponse response = command.sendReceive();

                // Make sure the response was recieved at the command level
                assertTrue(command.isAckOrResponseReceived());
                assertNotEquals(null, response);
            }

            private DcMotorControllerExFake getController(DcMotorImplExFake motor) {
                return (DcMotorControllerExFake) (motor.getController());
            }

            @DisplayName("Bulk Response is correct")
            @Test
            void isResponseCorrect() throws InterruptedException, LynxNackException {
                // Add some variation to the motors
                motors[0].setVelocity(312);
                motors[1].setVelocity(628);
                motors[1].addAngularVelOffset(2 * Math.PI / 100 * 100); // 100 ticks / sec
                motors[2].setTargetPositionTolerance(2);
                motors[2].setTargetPosition(250);
                motors[2].setMode(DcMotorImplExFake.RunMode.RUN_TO_POSITION);

                // Get the command, verify, and response
                final LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);

                assertDoesNotThrow(() -> module.validateCommand(command));

                assertTrue(command.isResponseExpected());
                assertTrue(command.isAckable());
                assertFalse(command.isAckOrResponseReceived());

                final LynxGetBulkInputDataResponse response = command.sendReceive();

                // Make sure the response was recieved at the command level
                assertTrue(command.isAckOrResponseReceived());
                assertNotEquals(null, response);

                //#region DEV START: Logging the payload as bytes
                // [00]  uint8_t  digitalInputs;      // DIGITAL_START

                // [01]  int32_t  motor0position_enc; // ENCODER_START
                // [05]  int32_t  motor1position_enc;
                // [09]  int32_t  motor2position_enc;
                // [13]  int32_t  motor3position_enc;

                // [17]  uint8_t  motorStatus;        // STATUS_START

                // [18]  int16_t  motor0velocity_cps; // VELOCITY_START
                // [20]  int16_t  motor1velocity_cps;
                // [22]  int16_t  motor2velocity_cps;
                // [24]  int16_t  motor3velocity_cps;

                // [26]  int16_t  analog0_mV;         // ANALOG_START
                // [28]  int16_t  analog1_mV;
                // [30]  int16_t  analog2_mV;
                // [32]  int16_t  analog3_mV;
                int j = 0;
                for(byte b : response.toPayloadByteArray()) {
                    System.out.println("[is correct] At <" + j + ">: 0x" + Integer.toHexString(b));
                    j++;
                }
                //#endregion DEV END

                // Make sure the data is correct
                for(int i = 0; i < motors.length; i++) {
                    final DcMotorImplExFake motor = motors[i];
                    final int portNumber = i; 
                    final MotorData data = getController(motor).getData(0);
                    System.out.println("[is correct] port: " + portNumber);
                    System.out.println("[is correct] getCur: " + motor.getCurrentPosition());
                    assertWithin(data.position, response.getEncoder(portNumber), 0.5);
                    assertWithin(data.getActualVelocity(), response.getVelocity(portNumber), 0.5);

                    if(motor.getMode() == DcMotorImplExFake.RunMode.RUN_TO_POSITION) {
                        assertEquals(
                            Math.abs(data.targetPosition - data.position) < data.tolerance, 
                            response.isAtTarget(portNumber)
                        );
                    }

                    // NOTE: Not testing the isOverCurrent as that relies on a flawed current reading
                }

                // TODO: Verify digital channel and analog inputs
            }
            
            /* // TODO: Move these tests into a LynxModuleUnitTest class
            @DisplayName("Bulk OFF never issues")
            @Test
            void bulkOffNeverIsses() {}

            @DisplayName("Bulk AUTOMATIC issues only on repeated calls")
            @Test
            void bulkAutoIssuesOnRepeatedCalls() {}

            @DisplayName("Bulk MANUAL issues only when clear")
            @Test
            void bulkAutoIssuesWhenClear() {} */
        }

    }
}