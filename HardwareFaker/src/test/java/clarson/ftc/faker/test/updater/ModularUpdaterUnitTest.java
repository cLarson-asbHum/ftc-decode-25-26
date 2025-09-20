package clarson.ftc.faker.test.updater;

import clarson.ftc.faker.ContinuousServoData;
import clarson.ftc.faker.CRServoImplExFake;
import clarson.ftc.faker.DcMotorImplExFake;
import clarson.ftc.faker.DcMotorControllerExFake;
import clarson.ftc.faker.LynxModuleHardwareFake;
import clarson.ftc.faker.LynxUsbDeviceImplFake;
import clarson.ftc.faker.MotorData;
import clarson.ftc.faker.ServoData;
import clarson.ftc.faker.updater.ModularUpdater;
import clarson.ftc.faker.updater.Updateable;
import clarson.ftc.faker.updater.Updater;

import static clarson.ftc.faker.test.TestUtil.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxKeepAliveCommand;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.Arguments;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.stream.Stream;

import static java.util.concurrent.TimeUnit.SECONDS;

class ModularUpdaterUnitTest {
    @DisplayName("Can construct")
    @Test
    void canConstruct() {
        assertDoesNotThrow(() -> new ModularUpdater());
    }

    @DisplayName("Post Construct")
    @Nested
    class PostConstruct {
        final LynxModuleDescription controlDesc = new LynxModuleDescription.Builder(1, true)
            .setUserModule()
            .build();
        final LynxModuleDescription expansionDesc = new LynxModuleDescription.Builder(2, false)
            .setUserModule()
            .build();

        LynxUsbDeviceImplFake device = new LynxUsbDeviceImplFake();
        LynxModuleHardwareFake control;
        LynxModuleHardwareFake expansion;

        ModularUpdater updater;

        @BeforeEach
        void construct() throws InterruptedException, RobotCoreException {
            updater = new ModularUpdater();
            control = (LynxModuleHardwareFake) device.getOrAddModule(controlDesc);
            expansion = (LynxModuleHardwareFake) device.getOrAddModule(expansionDesc);
            
            try {
                device.armOrPretend();
            } catch(RobotCoreException | InterruptedException err) {
                fail(err);
            }
        }

        @DisplayName("Can register")
        @Test
        void canRegister() throws InterruptedException, RobotCoreException {
            // No module registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(312, 576.6)));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1600, 100)));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(180)));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(60)));
            
            // Control Hub  registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1, 2), control));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(1, 2), control));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(3, 4), control));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(3, 4), control));
            
            // Expansion Hub  registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1, 2), expansion));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(1, 2), expansion));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(3, 4), expansion));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(3, 4), expansion));
        }

        @DisplayName("Register fails on duplicates and passes otherwise")
        @Test
        void duplicateRegistersFail() throws InterruptedException, RobotCoreException {

            // No module registers
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(updater.register(motorNone));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100)));
            assertTrue(updater.register(servoNone));
            assertTrue(updater.register(new CRServoImplExFake(180)));
            assertFalse(updater.register(motorNone));
            assertFalse(updater.register(servoNone));
            

            // Control hub module registers
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoCont = new CRServoImplExFake(60);
            assertTrue(updater.register(motorCont, control));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100), control));
            assertTrue(updater.register(servoCont, control));
            assertTrue(updater.register(new CRServoImplExFake(180), control));
            assertFalse(updater.register(motorCont, control));
            assertFalse(updater.register(servoCont, control));
            
            // Expansion hub registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoExp = new CRServoImplExFake(60);
            assertTrue(updater.register(motorExp, expansion));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100), expansion));
            assertTrue(updater.register(servoExp, expansion));
            assertTrue(updater.register(new CRServoImplExFake(180), expansion));
            assertFalse(updater.register(motorExp, expansion));
            assertFalse(updater.register(servoExp, expansion));

            // Duplicates, with different modules
            assertFalse(updater.register(motorCont)); // Equal to nul hub
            assertFalse(updater.register(servoCont)); // Equal to nul hub

            assertFalse(updater.register(motorCont, expansion));
            assertFalse(updater.register(servoCont, expansion));

            assertFalse(updater.register(motorExp)); // Equal to null hub
            assertFalse(updater.register(servoExp)); // Equal to null hub

            assertFalse(updater.register(motorExp, control));
            assertFalse(updater.register(servoExp, control));

            assertFalse(updater.register(motorNone, control));
            assertFalse(updater.register(servoNone, control));

            assertFalse(updater.register(motorNone, expansion));
            assertFalse(updater.register(servoNone, expansion));

        }

        @DisplayName("Can unregister")
        @Test
        void canUnregister() {
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));

            // Unregistering null hub  updateables.
            assertDoesNotThrow(() -> updater.unregister(motorNone));
            assertDoesNotThrow(() -> updater.unregister(servoNone));
            
            // Unregistering control hub updateables. 
            assertDoesNotThrow(() -> updater.unregister(motorCont));
            assertDoesNotThrow(() -> updater.unregister(servoCont));

            // Unregistering expansion hub updateables. 
            assertDoesNotThrow(() -> updater.unregister(motorExp));
            assertDoesNotThrow(() -> updater.unregister(servoExp));
        }

        @DisplayName("Unregistering something nonexistent fails; passes otherwise")
        @Test
        void unregisterNonexistentFails() {
            
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertFalse(() -> updater.unregister(motorNone));
            assertFalse(() -> updater.unregister(servoNone));
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertFalse(() -> updater.unregister(motorCont));
            assertFalse(() -> updater.unregister(servoCont));
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertFalse(() -> updater.unregister(motorExp));
            assertFalse(() -> updater.unregister(servoExp));
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));

            // Unregistering null hub  updateables.
            assertTrue(() -> updater.unregister(motorNone));
            assertFalse(() -> updater.unregister(motorNone));
            assertTrue(() -> updater.unregister(servoNone));
            assertFalse(() -> updater.unregister(servoNone));
            
            // Unregistering control hub updateables. 
            assertTrue(() -> updater.unregister(motorCont));
            assertFalse(() -> updater.unregister(motorCont));
            assertTrue(() -> updater.unregister(servoCont));
            assertFalse(() -> updater.unregister(servoCont));

            // Unregistering expansion hub updateables. 
            assertTrue(() -> updater.unregister(motorExp));
            assertFalse(() -> updater.unregister(motorExp));
            assertTrue(() -> updater.unregister(servoExp));
            assertFalse(() -> updater.unregister(servoExp));
        }

        @DisplayName("Has registered identifies registered")
        @Test
        void registerCanReturnTrue() {
            
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            assertTrue(updater.hasRegistered(motorNone));
            assertTrue(updater.hasRegistered(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            assertTrue(updater.hasRegistered(motorCont));
            assertTrue(updater.hasRegistered(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));
            assertTrue(updater.hasRegistered(motorExp));
            assertTrue(updater.hasRegistered(servoExp));
        }

        @DisplayName("Has registered identifies unregistered")
        @Test
        void registerCanReturnFalse() {
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertFalse(updater.hasRegistered(motorNone));
            assertFalse(updater.hasRegistered(servoNone));
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            assertTrue(updater.unregister(servoNone));
            assertTrue(updater.unregister(motorNone));
            assertFalse(updater.hasRegistered(motorNone));
            assertFalse(updater.hasRegistered(servoNone));

            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertFalse(updater.hasRegistered(motorCont));
            assertFalse(updater.hasRegistered(servoCont));
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            assertTrue(updater.unregister(servoCont));
            assertTrue(updater.unregister(motorCont));
            assertFalse(updater.hasRegistered(motorCont));
            assertFalse(updater.hasRegistered(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertFalse(updater.hasRegistered(motorExp));
            assertFalse(updater.hasRegistered(servoExp));
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));
            assertTrue(updater.unregister(servoExp));
            assertTrue(updater.unregister(motorExp));
            assertFalse(updater.hasRegistered(motorExp));
            assertFalse(updater.hasRegistered(servoExp));
        }

        @DisplayName("Post Register")
        @Nested
        class PostRegister {
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final DcMotorControllerExFake controlHubController = new DcMotorControllerExFake(null);
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(new MotorData(100, 200), controlHubController);
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(200, 300);

            // final DcMotorImplExFake[] motors = { motorNone, motorCont, motorExp };
            final DcMotorImplExFake[] motors = { motorCont };

            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);

            final MotorData motorNoneData = motorNone.getData();
            final ServoData servoNoneData = servoNone.getData();
            final MotorData motorContData = motorCont.getData();
            final ServoData servoContData = servoCont.getData();
            final MotorData motorExpData = motorExp.getData();
            final ContinuousServoData servoExpData = servoExp.getData();

            boolean expectingUpdateAll = false;
            ModularUpdater mockUpdater;


            @BeforeEach
            void addHardware() {
                assertTrue(updater.register(motorNone, null));
                assertTrue(updater.register(servoNone, null));
                assertTrue(updater.register(motorCont, control));
                assertTrue(updater.register(servoCont, control));
                assertTrue(updater.register(motorExp, expansion));
                assertTrue(updater.register(servoExp, expansion));
                
                device.setMotors(motors);
                controlHubController.setLynxModule(control);

            }

            @DisplayName("UpdateAll accurrately updates all added hardware")
            @Test
            void updateAllUpdatesAllHardware() {
                // Getting the original positions

                final double motorNoneOg = motorNoneData.position;
                final double servoNoneOg = servoNoneData.position;
                final double motorContOg = motorContData.position;
                final double servoContOg = servoContData.position;
                final double motorExpOg = motorExpData.position;
                final double servoExpOg = servoExpData.position;

                // Setting the powers
                updater.setUpdatingEnabledAll(false); // Preventing automatic updates
                motorNone.setPower(1.0);
                servoNone.setPower(1.0);

                motorCont.setPower(-1.0);
                servoCont.setPower(0.5);

                motorExp.setPower(0);
                servoExp.setDirection(DcMotorImplExFake.Direction.REVERSE);
                servoExp.setPower(1.0);
                servoExp.setAngularVelOffset(1.5 * 2 * Math.PI * servoExpData.maxRevsPerSec);
                updater.setUpdatingEnabledAll(true); // Allowing manual updates

                // Updating and comparing
                updater.updateAll(1.0);
                
                final double eps = 1e-13;
                assertFloatEquals(motorNoneOg + motorNoneData.maxTicksPerSec, motorNoneData.position, eps);
                assertFloatEquals(servoNoneOg + servoNoneData.maxRevsPerSec, servoNoneData.position, eps);

                assertFloatEquals(motorContOg - motorContData.maxTicksPerSec, motorContData.position, eps);
                assertFloatEquals(servoContOg + 0.5 * servoContData.maxRevsPerSec, servoContData.position, eps);

                assertFloatEquals(motorExpOg, motorExpData.position, eps);
                assertFloatEquals(servoExpOg + 0.5 * servoExpData.maxRevsPerSec, servoExpData.position, eps);
            }

            @DisplayName("UpdateAll ignores unregistered hardware")
            @Test
            void updateAllIgnoresUnregisteredHardware() {
                // Setting powers and doing and checkinig that they can move upon update
                updateAllUpdatesAllHardware();

                // Getting the data
                final MotorData motorNoneData = motorNone.getData();
                final ServoData servoNoneData = servoNone.getData();
                final MotorData motorContData = motorCont.getData();
                final ServoData servoContData = servoCont.getData();
                final MotorData motorExpData = motorExp.getData();
                final ServoData servoExpData = servoExp.getData();

                final double motorNoneOg = motorNoneData.position;
                final double servoNoneOg = servoNoneData.position;
                final double motorContOg = motorContData.position;
                final double servoContOg = servoContData.position;
                final double motorExpOg = motorExpData.position;
                final double servoExpOg = servoExpData.position;

                // Unregistering
                assertTrue(() -> updater.unregister(motorNone));
                assertTrue(() -> updater.unregister(servoNone));
                assertTrue(() -> updater.unregister(motorCont));
                assertTrue(() -> updater.unregister(servoCont));
                assertTrue(() -> updater.unregister(motorExp));
                assertTrue(() -> updater.unregister(servoExp));

                // Updating and comparing
                updater.updateAll(1.0);
                
                final double eps = 1e-13;
                assertFloatEquals(motorNoneOg, motorNoneData.position, eps);
                assertFloatEquals(servoNoneOg, servoNoneData.position, eps);

                assertFloatEquals(motorContOg, motorContData.position, eps);
                assertFloatEquals(servoContOg, servoContData.position, eps);

                assertFloatEquals(motorExpOg, motorExpData.position, eps);
                assertFloatEquals(servoExpOg, servoExpData.position, eps);
            }
        
            /**
             * Mocks a ModularUpdater, asserting that updateall is called 
             * from updateAllOnlyIfCacheOutdated only if expectingUpdateAll is true.
             * Test should set expectingUpdateAll to true when an update is expected;
             * otherwise, set it to false.
             * 
             * updateAllOnlyIfCacheOutdated also asserts that its return value accurately 
             * represents whether updateAll was called. This behavior is checked no matter
             * the value of expectingUpdateAll.
             */
            class ModularUpdaterMock extends ModularUpdater implements Updater {
                public boolean isInCacheUpdateStack = false;
                public boolean updateAllCalled = false;
                private final ModularUpdater actualUpdater;

                ModularUpdaterMock(ModularUpdater updater) {
                    this.actualUpdater = updater;
                    // this.registered = updater.registered;

                    // Adding all the motors
                    if(updater.hasRegistered(motorNone)) this.registered.put(motorNone, null);
                    if(updater.hasRegistered(motorCont)) this.registered.put(motorCont, control);
                    if(updater.hasRegistered(motorExp))  this.registered.put(motorExp,  expansion);
                }

                @Override
                public void updateAll(double deltaSec) {
                    System.out.println("[mock updateAll] did! <" + this.toString() + ">");
                    updateAllCalled = true;

                    if(isInCacheUpdateStack) {
                        assertTrue(expectingUpdateAll, "updateAll called from cache only when expected");
                    }
                    
                    actualUpdater.updateAll(deltaSec);
                }

                @Override
                public void setUpdatingEnabledAll(boolean newE) {
                    actualUpdater.setUpdatingEnabledAll(newE);
                }

                @Override
                public void rememberEnablingStatus() {
                    actualUpdater.rememberEnablingStatus();
                }

                @Override
                public boolean applyAndForgetEnablingStatus() {
                    return actualUpdater.applyAndForgetEnablingStatus();
                }

                @Override
                public boolean updateAllIfCacheOutdated(
                    double deltaSec, 
                    Updateable up, 
                    LynxDekaInterfaceCommand<?> command, 
                    String tag
                ) {
                    // didJustUpdateBulkData = false;
                    awaitUpdateAll(List.of(this));

                    final LynxModuleHardwareFake upModule = this.registered.get(up);
                    final LynxModuleHardwareFake controllerModule = ((DcMotorControllerExFake) ((DcMotorImplExFake) up).getController()).getLynxModule();

                    System.out.println("[update all cache] controller is control: " + (controllerModule == control));
                    System.out.println("[update all cache] controller is exp: " + (controllerModule == expansion));
                    System.out.println("[update all cache] up is control: " + (upModule == control));
                    System.out.println("[update all cache] up is exp: " + (upModule == expansion));

                    assertEquals(
                        upModule, 
                        controllerModule, 
                        "they have the same module"
                    );

                    // updateAllIfCacheOutdated sets didJustUpdateBulkData if a LynxGetBulkInputDataCommand was sent
                    // This is because of the LynxUsbDeviceImplFakeMock we set before each test.
                    final boolean result = super.updateAllIfCacheOutdated(deltaSec, up, command, tag);
                    assertEquals(expectingUpdateAll, result, "result of updateAllIfCacheOutdated represents updateAll invokation");
                    // assertEquals(expectingUpdateAll, updateAllCalled, "updateAll called when expected");
                    // if(this.registered.get(up).getBulkCachingMode() != LynxModule.BulkCachingMode.OFF) {
                    //     // Bulk data gets are not done during OFF, even though data is obtained
                    //     assertEquals(expectingUpdateAll, didJustUpdateBulkData, "BulkDataCommand was transmitted to LynxUsbDeviceImpl");
                    // }

                    endAwait(List.of(this));
                    return result;
                }
            
                @Override
                public boolean isCacheOutdated(Updateable up, LynxDekaInterfaceCommand<?> command, String tag) {
                    return actualUpdater.isCacheOutdated(up, command, tag);
                }
            }
            

            private void awaitUpdateAll(Collection<ModularUpdaterMock> mocks) {
                for(final ModularUpdaterMock mock : mocks) {
                    mock.isInCacheUpdateStack = true;
                    mock.updateAllCalled = false;
                }
            }

            private void endAwait(Collection<ModularUpdaterMock> mocks) {
                for(final ModularUpdaterMock mock : mocks) {
                    assertEquals(expectingUpdateAll, mock.updateAllCalled, "updateAll called when expected, <" + mock + ">");
                    mock.isInCacheUpdateStack = false;
                }
            }

            @DisplayName("Bulk Cache UpdateAll")
            @Nested
            class BulkCacheUpdateAll {
                boolean didJustUpdateBulkData = false;

                /**
                 * Mocks LynxUsbDeviceImplFake, asserting that BulkDataCommands are transmitted
                 * when expectingUpdateAll (which also checks for bulk data gets) is issued.
                 */
                class LynxUsbDeviceImplFakeMock extends LynxUsbDeviceImplFake {
                    private final LynxUsbDeviceImplFake actualDevice;

                    LynxUsbDeviceImplFakeMock(LynxUsbDeviceImplFake device) {
                        this.actualDevice = device;
                    }

                    @Override
                    public void transmit(LynxMessage message) {
                        didJustUpdateBulkData = 
                            message instanceof LynxGetBulkInputDataCommand
                            || message instanceof LynxGetMotorEncoderPositionCommand
                            ||message instanceof LynxIsMotorAtTargetCommand;
                        actualDevice.transmit(message);
                    }
                }
                
                @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
                @BeforeEach
                void mockLynxModuleHardwareFake() throws InterruptedException, RobotCoreException {
                    // Mocking the LynxUsbDeviceImplFake so that we can check the behvior of transmitting BulkDataCommands
                    device = new LynxUsbDeviceImplFakeMock(device);
                    control = (LynxModuleHardwareFake) device.getOrAddModule(controlDesc);
                    expansion = (LynxModuleHardwareFake) device.getOrAddModule(expansionDesc);

                    device.armOrPretend();
                    controlHubController.setLynxModule(control);


                    // Regregisterign the hardware but with the mock lynx modules
                    assertTrue(updater.unregister(motorNone));
                    assertTrue(updater.unregister(servoNone));
                    assertTrue(updater.unregister(motorCont));
                    assertTrue(updater.unregister(servoCont));
                    assertTrue(updater.unregister(motorExp));
                    assertTrue(updater.unregister(servoExp));

                    assertTrue(updater.register(motorNone, null));
                    assertTrue(updater.register(servoNone, null));
                    assertTrue(updater.register(motorCont, control));
                    assertTrue(updater.register(servoCont, control));
                    assertTrue(updater.register(motorExp, expansion));
                    assertTrue(updater.register(servoExp, expansion));

                    assertTrue(updater.hasRegistered(motorCont));
                    
                    mockUpdater = new ModularUpdaterMock(updater);
                    assertTrue(mockUpdater.hasRegistered(motorCont));
                    
                    // Disabling automatic updates.
                    for(final DcMotorImplExFake motor : motors) {
                        motor.forget(updater);
                        motor.forget(mockUpdater);
                    }
                }

                @Disabled("updateAllIfCacheOutdated no longer transmits commands, and shouldn't be expected to")
                @DisplayName("LynxUsbDeviceImplFakeMock sets didJustUpdateBulkData")
                @Test
                void usbDeviceMockSetsDidJust() {
                    assertFalse(didJustUpdateBulkData);
                    device.transmit(new LynxGetBulkInputDataCommand(control));
                    assertTrue(didJustUpdateBulkData);

                    device.transmit(new LynxKeepAliveCommand(control, false));
                    assertFalse(didJustUpdateBulkData);
                    
                    // Testing that everything also works on the expansion hub, not just the control
                    device.transmit(new LynxGetBulkInputDataCommand(expansion));
                    assertTrue(didJustUpdateBulkData);

                    device.transmit(new LynxKeepAliveCommand(expansion, false));
                    assertFalse(didJustUpdateBulkData);
                }
                
                @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
                @DisplayName("Default mode is OFF")
                @Test
                void defaultModeIsOff() {
                    assertEquals(LynxModule.BulkCachingMode.OFF, control.getBulkCachingMode());
                    assertEquals(LynxModule.BulkCachingMode.OFF, expansion.getBulkCachingMode());
                }

                @DisplayName("BulkCachingMode.AUTO updates on clear")
                @Test
                void autoUpdatesOnClear() {
                    System.out.println("🚒");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Bulk cache has been obtained, but the next command is new, so no update
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetBulkInputDataCommand(control)
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double thirdPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition); // NOTE: Delay from getCurrentPosition is disabled


                    // Cleaing and seeing that an update comes through
                    control.clearBulkCache();
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxIsMotorAtTargetCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double qincePosition = motorCont.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(secondPosition, qincePosition);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                }

                @DisplayName("isCacheOutdated is correct for BulkCachingMode.AUTO")
                @Test
                void isCacheOutdatedCorrectForAuto() {                    
                    System.out.println("🚒");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    final LynxGetMotorEncoderPositionCommand command1 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command1, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command1
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Bulk cache has been obtained, but the next command is new, so no update
                    final LynxGetBulkInputDataCommand command2 = new LynxGetBulkInputDataCommand(control);
                    expectingUpdateAll = false;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command2, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command2
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double thirdPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Cleaing and seeing that an update comes through
                    final LynxGetMotorEncoderPositionCommand command3 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command3, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command3
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double qincePosition = motorCont.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(secondPosition, qincePosition);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Command not new this time, so no new update
                    final LynxGetBulkInputDataCommand command4 = new LynxGetBulkInputDataCommand(control);
                    expectingUpdateAll = false;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command4, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command4
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double quadPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), quadPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(qincePosition, quadPosition); // NOTE: Delay from getCurrentPosition is disabled
                }

                @DisplayName("BulkCachingMode.AUTO updates on duplicate request")
                @Test
                void autoUpdatesOnlyOnDuplicateRequest() {
                    System.out.println("🚒");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Bulk cache has been obtained, but the next command is new, so no update
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetBulkInputDataCommand(control)
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double thirdPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Cleaing and seeing that an update comes through
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double qincePosition = motorCont.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(secondPosition, qincePosition);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Command not new this time, so no new update
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetBulkInputDataCommand(control)
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double quadPosition = motorCont.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), quadPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(qincePosition, quadPosition); // NOTE: Delay from getCurrentPosition is disabled

                }

                @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
                @DisplayName("BulkCachingMode.OFF always updates")
                @Test
                void offUpdatesAlways() {
                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(originalPosition, motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                }
                
                @DisplayName("isCacheOutdated is correct for BulkCachingMode.OFF")
                @Test
                void isCacheOutdatedCorrectForOff() {
                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    final LynxGetMotorEncoderPositionCommand command1 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command1, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command1
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(originalPosition, motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                
                }

                
                @Timeout(value = 1, unit = SECONDS, threadMode = Timeout.ThreadMode.SEPARATE_THREAD)
                @DisplayName("BulkCachingMode.MANUAL Only updates on clear")
                @Test
                void manualUpdatesOnlyClear() {
                    System.out.println("🎈");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    System.out.println("[manual update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // We just got a bulk cache, so no new update will be gotten.
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    final double thirdPosition = motorCont.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(Math.round(originalPosition), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition);

                    // Asserting that when forced to update, the cache stilll wont change
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAll(1.0));
                    final double quadPos= motorCont.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(Math.round(originalPosition), quadPos);
                    assertNotEquals(Math.round(motorContData.position), quadPos);
                    assertEquals(thirdPosition, quadPos);

                    // Cleaing and seeing that an update comes through
                    control.clearBulkCache();
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double qincePosition = motorCont.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(thirdPosition, qincePosition);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                }
            
                
                @DisplayName("isCacheOutdated is correct for BulkCachingMode.MANUAL")
                @Test
                void isCacheOutdatedCorrectForManual() {
                    System.out.println("🎈");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    motorCont.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    final LynxGetMotorEncoderPositionCommand command1 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command1, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command1
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    System.out.println("[manual update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled

                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // We just got a bulk cache, so no new update will be gotten.
                    final LynxGetMotorEncoderPositionCommand command2 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = false;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command2, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command2
                    ));
                    final double thirdPosition = motorCont.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(Math.round(originalPosition), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition);

                    // Asserting that when forced to update, the cache stilll wont change
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> mockUpdater.updateAll(1.0));
                    final double quadPos= motorCont.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(Math.round(originalPosition), quadPos);
                    assertNotEquals(Math.round(motorContData.position), quadPos);
                    assertEquals(thirdPosition, quadPos);

                    // Cleaing and seeing that an update comes through
                    final LynxGetMotorEncoderPositionCommand command3 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    control.clearBulkCache();
                    expectingUpdateAll = true;
                    assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command3, ""));
                    assertDoesNotThrow(() -> mockUpdater.updateAllIfCacheOutdated(
                        1.0, 
                        motorCont,
                        command3
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double qincePosition = motorCont.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(thirdPosition, qincePosition);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                }
            }
        
            @DisplayName("MultiRegister")
            @Nested
            class MultiRegister {
                ModularUpdater updater2 = new ModularUpdater();
                ModularUpdater updater3 = new ModularUpdater();
                List<ModularUpdaterMock> ups = null;

                DcMotorImplExFake motorCont2 = new DcMotorImplExFake(new MotorData(100, 204), controlHubController);
                DcMotorImplExFake motorCont3 = new DcMotorImplExFake(new MotorData(100, 304), controlHubController);
                
                MotorData data2 = motorCont2.getData();
                MotorData data3 = motorCont3.getData();

                @BeforeEach
                void addHardwareToMockUpdaters() {
                    System.out.println("");
                    device.setMotors(new DcMotorImplExFake[]{ motorCont, motorCont2, motorCont3 });

                    mockUpdater = new ModularUpdaterMock(updater);
                    assertTrue(mockUpdater.hasRegistered(motorCont));

                    for(final ModularUpdater up : new ModularUpdater[]{ updater2, updater3 }) {
                        // assertTrue(up.register(servoCont, control));
                        // assertTrue(up.register(motorNone, null));
                        // assertTrue(up.register(motorExp, expansion));
                        assertTrue(up.register(motorCont, control));
                    }

                    assertTrue(updater2.register(motorCont2, control));
                    assertTrue(updater2.register(motorCont3, null));
                    assertTrue(updater3.register(motorCont3, control));

                    // Disabling automatic updates.
                    for(final DcMotorImplExFake motor : motors) {
                        motor.forget(updater);
                        motor.forget(updater2);
                        motor.forget(updater3);
                        motor.forget(mockUpdater);
                    }
                    motorCont2.forget(updater);
                    motorCont2.forget(updater2);
                    motorCont2.forget(updater3);
                    motorCont2.forget(mockUpdater);

                    motorCont3.forget(updater);
                    motorCont3.forget(updater2);
                    motorCont3.forget(updater3);
                    motorCont3.forget(mockUpdater);

                    // Creating the mocks for the new updaters
                    updater2 = new ModularUpdaterMock(updater2);
                    updater3 = new ModularUpdaterMock(updater3);
                    ups = Arrays.asList(new ModularUpdaterMock[]{ 
                        (ModularUpdaterMock) mockUpdater, 
                        (ModularUpdaterMock) updater2, 
                        (ModularUpdaterMock) updater3 
                    });

                    // Disabling even more automatic updates
                    for(final DcMotorImplExFake motor : motors) {
                        // motor.forget(updater);
                        motor.forget(updater2);
                        motor.forget(updater3);
                        // motor.forget(mockUpdater);
                    }
                    // motorCont2.forget(updater);
                    motorCont2.forget(updater2);
                    motorCont2.forget(updater3);
                    // motorCont2.forget(mockUpdater);

                    // motorCont3.forget(updater);
                    motorCont3.forget(updater2);
                    motorCont3.forget(updater3);
                    // motorCont3.forget(mockUpdater);

                    System.out.println("[add hardware to] mockUpdater: " + mockUpdater.toString());
                    System.out.println("[add hardware to] updater2: " + updater2.toString());
                    System.out.println("[add hardware to] updater3: " + updater3.toString());
                    
                    System.out.println("[add hardware to] motorData: " + motorContData.toString());
                    System.out.println("[add hardware to] motorData: " + data2.toString());
                    System.out.println("[add hardware to] motorData: " + data3.toString());

                }
            
                // TODO: If any other Updater classes are added, these 5 tests should be replicated
                @DisplayName("Updater.updateAllOnce traverses all given ModularUpdaters")
                @Test
                void traversesAllGivenUpdaters() {
                    // This will throw if any updater (mock) does not have updateAll() called
                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    assertDoesNotThrow(() -> Updater.updateAllOnce(ups, 1.0));
                    assertDoesNotThrow(() -> endAwait(ups));

                    // Making sure it does throw if an updater is forgotten
                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    assertDoesNotThrow(() -> Updater.updateAllOnce(List.of(updater2, updater3), 1.0));
                    assertThrows(Throwable.class, () -> endAwait(ups));
                }

                @DisplayName("Updater.updateAllOnce updates all registered updateables")
                @Test
                void updatesAllUpdateables() {
                    final double originalPos1 = motorContData.position;
                    final double originalPos2 = data2.position;
                    final double originalPos3 = data3.position;

                    expectingUpdateAll = false; // Ensuring no automatic update occurs
                    awaitUpdateAll(ups);
                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);
                    assertDoesNotThrow(() -> endAwait(ups), "no accidental automatic update");

                    assertNotWithin(0.0, motorContData.getActualVelocity(), 1e-10);
                    assertNotWithin(0.0, data2.getActualVelocity(), 1e-10);
                    assertNotWithin(0.0, data3.getActualVelocity(), 1e-10);

                    Updater.updateAllOnce(ups, 1.0);

                    assertNotWithin(originalPos1, motorContData.position, 1e-7);
                    assertNotWithin(originalPos2, data2.position, 1e-7);
                    assertNotWithin(originalPos3, data3.position, 1e-7);
                }

                @DisplayName("Updater.updateAllOnce updates reigstered updateables once")
                @Test
                void updatesOnlyOnce() {
                    final double originalPos1 = motorContData.position;
                    final double originalPos2 = data2.position;
                    final double originalPos3 = data3.position;

                    expectingUpdateAll = false; // Ensuring no automatic update occurs
                    awaitUpdateAll(ups);
                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);
                    assertDoesNotThrow(() -> endAwait(ups), "no accidental automatic update");

                    assertNotWithin(0.0, motorContData.getActualVelocity(), 1e-10);
                    assertNotWithin(0.0, data2.getActualVelocity(), 1e-10);
                    assertNotWithin(0.0, data3.getActualVelocity(), 1e-10);

                    Updater.updateAllOnce(ups, 1.0);

                    assertFloatEquals(motorContData.maxTicksPerSec + originalPos1, motorContData.position, 1e-7);
                    assertFloatEquals(data2.maxTicksPerSec + originalPos2, data2.position, 1e-7);
                    assertFloatEquals(data3.maxTicksPerSec + originalPos3, data3.position, 1e-7);
                }

                @DisplayName("Updater.updateAllOnce respects the wishes of setUpdatingEnabled")
                @ParameterizedTest
                @MethodSource("eightBooleans")
                void updateAllOnceRespectsSetUpdatingEnabled(boolean bool1, boolean bool2, boolean bool3) {
                    final double originalPosition = motorContData.position;
                    final double originalPosition2 = data2.position;
                    final double originalPosition3 = data3.position;

                    motorCont.setUpdatingEnabled(bool1);
                    motorCont2.setUpdatingEnabled(bool2);
                    motorCont3.setUpdatingEnabled(bool3);

                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);

                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    Updater.updateAllOnce(ups, 1.0);
                    endAwait(ups);

                    // assertEquals(motorContData.maxTicksPerSec + originalPosition, motorContData.position);
                    // assertEquals(data2.maxTicksPerSec + originalPosition2, data2.position);
                    // assertEquals(data3.maxTicksPerSec + originalPosition3, data3.position);

                    assertEquals(bool1, motorContData.position != originalPosition);
                    assertEquals(bool2, data2.position != originalPosition2);
                    assertEquals(bool3, data3.position != originalPosition3);
                }

                static Stream<Arguments> eightBooleans() {
                    final Stream.Builder<Arguments> builder = Stream.builder();
                    for(int x = 0; x < 2; x++) {
                        for(int y = 0; y < 2; y++) {
                            for(int z = 0; z < 2; z++) {
                                builder.add(Arguments.of(x != 0, y != 0, x != 0));
                            }
                        }
                    }
                    return builder.build();
                }

                @DisplayName("Updater.updateAllOnce respects perserves of setUpdatingEnabled")
                @ParameterizedTest
                @MethodSource("eightBooleans")
                void updateAllOncePerservesSetUpdatingEnabled(boolean bool1, boolean bool2, boolean bool3) {
                    final double originalPosition = motorContData.position;
                    final double originalPosition2 = data2.position;
                    final double originalPosition3 = data3.position;

                    motorCont.setUpdatingEnabled(bool1);
                    motorCont2.setUpdatingEnabled(bool2);
                    motorCont3.setUpdatingEnabled(bool3);

                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);

                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    Updater.updateAllOnce(ups, 1.0);
                    endAwait(ups);

                    // assertEquals(motorContData.maxTicksPerSec + originalPosition, motorContData.position);
                    // assertEquals(data2.maxTicksPerSec + originalPosition2, data2.position);
                    // assertEquals(data3.maxTicksPerSec + originalPosition3, data3.position);

                    assertEquals(bool1, motorCont.isUpdatingEnabled());
                    assertEquals(bool2, motorCont2.isUpdatingEnabled());
                    assertEquals(bool3, motorCont3.isUpdatingEnabled());
                }

                @DisplayName("ModularUpdater.updateAllOnceIf... updates always on OFF")
                @Test
                void updatesAlwaysOnOff() {
                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    final double originalPosition2 = data2.position;
                    final double originalPosition3 = data3.position;
                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    final LynxGetMotorEncoderPositionCommand command1 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    // assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command1, ""));
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        command1
                    ));
                    endAwait(ups);

                    assertNotEquals(originalPosition, motorContData.position);
                    assertNotEquals(originalPosition2, data2.position);
                    assertNotEquals(originalPosition3, data3.position);

                    assertNotEquals(originalPosition, motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(originalPosition2, motorCont2.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(originalPosition3, motorCont3.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled

                    assertEquals(Math.round(motorContData.position), motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), motorCont2.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), motorCont3.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                
                    final LynxGetMotorEncoderPositionCommand command2 = new LynxGetMotorEncoderPositionCommand(control, motorCont2.getPortNumber());
                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    // assertEquals(expectingUpdateAll, mockUpdater.isCacheOutdated(motorCont, command2, ""));
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont3,
                        command2
                    ));
                    endAwait(ups);

                    assertNotEquals(originalPosition, motorContData.position);
                    assertNotEquals(originalPosition2, data2.position);
                    assertNotEquals(originalPosition3, data3.position);

                    assertNotEquals(originalPosition, motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(originalPosition2, motorCont2.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(originalPosition3, motorCont3.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled

                    assertEquals(Math.round(motorContData.position), motorCont.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), motorCont2.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), motorCont3.getCurrentPosition()); // NOTE: Delay from getCurrentPosition is disabled
                }

                @DisplayName("ModularUdpater.updateAllOnceIf... updates when repeated on AUTO")
                @Test
                void updatesWhenRepeatedOnAuto() {
                    System.out.println("🚒");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    final double originalPosition2 = data2.position;
                    final double originalPosition3 = data3.position;
                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    final LynxGetMotorEncoderPositionCommand command1 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        command1
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    final double secondPosition = motorCont.getCurrentPosition();
                    final double secondPosition2 = motorCont2.getCurrentPosition();
                    final double secondPosition3 = motorCont3.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), secondPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), secondPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), secondPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), secondPosition3); // NOTE: Delay from getCurrentPosition is disabled

                    // Bulk cache has been obtained, but the next command is new, so no update
                    final LynxGetBulkInputDataCommand command2 = new LynxGetBulkInputDataCommand(control);
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        command2
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    final double thirdPosition = motorCont.getCurrentPosition();
                    final double thirdPosition2 = motorCont2.getCurrentPosition();
                    final double thirdPosition3 = motorCont3.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), secondPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), secondPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), thirdPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), thirdPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition2, thirdPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition3, thirdPosition3); // NOTE: Delay from getCurrentPosition is disabled

                    // Cleaing and seeing that an update comes through
                    final LynxGetMotorEncoderPositionCommand command3 = new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber());
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        command3
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    final double qincePosition = motorCont.getCurrentPosition();
                    final double qincePosition2 = motorCont2.getCurrentPosition();
                    final double qincePosition3 = motorCont3.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), qincePosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), qincePosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(secondPosition, qincePosition);
                    assertNotEquals(secondPosition2, qincePosition2);
                    assertNotEquals(secondPosition3, qincePosition3);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), qincePosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), qincePosition3); // NOTE: Delay from getCurrentPosition is disabled

                    // Command not new this time, so no new update
                    final LynxGetBulkInputDataCommand command4 = new LynxGetBulkInputDataCommand(control);
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        command4
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    final double quadPosition = motorCont.getCurrentPosition();
                    final double quadPosition2 = motorCont2.getCurrentPosition();
                    final double quadPosition3 = motorCont3.getCurrentPosition();
                    System.out.println("[auto update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), qincePosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), qincePosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), quadPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), quadPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), quadPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(qincePosition, quadPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(qincePosition2, quadPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(qincePosition3, quadPosition3); // NOTE: Delay from getCurrentPosition is disabled
                }

                @DisplayName("ModularUpdater.updateAllOnceIf... updates when clear on MANUAL")
                @Test
                void updatesWhenClearOnManual() {
                    System.out.println("🎈");
                    control.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

                    // Storing the original position and then updating it.
                    final double originalPosition = motorContData.position;
                    final double originalPosition2 = data2.position;
                    final double originalPosition3 = data3.position;

                    motorCont.setPower(1.0);
                    motorCont2.setPower(1.0);
                    motorCont3.setPower(1.0);
                    
                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // No data has been requested yet, so this will pass
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    final double secondPosition = motorCont.getCurrentPosition();
                    final double secondPosition2 = motorCont2.getCurrentPosition();
                    final double secondPosition3 = motorCont3.getCurrentPosition();
                    System.out.println("[manual update clear] actual pos: " + motorContData.position);
                    assertNotEquals(Math.round(originalPosition), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), secondPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), secondPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), secondPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), secondPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), secondPosition3); // NOTE: Delay from getCurrentPosition is disabled

                    // Checking beahvior. Unexpected behavior throws in updateAllIfCacheOutdated
                    // We just got a bulk cache, so no new update will be gotten.
                    expectingUpdateAll = false;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    final double thirdPosition = motorCont.getCurrentPosition();
                    final double thirdPosition2 = motorCont2.getCurrentPosition();
                    final double thirdPosition3 = motorCont3.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    assertNotEquals(Math.round(originalPosition), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), thirdPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), thirdPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(motorContData.position), thirdPosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), thirdPosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), thirdPosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(secondPosition, thirdPosition);
                    assertEquals(secondPosition2, thirdPosition2);
                    assertEquals(secondPosition3, thirdPosition3);

                    // Asserting that when forced to update, the cache stilll wont change
                    expectingUpdateAll = true;
                    awaitUpdateAll(ups);
                    assertDoesNotThrow(() -> Updater.updateAllOnce(ups, 1.0));
                    endAwait(ups);
                    final double quadPos= motorCont.getCurrentPosition();
                    final double quadPos2= motorCont2.getCurrentPosition();
                    final double quadPos3= motorCont3.getCurrentPosition();
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    assertNotEquals(Math.round(originalPosition), quadPos);
                    assertNotEquals(Math.round(originalPosition2), quadPos2);
                    assertNotEquals(Math.round(originalPosition3), quadPos3);
                    assertNotEquals(Math.round(motorContData.position), quadPos);
                    assertNotEquals(Math.round(data2.position), quadPos2);
                    assertNotEquals(Math.round(data3.position), quadPos3);
                    assertEquals(thirdPosition, quadPos);
                    assertEquals(thirdPosition2, quadPos2);
                    assertEquals(thirdPosition3, quadPos3);

                    // Cleaing and seeing that an update comes through
                    control.clearBulkCache();
                    expectingUpdateAll = true;
                    assertDoesNotThrow(() -> ModularUpdater.updateAllOnceIfAnyCacheOutdated(
                        ups,
                        1.0, 
                        motorCont,
                        new LynxGetMotorEncoderPositionCommand(control, motorCont.getPortNumber())
                    ));
                    assertNotEquals(motorContData.position, originalPosition);
                    assertNotEquals(data2.position, originalPosition2);
                    assertNotEquals(data3.position, originalPosition3);
                    final double qincePosition = motorCont.getCurrentPosition();
                    final double qincePosition2 = motorCont2.getCurrentPosition();
                    final double qincePosition3 = motorCont3.getCurrentPosition();
                    assertNotEquals(Math.round(originalPosition), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition2), qincePosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(Math.round(originalPosition3), qincePosition3); // NOTE: Delay from getCurrentPosition is disabled
                    assertNotEquals(thirdPosition, qincePosition);
                    assertNotEquals(thirdPosition2, qincePosition2);
                    assertNotEquals(thirdPosition3, qincePosition3);
                    assertEquals(Math.round(motorContData.position), qincePosition); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data2.position), qincePosition2); // NOTE: Delay from getCurrentPosition is disabled
                    assertEquals(Math.round(data3.position), qincePosition3); // NOTE: Delay from getCurrentPosition is disabled
                }
            }
        }
    }
}