package org.firstinspires.ftc.teamcode.test.teleop;

import clarson.ftc.faker.CRServoImplExFake;
import clarson.ftc.faker.DcMotorImplExFake;
import clarson.ftc.faker.LynxModuleHardwareFake;
import clarson.ftc.faker.LynxUsbDeviceImplFake;
import clarson.ftc.faker.ServoImplExFake;
import clarson.ftc.faker.updater.ModularUpdater;
import clarson.ftc.faker.updater.Updateable;
// import clarson.ftc.faker.ColorRangeSensorFake;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.teleop.CompetitionTeleop;
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

import static org.firstinspires.ftc.teamcode.util.Util.*;
import static org.firstinspires.ftc.teamcode.test.TestUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import java.io.InputStream;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map;

@Disabled("This always fails because ColorRangeSensorFake has not been created")
@DisplayName("CompetitionTeleop")
class CompetitionTeleopUnitTest {
    public class NullTelemetry implements Telemetry {
        private final PrintStream out;

        public NullTelemetry(PrintStream out) {
            this.out = out;
        }
        @Override public Telemetry.Item addData(String caption, String format, Object[] args) {return null;}
        @Override public Telemetry.Item addData(String caption, Object value) {return null;}
        @Override public <T> Telemetry.Item addData(String caption, Func<T> valueProducer) {return null;}
        @Override public <T> Telemetry.Item addData(String caption, String format, Func<T> valueProducer) {return null;}
        @Override public boolean removeItem(Telemetry.Item item) { return false; }
        @Override public void clear() {}
        @Override public void clearAll() {}
        @Override public Object addAction(Runnable action) { return null; }
        @Override public boolean removeAction(Object token) { return false; }
        @Override public void speak(String text) {}
        @Override public void speak(String text, String languageCode, String countryCode) {}
        @Override public boolean update() { return false; }
        @Override public Telemetry.Line addLine() { return null; }
        @Override public Telemetry.Line addLine(String lineCaption) { return null; }
        @Override public boolean removeLine(Telemetry.Line line) { return false; }
        @Override public boolean isAutoClear() { return false; }
        @Override public void setAutoClear(boolean autoClear) {}
        @Override public int getMsTransmissionInterval() { return -1; }
        @Override public void setMsTransmissionInterval(int msTransmissionInterval) {}
        @Override public String getItemSeparator() { return null; }
        @Override public void setItemSeparator(String itemSeparator) {}
        @Override public String getCaptionValueSeparator() { return null; }
        @Override public void setCaptionValueSeparator(String captionValueSeparator) {}
        @Override public void setDisplayFormat(Telemetry.DisplayFormat displayFormat) {}
        @Override public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {}
        @Override public Telemetry.Log log() { return null; }
    }

    @DisplayName("Can Initialize")
    @Test
    void canInitialize() {
        final CompetitionTeleop opmode = new CompetitionTeleop();

        // Creating a hardwareMap, gamepads, and a telemetry
        final Gamepad gamepad1 = new Gamepad();
        final Gamepad gamepad2 = new Gamepad();
        final HardwareMap hardwareMap = new HardwareMap(null, null);
        final Telemetry telemetry = new NullTelemetry(System.out);

        // Setting the relevant fields on opmode
        assertDoesNotThrow(() -> opmode.gamepad1 = gamepad1);
        assertDoesNotThrow(() -> opmode.gamepad2 = gamepad2);
        assertDoesNotThrow(() -> opmode.hardwareMap = hardwareMap);
        assertDoesNotThrow(() -> opmode.telemetry = telemetry);
    }

    @DisplayName("PostInitialize")
    @Nested
    class PostInitialize {
        final CompetitionTeleop opmode = new CompetitionTeleop();
        final Gamepad gamepad1 = new Gamepad();
        final Gamepad gamepad2 = new Gamepad();
        final HardwareMap hardwareMap = new HardwareMap(null, null) {
            @Override
            public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
                synchronized (lock) {
                    deviceName = deviceName.trim();
                    java.util.List<HardwareDevice> list = allDevicesMap.get(deviceName);
                    T result = null;

                    if (list != null) {
                        for (HardwareDevice device : list) {
                            if (classOrInterface.isInstance(device)) {
                                // As stated in intializeDeviceIfNecessary's original source, it is currenlty only
                                // used for I2C devices, and we really don't care for hardware fakes anyways.
                                // super.initializeDeviceIfNecessary(device);
                                result = classOrInterface.cast(device);
                                break;
                            }
                        }
                    }

                    return result;
                }
            }
        };
        final Telemetry telemetry = new NullTelemetry(System.out);

        final LynxUsbDeviceImplFake usbDevice = new LynxUsbDeviceImplFake();
        LynxModuleHardwareFake controlHub;
        LynxModuleHardwareFake expansionHub;
        final ModularUpdater updater = new ModularUpdater();

        final HashMap<String, Updateable> expansionConfig = new HashMap<>() {{
            put("frontLeft",  new DcMotorImplExFake(312, 576.6));
            put("backLeft",   new DcMotorImplExFake(312, 576.6));
            put("frontRight", new DcMotorImplExFake(-312, 576.6));
            put("backRight",  new DcMotorImplExFake(-312, 576.6));
            put("leftShooter",  new DcMotorImplExFake(5400, 28)); // Theoretically 6000 rpm, but practically 5400
            put("rightShooter", new DcMotorImplExFake(-5400, 28)); // Theoretically 6000 rpm, but practically 5400
            put("intake", new DcMotorImplExFake(312, 576.6));

            // TODO: Make ramp pivot have correct linear speed and range
            final double MM_PER_SEC = 1; 
            final double MAX_MM = 10;
            put("rampPivot", new ServoImplExFake(MM_PER_SEC, MAX_MM)); // Linear servo. Rather than revs / sec, treat this as mm / sec
            put("leftFeeder",  new CRServoImplExFake(180)); // Super speed
            put("rightFeeder", new CRServoImplExFake(180)); // Super speed
            put("leftBlocker",  new ServoImplExFake(50, 5)); // TORQUE, 5 turn
            put("rightBlocker", new ServoImplExFake(50, 5)); // TORQUE, 5 turn
            put("duckSpinner", new CRServoImplExFake(180)); // Super speed
        }};

        final HashMap<String, Updateable> controlConfig = new HashMap<>() {{}};

        @BeforeEach
        void init() throws InterruptedException, RobotCoreException {
            // Creating the lynx modules
            controlHub = (LynxModuleHardwareFake) usbDevice.getOrAddModule(
                new LynxModuleDescription.Builder(-1, true)
                    .setUserModule()
                    .build()
            );
            expansionHub = (LynxModuleHardwareFake) usbDevice.getOrAddModule(
                new LynxModuleDescription.Builder(-2, false)
                    .setUserModule()
                    .build()
            );

            // Adding the hardware to the hardware map and the updater
            // The updater allows there to a global time for hardware.
            // As a result, delay from expensive methods (e.g. setPower, getDistance) is simulated.
            hardwareMap.put("Control Hub", controlHub);
            for(final Map.Entry<String, Updateable> e : controlConfig.entrySet()) {
                updater.register(e.getValue(), controlHub);
                hardwareMap.put(e.getKey(), (HardwareDevice) e.getValue());
            }

            hardwareMap.put("Expansion Hub 2", expansionHub);
            for(final Map.Entry<String, Updateable> e : expansionConfig.entrySet()) {
                updater.register(e.getValue(), controlHub);
                hardwareMap.put(e.getKey(), (HardwareDevice) e.getValue());
            }

            // Setting the relevant fields on opmode
            opmode.gamepad1 = gamepad1;
            opmode.gamepad2 = gamepad2;
            opmode.hardwareMap = hardwareMap;
            opmode.telemetry = telemetry;
        }
    
        @DisplayName("Can Init OpMode")
        @Test
        void canInitOpMode() {
            opmode.init();
        }
    }
}