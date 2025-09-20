package clarson.ftc.faker;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionResponse;
import com.qualcomm.hardware.lynx.commands.standard.LynxKeepAliveCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxStandardCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxQueryInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxQueryInterfaceResponse;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.hardware.LynxModuleMetaList;
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.hardware.DeviceManager;

import java.util.concurrent.TimeoutException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList;
import org.firstinspires.ftc.robotcore.internal.ui.ProgressParameters;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;
import org.firstinspires.ftc.robotcore.internal.hardware.TimeWindow;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class LynxUsbDeviceImplFake extends LynxUsbDeviceImpl {
    public static final String UNRESPONDABLE_MSG = "LynxMessage %s could not receive its response.";
    public static final String UN_NACKABLE_MSG = "LynxMessage %s could not receive its nack.";
    public static final String UN_ACKABLE_MSG = "LynxMessage %s could not receive its ack(nowledgement).";

    private static class RobotUsbDeviceFake implements RobotUsbDevice {
        // ----------------------------------------
        //     Important Methods! Are called!
        // ----------------------------------------

        private SerialNumber serial;
        
        @Override 
        public USBIdentifiers getUsbIdentifiers() {
            return USBIdentifiers.createLynxIdentifiers();
        }
        
        @Override 
        @NonNull 
        public DeviceManager.UsbDeviceType getDeviceType() {
            return DeviceManager.UsbDeviceType.LYNX_USB_DEVICE;
        }
        
        @Override 
        @NonNull 
        public SerialNumber getSerialNumber() {
            return serial;
        }

        
        // ----------------------------------------
        //          Do-Nothing Methods 
        // ----------------------------------------
        @Override public void setDebugRetainBuffers(boolean retain) {}
        @Override public void logRetainedBuffers(long nsOrigin, long nsTimerExpire, String tag, String format, Object...args) {}
        @Override public void setBaudRate(int rate) throws RobotUsbException {}
        @Override public void setDataCharacteristics(byte dataBits, byte stopBits, byte parity) throws RobotUsbException {}
        @Override public void setLatencyTimer(int latencyTimer) throws RobotUsbException {}
        @Override public void setBreak(boolean enable) throws RobotUsbException {}
        @Override public void resetAndFlushBuffers() throws RobotUsbException {}
        @Override public void write(byte[] data) throws InterruptedException, RobotUsbException {}
        @Override public void skipToLikelyUsbPacketStart() {}
        @Override public void requestReadInterrupt(boolean interruptRequested) {}
        @Override public void close() {}
        @Override public void setFirmwareVersion(FirmwareVersion version) {}
        @Override public void setDeviceType(@NonNull DeviceManager.UsbDeviceType deviceType) {}
        @Override public boolean mightBeAtUsbPacketStart() {
            return false;
        }
        @Override public int read(byte[] data, int ibFirst, int cbToRead, long msTimeout, @Nullable TimeWindow timeWindow) throws RobotUsbException, InterruptedException {
            return 0;
        }
        @Override public boolean isOpen() {
            return true;
        }
        @Override public boolean isAttached() {
            return true;
        }
        @Override public FirmwareVersion getFirmwareVersion() {
            return null;
        }
        @Override @NonNull public String getProductName() {
            return "Do-Nothing RobotUsbDevice";
        }
        @Override public boolean getDebugRetainBuffers() {
            return false;
        }
    }

    private static class RobotUsbManagerFake implements RobotUsbManager {
        // Looking at the source of LynxUsbDeviceImpl and ArmableUsbDevice, this 
        // RobotUsbDevice is only used to reset the lynx board, and read 
        // and write to peripherals. Methods that get device info such as name or
        // conenctions seem to be used, but only when LynxUsbDevice device info is
        // requested, which a user most likely isn't doing.
        //
        // Reading and writing can be simulated in the `LynxUsbDevice.transmit()` 
        // method, so no worrys there.
        public RobotUsbDeviceFake fake = new RobotUsbDeviceFake();

        public RobotUsbManagerFake(SerialNumber serial) {
            this.fake.serial = serial;
        }

        @Override
        public List<SerialNumber> scanForDevices() {
            // TODO: If webcams or LimeLight is added, add those to here
            return Arrays.asList(fake.getSerialNumber());
        }

        @Override
        public RobotUsbDevice openBySerialNumber(SerialNumber serial) {
            if(serial.equals(fake.getSerialNumber())) {
                return fake;
            }

            return null;
        }
    }
    
    private static final SerialNumber createFakeSerial() {
        return lastFakeSerial = SerialNumber.createEmbedded();
    }

    private static void throwIfWrongSize(Object[] hardware, int maxAllowableSize, String hardwareName) {
        if(hardware.length > maxAllowableSize) {
            throw new IllegalArgumentException(String.format(
                "Number of %ss (%d) greater than max (%d)",
                hardwareName,
                hardware.length,
                maxAllowableSize 
            ));
        }
    }

    private static byte getByte(int num, int litteEndianIndex) {
        final int byteIndex = 8 * litteEndianIndex;
        return (byte) ((num & (0xff << byteIndex)) >>> byteIndex);
    }

    private static RobotUsbManagerFake usbManagerFake;
    private static SerialNumber lastFakeSerial;
    private DcMotorImplExFake[] motors = null;
    // private AnalogInput[] analogInputs = null;
    // private DigitalChannel[] digitalChannels = null;

    public LynxUsbDeviceImplFake() {        
        super(null, createFakeSerial(), null, usbManagerFake = new RobotUsbManagerFake(lastFakeSerial));
        // All hardware lists are left null.

        // System.out.println("[impl fake init] serial: " + lastFakeSerial);
        // System.out.println("[impl fake init] subUsb serial: " + usbManagerFake.fake.serial);
        // System.out.println("[impl fake init] usbDevice: " + usbManagerFake.openBySerialNumber(lastFakeSerial));
    }  


    // TODO: Create the analog and digital sensors
    // * @param analogs Data for all analog inputs. Length must be at or below 4
    // * @param digitals Data for all digital inputs. Length must be at or below 8
    /**
     * Creates the payload for the bulk input response. The bytes are stored in 
     * little endian format. 
     * 
     * If any of the given hardware iterables exceed its expected length, the 
     * method throws an IllegalArgumentException
     * 
     * @param motors Data for all connected motors. Length must be at or below 4
     * @return The payload. Contains all data for `LynxGetBulkInputDataResponse`
     */
    protected byte[] readBulkDataPayload(
        DcMotorImplExFake[] motors/* , 
        DigitalChannel[] digitals,
        AnalogInput[] analogs
        */
    ) {
        /*
        struct Payload {
            [00]  uint8_t  digitalInputs;      // DIGITAL_START

            [01]  int32_t  motor0position_enc; // ENCODER_START
            [05]  int32_t  motor1position_enc;
            [09]  int32_t  motor2position_enc;
            [13]  int32_t  motor3position_enc;

            [17]  uint8_t  motorStatus;        // STATUS_START

            [18]  int16_t  motor0velocity_cps; // VELOCITY_START
            [20]  int16_t  motor1velocity_cps;
            [22]  int16_t  motor2velocity_cps;
            [24]  int16_t  motor3velocity_cps;

            [26]  int16_t  analog0_mV;         // ANALOG_START
            [28]  int16_t  analog1_mV;
            [30]  int16_t  analog2_mV;
            [32]  int16_t  analog3_mV;
        }
        */
        // Initializing the payload.
        final int DIGITAL_START  = 0;
        final int ENCODER_START  = DIGITAL_START  +  1;
        final int STATUS_START   = ENCODER_START  +  LynxConstants.NUMBER_OF_MOTORS * 4;
        final int VELOCITY_START = STATUS_START   +  1;
        final int ANALOG_START   = VELOCITY_START +  LynxConstants.NUMBER_OF_MOTORS * 2;
        final int LENGTH         = ANALOG_START   +  LynxConstants.NUMBER_OF_ANALOG_INPUTS * 2;
        final byte[] payload = new byte[LENGTH];
        
        // Validating and reading the motors
        throwIfWrongSize(motors, LynxConstants.NUMBER_OF_MOTORS, "motor");
        for(int i = 0; i < motors.length; i++) {
            // Serializing the encoder position as a little endian int.
            final DcMotorImplExFake motor = motors[i];
            ((DcMotorControllerExFake) motor.getController()).setForceReread(true);  // Preventing infinite recursion

            final int encoderPosition = motor.getCurrentPosition();
            payload[ENCODER_START + 4 * i]     = getByte(encoderPosition, 0);
            payload[ENCODER_START + 4 * i + 1] = getByte(encoderPosition, 1);
            payload[ENCODER_START + 4 * i + 2] = getByte(encoderPosition, 2);
            payload[ENCODER_START + 4 * i + 3] = getByte(encoderPosition, 3);

            // Serializing the velocity as a little endian short.
            final int velocity = (int) Math.round(motor.getVelocity());
            payload[VELOCITY_START + 2 * i]     = getByte(velocity, 0);
            payload[VELOCITY_START + 2 * i + 1] = getByte(velocity, 1);

            // Serializing the status of the motor. Motor status is stored in one 
            // byte, with the lower half for isAtTarget and the upper half for 
            // isOverCurrent. E.g., if our status was 0b1001_1100, then isAtTarget
            // would take 0b1001, and isOverCurrent would take 0b1100
            final int isOverCurrentBit = (motor.isOverCurrent() ? 1 : 0) << i;
            final int isAtTargetBit = (!motor.isBusy() ? 1 : 0) << (i + 4);
            payload[STATUS_START] = (byte) (payload[STATUS_START] | isOverCurrentBit | isAtTargetBit);

            ((DcMotorControllerExFake) motor.getController()).setForceReread(false);
        }

        //#region DEV START: Logging the payload
        for(int i = 0; i < payload.length; i++) {
            System.out.println("    [read bulk payload] byte: " + payload[i]);
        }
        
        //#endregion DEV END

        // Validating and reading the analog input        
        // throwIfWrongSize(analogs, LynxConstants.NUMBER_OF_ANALOG_INPUTS, "analog input");

        // Validating and reading the digital IO
        // Note that all 
        // throwIfWrongSize(digitals, LynxConstants.NUMBER_OF_DIGITAL_IOS, "digital input/output");

        return payload;
    }

    protected LynxGetBulkInputDataResponse createFromPayload(LynxModuleIntf module, byte[] buffer) {
        final LynxGetBulkInputDataResponse response = new LynxGetBulkInputDataResponse(module);
        response.fromPayloadByteArray(buffer);
        return response;
    }
    
    private LynxGetBulkInputDataResponse handleBulkDataCommand(LynxModuleIntf module) {
        return createFromPayload(module, readBulkDataPayload(
            motors/*,   
            getDigitalChannels(),
            getAnalogInputs() */
        ));
    }

    private LynxQueryInterfaceResponse handleQueryInterfaceCommand(LynxModuleIntf module) {
        final LynxQueryInterfaceResponse result = new LynxQueryInterfaceResponse((LynxModule) module);
        final short commandNumberFirst = LynxStandardCommand.COMMAND_NUMBER_FIRST;
        final short numberOfCommands = LynxStandardCommand.COMMAND_NUMBER_LAST - LynxStandardCommand.COMMAND_NUMBER_FIRST + 1;
        result.fromPayloadByteArray(new byte[] { 
            getByte(commandNumberFirst, 0),
            getByte(commandNumberFirst, 1),
            getByte(numberOfCommands, 0),
            getByte(numberOfCommands, 1),
        });
        return result;
    }

    /**
     * Takes the given command, and produces a response based off of it. Only 
     * certain commands are supported, which are listed below:
     * 
     *  - LynxGetBulkInputDataCommand: Get bulk data from the LynxModuleFake
     * 
     * If a command is not supported, a LynxNack (reponse error message) is 
     * created, and is passed as the response. The specific code is 
     * StandardReasonCode 253, `COMMAND_IMPL_PENDING`.
     * 
     * @param message What was sent to the LynxModule to be done. 
     */
    @Override
    public void transmit(LynxMessage message) {
        LynxMessage response = null;
        LynxAck ack = null;

        // If the command is supported, handle th command and its resopnse (if necessary).
        // This is hacky and I don't like instaceof chains, but its the best we can do.
        // NOTE: If you want to fix this, I'm open for a PR!
        if(message instanceof LynxGetBulkInputDataCommand) {
            response = handleBulkDataCommand(message.getModule());
        }

        if(message instanceof LynxGetMotorEncoderPositionCommand) {
            response = new LynxGetMotorEncoderPositionResponse(message.getModule());
            final DcMotorImplExFake motor = motors[message.toPayloadByteArray()[0]];
            final int position = motor.getCurrentPosition();
            response.fromPayloadByteArray(new byte[] { 
                 getByte(position, 0),
                 getByte(position, 1),
                 getByte(position, 2),
                 getByte(position, 3)
            });
        }
        

        if(message instanceof LynxIsMotorAtTargetCommand) {
            response = new LynxIsMotorAtTargetResponse(message.getModule());
            final DcMotorImplExFake motor = motors[message.toPayloadByteArray()[0]];
            response.fromPayloadByteArray(new byte[]{ motor.isBusy() ? (byte) 0x00 : (byte) 0x01 });
        }

        if(message instanceof LynxQueryInterfaceCommand) {
            response = handleQueryInterfaceCommand(message.getModule());
        }

        if(message instanceof LynxKeepAliveCommand) {
            ack = new LynxAck(message.getModule(), false);
        }

        // TODO: Implement LED command functionality

        // Handling the response if a response was received. We first verify ability to respond
        if(response != null && !(message.isAckable() && message.isResponseExpected())) {
            throw new UnsupportedLynxUsbCommandException(String.format(
                Locale.US, 
                UNRESPONDABLE_MSG, 
                message.toString()
            ));
        }

        if(response != null) {
            // Ending the function
            ((LynxRespondable) message).onResponseReceived(response);
            return;
        }

        // Handling an acknowledgement (informally known as an "ack")
        if(ack != null && !message.isAckable()) {
            throw new UnsupportedLynxUsbCommandException(String.format(
                Locale.US,
                UN_ACKABLE_MSG, // Not to be confused with UNNACKABLE  
                message.toString()
            ));
        }

        if(ack != null) {
            ((LynxRespondable) message).onAckReceived(ack);
            return;
        }

        // Sending the error response. We first must verify that we can, however
        if(!message.isAckable()) {
            throw new UnsupportedLynxUsbCommandException(String.format(
                Locale.US, 
                UN_NACKABLE_MSG, // Not to be confused with UN_ACKABLE_MESSAGE 
                message.toString()
            ));
        }

        final LynxNack nackResponse = new LynxNack(
            message.getModule(), 
            LynxNack.StandardReasonCode.COMMAND_ROUTING_ERROR
        );
        ((LynxRespondable) message).onNackReceived(nackResponse);
        return;
    }

    public LynxUsbDeviceImplFake setMotors(DcMotorImplExFake[] newMotors) {
        this.motors = newMotors;
        return this;
    }

    /* public LynxUsbDeviceImplFake setDigitalsChannels(DigitalChannel[] newInputs) {
        this.digitalsChannels = newInputs;
        return this;
    }

    public LynxUsbDeviceImplFake setAnalogInputs(AnalogInput[] newInputs) {
        this.analogInputs = newInputs;
        return this;
    }*/
 
    @Override
    public void acquireNetworkTransmissionLock(LynxMessage message) {
        // Do nothing, as you hopefully shouldn't be using concurrency nor the network.
    }
 
    /*
     * This method "armDevice(RobotUsbDevice device)", modified from the 
     * original source form, is subject to the following copyright:
     *
     * Copyright (c) 2016 Robert Atkinson
     *
     * All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of Robert Atkinson nor the names of his contributors may be used to
     * endorse or promote products derived from this software without specific prior
     * written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
     * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
     * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */
    @Override
    protected void armDevice(RobotUsbDevice device) throws RobotCoreException, InterruptedException {
        synchronized (armingLock) {
            // RobotLog.vv(TAG, "armDevice() serial=%s...", serialNumber);
            // Assert.assertTrue(device != null);
            this.robotUsbDevice = device;

            // Issue a hardware reset to the device. This is just a good housekeeping
            // practice, as it guarantees that the device is in a pristine, known state.
            // This might help, for example, to recover from synchronization errors.
            //
            // It also will kick the device out of firmware update mode if it happens to be
            // in same. That all said, resetting will obliterate any in-memory state
            // maintained by the lynx firmware, some of which, like motor modes etc, are
            // hard to recreate. If we could come up with a simple enough (re)initialization
            // strategy in which user code could participate, we might reasonably *always*
            // reset, but for now we make sure we do it at least once but not thereafter.
            //
            if (!resetAttempted) {
                resetAttempted = true;
                // resetDevice(this.robotUsbDevice);
            }

            this.hasShutdownAbnormally = false;
            if (syncdDeviceManager != null) {
                syncdDeviceManager.registerSyncdDevice(this);
            }

            resetNetworkTransmissionLock();
            // startPollingForIncomingDatagrams();
            // System.out.println("[arm impl fake] known modules: " + this.getKnownModules());
            pingAndQueryKnownInterfaces();
            // startRegularPinging();
            // RobotLog.vv(TAG, "...done armDevice()");
        }
    }

    /*
     * This method "getOrAddModule(RobotUsbDevice device)", modified from the 
     * original source form, is subject to the following copyright:
     *
     * Copyright (c) 2016 Robert Atkinson
     *
     * All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of Robert Atkinson nor the names of his contributors may be used to
     * endorse or promote products derived from this software without specific prior
     * written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
     * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
     * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */
    @Override
    public LynxModule getOrAddModule(LynxModuleDescription moduleDescription)
            throws InterruptedException, RobotCoreException {
        // Avoid potential race condition where
        // performSystemOperationOnConnectedModule() closes a module that this
        // method just promoted to be a user module.
        synchronized (sysOpStartStopLock) {
            int moduleAddress = moduleDescription.address;
            // RobotLog.vv(TAG, "addConfiguredModule() module#=%d", moduleAddress);
            // boolean added = false;
            LynxModule module;

            synchronized (this.knownModules) {
                if (!this.knownModules.containsKey(moduleAddress)) {
                    module = new LynxModuleHardwareFake(
                        this, 
                        moduleAddress, 
                        moduleDescription.isParent,
                        moduleDescription.isUserModule
                    );

                    if (moduleDescription.isSystemSynthetic) {
                        module.setSystemSynthetic(true);
                    }
                    
                    this.knownModules.put(moduleAddress, module);
                    // added = true;
                } else {
                    module = knownModules.get(moduleAddress);
                    // RobotLog.vv(TAG, "addConfiguredModule() module#=%d: already exists",
                    // moduleAddress);

                    // noinspection ConstantConditions
                    if (moduleDescription.isUserModule && !module.isUserModule()) {
                        // The caller of this method is trying to set up a user module, but the
                        // currently-registered module is a non-user module, so we convert the
                        // registered module into a user module.
                        // RobotLog.vv(TAG, "Converting module #%d to a user module",
                        // module.getModuleAddress());
                        module.setUserModule(true);
                    }

                    // noinspection ConstantConditions
                    if (
                        moduleDescription.isUserModule 
                        && moduleDescription.isSystemSynthetic
                        && !module.isSystemSynthetic()
                    ) {
                        // The caller of this method is trying to set up a user module that was added
                        // implicitly, rather than explicitly stated in the XML configuration. Add that
                        // property to the registered module.
                        module.setSystemSynthetic(true);
                    }

                    // noinspection ConstantConditions
                    if (moduleDescription.isParent != module.isParent()) {
                        // RobotLog.ww(TAG, "addConfiguredModule(): The active configuration file may be
                        // incorrect about whether Expansion Hub %d is the parent",
                        // module.getModuleAddress());
                    }
                }
            }

            return module;
            }
        }
}