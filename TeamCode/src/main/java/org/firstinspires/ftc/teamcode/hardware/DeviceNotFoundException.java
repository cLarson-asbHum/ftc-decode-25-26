package org.firstinspires.ftc.teamcode.hardware;

public class DeviceNotFoundException extends RuntimeException {
    public DeviceNotFoundException(String message) {
        super(message);
    }

    public DeviceNotFoundException(Throwable cause) {
        super(cause);
    }
    
    public DeviceNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}