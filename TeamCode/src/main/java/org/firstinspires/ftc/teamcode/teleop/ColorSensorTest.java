package org.firstinspires.ftc.teamcode.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;

@Configurable
@TeleOp(group="B - Testing")
public class ColorSensorTest extends LinearOpMode {

    public static int TRANSMISSION_MS_INTERVAL = 33;
    public static int SLEEP_TIME = 33;
    public static double GAIN = 200.0;
    public static boolean USE_NORM = false;
    public static boolean USE_BITMAP = false;


    @Override
    public void runOpMode() {
        final ColorRangeSensor rightReloadSensor = hardwareMap.tryGet(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = hardwareMap.tryGet(ColorRangeSensor.class, "leftReload");

        final LED rightRed = hardwareMap.tryGet(LED.class, "rightRed");
        final LED rightGreen = hardwareMap.tryGet(LED.class, "rightGreen");
        
        final LED leftRed = hardwareMap.tryGet(LED.class, "leftRed");
        final LED leftGreen = hardwareMap.tryGet(LED.class, "leftGreen");

        // Getting a non-null color sensor
        if(rightReloadSensor == null && leftReloadSensor == null) {
            // Both are null; throw an exception
            throw new RuntimeException("No hardware of type ColorRangeSensor with any of the following names was found: \n    \"rightReload\", \n    \"leftReload\"");
        }

        telemetry.setMsTransmissionInterval(40);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // final ColorRangeSensor sensor = rightReloadSensor == null ? leftReloadSensor : rightReloadSensor;
        final String rightColorSensorName = "Right";
        final ArtifactColorRangeSensor rightWrapper = new ArtifactColorRangeSensor(
            rightReloadSensor, 
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst()
            // new ArtifactColorRangeSensor.ColorSensorConst()
        );

        // final ColorRangeSensor leftSensor = rightReloadSensor == null ? leftReloadSensor : rightReloadSensor;
        final String leftColorSensorName = "Left";
        final ArtifactColorRangeSensor leftWrapper = new ArtifactColorRangeSensor(leftReloadSensor);

        ColorRangeSensor sensor = rightReloadSensor == null ? leftReloadSensor : rightReloadSensor;
        ArtifactColorRangeSensor wrapper = rightReloadSensor == null ? leftWrapper : rightWrapper;
        String sensorName = rightReloadSensor == null ? leftColorSensorName : rightColorSensorName;

        // Getting the color
        boolean isPaused = false;
        // boolean isLedOn = true;
        String previousLog = "";
        // boolean aButton = false;
        // boolean bButton = false;

        opModeLoop:
        while(opModeIsActive() || opModeInInit()) {
            if(gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                isPaused = !isPaused;
            }

            // Swapping the color sensor upon the x button 
            if(gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                if(sensor == rightReloadSensor && leftReloadSensor != null) {
                    sensorName = leftColorSensorName;
                    sensor = leftReloadSensor;
                    wrapper = leftWrapper;
                } else if(sensor == leftReloadSensor && rightReloadSensor != null) {
                    sensorName = rightColorSensorName;
                    sensor = rightReloadSensor;
                    wrapper = rightWrapper;
                }   
            }

            // if(gamepad1.bWasPressed() || gamepad2.bWasPressed) {}

            if(isPaused) {
                try {
                    Thread.sleep(SLEEP_TIME);
                } catch(InterruptedException err) {
                    // Nothing important to do; this is normal and expected
                }
                continue opModeLoop;
            }

            telemetry.setMsTransmissionInterval(TRANSMISSION_MS_INTERVAL);

            // Getting the data
            final double dist = sensor.getDistance(DistanceUnit.CM);
            int argb = 0;
            int red = sensor.red();
            int green = sensor.green();
            int blue = sensor.blue();

            sensor.setGain((float) GAIN);
            if(USE_NORM) {
                red   = (int) (255 * sensor.getNormalizedColors().red);
                green = (int) (255 * sensor.getNormalizedColors().green);
                blue  = (int) (255 * sensor.getNormalizedColors().blue);
            }

            argb = sensor.argb();
            if(USE_BITMAP) {
                red   = (argb >> 16) & 0xff;
                green = (argb >>  8) & 0xff;
                blue  = (argb      ) & 0xff;
            }

            final double hue = JavaUtil.rgbToHue(red, green, blue);
            final double saturation = JavaUtil.rgbToSaturation(red, green, blue);
            final double value = JavaUtil.rgbToValue(red, green, blue);
            
            final ArtifactColor artifactcolor = wrapper.getColor();

            // Formatting the data
            final String format = 
                "<p>Distance (cm): %.2f</p>" + 
                "<br/>" + 

                "<h2>ArtifactColor (%s)</h2>" +
                "<p style=\"font-family:monospace;\">ArtifactColor: %s</p>" + 
                "<br/>" + 

                "<h2>RGB Color</h2>" + 
                "<p style=\"font-family:monospace;\">Red: %d</p>" +
                "<p style=\"font-family:monospace;\">Green: %d</p>" +
                "<p style=\"font-family:monospace;\">Blue: %d</p>" +
                "<p style=\"font-familt:monospace;\">ARGB: #%08x</p>" +
                "<br/>" +

                "<h2>Normalized Color</h2>" + 
                "<p style=\"font-family:monospace;\">Red: %.3f</p>" +
                "<p style=\"font-family:monospace;\">Green: %.3f</p>" +
                "<p style=\"font-family:monospace;\">Blue: %.3f</p>" +
                "<br/>" +

                // "\n <h2>Swatch</h2>" + 
                // "\n <p style=\"width:40px;height:40px;background-color:#%02X%02X%02X\"></p>" +
                // "\n <br/>" +

                "<h2>HSV Color</h2>" +
                "<p style=\"font-family:monospace;\">Hue: %.1f</p>" + 
                "<p style=\"font-family:monospace;\">Saturation: %.2f</p>" + 
                "<p style=\"font-family:monospace;\">Value: %.2f</p>" +
                "<br/>" + 

                "<h2>Misc</h2>" +
                "<p>Is Paused: %b</p>" +

                "<h2>ArtifactColorRangeSensor Color</h2>" + 
                "<p>Color: %s</p>" +

                "";

            previousLog = String.format(
                format, 

                dist,

                // ARTIFACTCOLOR
                sensorName,
                artifactcolor.name(),

                // RGB
                red, 
                green, 
                blue, 
                argb,
                
                // SWATCH
                sensor.getNormalizedColors().red, 
                sensor.getNormalizedColors().green, 
                sensor.getNormalizedColors().blue,

                // HSV
                hue,
                saturation,
                value,

                // MISC
                isPaused,
                wrapper.getColor()
            );

            // Logging the data
            telemetry.addLine(previousLog);
            telemetry.update();
            // Coloring the LEDs if at all possible
            if(sensor == rightReloadSensor) {
                if(rightRed != null && artifactcolor == ArtifactColor.PURPLE) {
                    rightRed.on();
                } else if(rightRed != null) {
                    rightRed.off();
                }
                
                // Coloring the LEDs if at all possible
                if(rightGreen != null && artifactcolor == ArtifactColor.GREEN) {
                    rightGreen.on();
                } else if(rightGreen != null) {
                    rightGreen.off();
                }
            }

            if(sensor == leftReloadSensor) {
                if(leftRed != null && artifactcolor == ArtifactColor.PURPLE) {
                    leftRed.on();
                } else if(leftRed != null) {
                    leftRed.off();
                }
                
                // Coloring the LEDs if at all possible
                if(leftGreen != null && artifactcolor == ArtifactColor.GREEN) {
                    leftGreen.on();
                } else if(leftGreen != null) {
                    leftGreen.off();
                }
            }
        }
    }
}