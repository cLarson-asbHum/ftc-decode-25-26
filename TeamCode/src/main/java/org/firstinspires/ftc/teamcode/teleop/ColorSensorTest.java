package org.firstinspires.ftc.teamcode.teleop;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;

@Configurable
@TeleOp(group="B - Testing")
public class ColorSensorTest extends LinearOpMode {
    public static class ColorSensorConst {
        public double greenMinHue = 130;
        public double greenMaxHue = 165;
        public double minGreenSaturation = 0.55;
        
        public double purpleMinHue = 170;
        public double purpleMaxHue = 360;
        public double minPurpleSaturation = 0.4;
     
        public double minDistCm = 0.00;
        public double maxDistCm = 7;
    }

    public static ColorSensorConst COLOR_SENSOR_CONST = new ColorSensorConst();

    public static enum ArtifactColor { 
        GREEN,
        PURPLE,
        UNKNOWN 
    }

    public static int TRANSMISSION_MS_INTERVAL = 33;
    public static int SLEEP_TIME = 33;
    public static double GAIN = 200.0;
    public static boolean USE_NORM = false;
    public static boolean USE_BITMAP = false;


    @Override
    public void runOpMode() {
        final ColorRangeSensor rightReloadSensor = hardwareMap.tryGet(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = hardwareMap.tryGet(ColorRangeSensor.class, "leftReload");

        // Getting a non-null color sensor
        if(rightReloadSensor == null && leftReloadSensor == null) {
            // Both are null; throw an exception
            throw new RuntimeException("No hardware of type ColorRangeSensor with any of the following names was found: \n    \"rightReload\", \n    \"leftReload\"");
        }

        telemetry.setMsTransmissionInterval(40);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        final ColorRangeSensor sensor = rightReloadSensor == null ? leftReloadSensor : rightReloadSensor;
        final ArtifactColorRangeSensor wrapper = new ArtifactColorRangeSensor(sensor);

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
            
            final ArtifactColor artifactcolor = determineArtifactColor(hue, saturation, dist);

            // Formatting the data
            final String format = 
                "<p>Distance (cm): %.2f</p>" + 
                "<br/>" + 

                "<h2>ArtifactColor</h2>" +
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
        }
    }

    protected ArtifactColor determineArtifactColor(double hue, double sat, double distCm) {
        // If the color cannot be safely determined
        if(distCm >= COLOR_SENSOR_CONST.maxDistCm || distCm <= COLOR_SENSOR_CONST.minDistCm) {
            return ArtifactColor.UNKNOWN;
        } 

        // Checking that the hue is correct
        boolean isPurple = COLOR_SENSOR_CONST.purpleMinHue <= hue && hue <= COLOR_SENSOR_CONST.purpleMaxHue;
        boolean isGreen  = COLOR_SENSOR_CONST.greenMinHue  <= hue && hue <= COLOR_SENSOR_CONST.greenMaxHue;

        // Checking that the saturations are also correct
        isPurple = isPurple && sat > COLOR_SENSOR_CONST.minPurpleSaturation;
        isGreen  = isGreen  && sat > COLOR_SENSOR_CONST.minGreenSaturation;

        // Check for nonsense and possibly return purple
        if(isGreen && isPurple) {
            // Neither color has dominance; the color cannot be determined
            return ArtifactColor.UNKNOWN;
        }

        if(isPurple) {
            return ArtifactColor.PURPLE;
        }
        
        // Check for nonsense and possibly return green
        if(isGreen) {
            return ArtifactColor.GREEN;
        }

        // The color was unrecognized; the color is unknown
        return ArtifactColor.UNKNOWN;
    }
}