package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Wraps a color sensor and determines the most accurate color based off of the 
 * reading. How the color data is updated can also be set, similar to the 
 * LynxModule's `BulkCachingMode`.
 */
public class ArtifactColorRangeSensor implements ArtifactColorGetter {
    /**
     * Constants that define the bounds of what each color is. Saturation is 
     * how colorful the color is (grays and whites have no saturation, neon is 
     * incredibly saturated). Hue is the class of color; 0 is red, 60 is yellow,
     * 120 is green/lime, 180 is cyan, 240 is blue, and 300 is magenta.
     */
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

    /**
     * Constants that define the bounds of what each color is. Saturation is 
     * how colorful the color is (grays and whites have no saturation, neon is 
     * incredibly saturated). Hue is the class of color; 0 is red, 60 is yellow,
     * 120 is green/lime, 180 is cyan, 240 is blue, and 300 is magenta.
     */
    public static ColorSensorConst COLOR_SENSOR_CONST = new ColorSensorConst();

    /**
     * How much the sensor readings are amplified. Determined experimentally.
     */
    public static double GAIN = 200.0;

    private final ColorRangeSensor sensor;
    private LynxModule.BulkCachingMode mode = LynxModule.BulkCachingMode.OFF;

    private boolean colorIsOutdated = true; // Used to prevent unecessary calls to clearBulkCache()
    private double dist = 0;
    private double hue = 0;
    private double saturation = 0;

    public ArtifactColorRangeSensor(ColorRangeSensor sensor) {
        this.sensor = sensor;
        this.sensor.setGain((float) GAIN);
    }

    @Override
    public ArtifactColor getColor() {
        // Update the sensor values if this is NOT manual and the color is outdated.
        // The color is only ever NOT outdated if we have immediately called clearBulkCache()
        if(mode != LynxModule.BulkCachingMode.MANUAL && colorIsOutdated) {
            clearBulkCache();
        }

        colorIsOutdated = true;

        return calculateArtifactColor(hue, saturation, dist);
    }

    /**
     * Changes the bulk caching mode. Only `MANUAL` affects behavior. Defaults to 
     * `OFF`.
     * 
     * @param mode The new mode. Only matters if equal to `MANUAL`
     */
    public void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
        this.mode = mode;
        colorIsOutdated = true;
    }

    /**
     * Updates all sensor readings. This is required if the mode is set to 
     * `MANUAL`.
     */
    public void clearBulkCache() {
        colorIsOutdated = false;

        // Distance is used to verify that the RGB values will be accurate
        dist = sensor.getDistance(DistanceUnit.CM);

        // Getting the components of the current color
        final int red = sensor.red();
        final int green = sensor.green();
        final int blue = sensor.blue();
        
        // Convertin the RGB color to something more natural
        hue = JavaUtil.rgbToHue(red, green, blue);
        saturation = JavaUtil.rgbToSaturation(red, green, blue);
        // final double value = JavaUtil.rgbToValue(red, green, blue);
    }

    protected ArtifactColor calculateArtifactColor(double hue, double sat, double distCm) {
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