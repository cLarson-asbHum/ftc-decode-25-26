package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ColorGetter;
import org.firstinspires.ftc.teamcode.util.DistanceGetter;

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
        public double greenMaxHue = 160;
        public double minGreenSaturation = 0.47;
        
        public double purpleMinHue = 160;
        public double purpleMaxHue = 360;
        public double minPurpleSaturation = 0.30;
     
        public double minDistCm = 0.00;
        public double maxDistCm = 7.25;
    }

    /**
     * Alternate to ColorSensorConost. Useful for bad color sensors
     */
    public static class AlternateColorSensorConst extends ColorSensorConst {
        public double greenMinHue = 130;
        public double greenMaxHue = 157;
        public double minGreenSaturation = 0.45;
        
        public double purpleMinHue = 157;
        public double purpleMaxHue = 360;
        public double minPurpleSaturation = 0.20;
     
        public double minDistCm = 0.00;
        public double maxDistCm = 12.3;

        public ColorSensorConst asColorSensorConst() {
            final ColorSensorConst result = new ColorSensorConst();
            result.greenMinHue = this.greenMinHue;
            result.greenMaxHue = this.greenMaxHue;
            result.minGreenSaturation = this.minGreenSaturation;
            result.purpleMinHue = this.purpleMinHue;
            result.purpleMaxHue = this.purpleMaxHue;
            result.minPurpleSaturation = this.minPurpleSaturation;
            result.minDistCm = this.minDistCm;
            result.maxDistCm = this.maxDistCm;
            return result;
        }
    }


    /**
     * Constants that define the bounds of what each color is. Saturation is 
     * how colorful the color is (grays and whites have no saturation, neon is 
     * incredibly saturated). Hue is the class of color; 0 is red, 60 is yellow,
     * 120 is green/lime, 180 is cyan, 240 is blue, and 300 is magenta.
     */
    public static ColorSensorConst COLOR_SENSOR_CONST = new ColorSensorConst();

    public final ColorSensorConst colorConst; // So that we can use a local copy if provided at construction

    /**
     * How much the sensor readings are amplified. Determined experimentally.
     */
    public static double GAIN = 200.0;

    // private final ColorRangeSensor sensor;
    private final ColorGetter colorGetter;
    private final DistanceGetter distanceGetter;
    private LynxModule.BulkCachingMode mode = LynxModule.BulkCachingMode.OFF;

    private boolean colorIsOutdated = true; // Used to prevent unecessary calls to clearBulkCache()
    private double dist = 0;
    private double hue = 0;
    private double saturation = 0;

    private final double[] weights;
    private final ArrayList<Integer> colors = new ArrayList<>();
    private final ArrayList<Double> dists = new ArrayList<>();


    public ArtifactColorRangeSensor(ColorRangeSensor sensor) {
        this(sensor, COLOR_SENSOR_CONST);
    }
    
    public ArtifactColorRangeSensor(ColorRangeSensor sensor, ColorSensorConst colorConst) {
        this(sensor, colorConst, new double[] { 1 });
    }

    public ArtifactColorRangeSensor(
        final ColorRangeSensor sensor, 
        final ColorSensorConst colorConst, 
        double[] rollingWeights
    ) {
        // this.sensor = sensor;
        // this.colorConst = colorConst;
        // this.weights = rollingWeights;
        this(() -> sensor.argb(), (unit) -> sensor.getDistance(unit), colorConst, rollingWeights);
        sensor.setGain((float) GAIN);
    }

    public ArtifactColorRangeSensor(
        final ColorRangeSensor colorSensor, 
        final DistanceSensor distanceSensor, 
        ColorSensorConst colorConst, 
        double[] rollingWeights
    ) {
        this(
            () -> colorSensor.argb(), 
            (unit) -> distanceSensor.getDistance(unit), 
            colorConst, 
            rollingWeights
        );
        colorSensor.setGain((float) GAIN);
    }

    public ArtifactColorRangeSensor(ColorGetter colorGetter, DistanceGetter distanceGetter, ColorSensorConst colorConst, double[] rollingWeights) {
        this.colorGetter = colorGetter;
        this.distanceGetter = distanceGetter;
        this.colorConst = colorConst;
        this.weights = rollingWeights;
    }

    @Override
    public ArtifactColor getColor() {
        // Update the sensor values if this is NOT manual and the color is outdated.
        // The color is only ever NOT outdated if we have immediately called clearBulkCache()
        // if(mode != LynxModule.BulkCachingMode.MANUAL && colorIsOutdated) {
        //     clearBulkCache();
        // }

        // colorIsOutdated = true;
        // TODO: Bring back the bulk cache!

        clearBulkCache();


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

        addNewColor(colorGetter.argb(), distanceGetter.getDistance(DistanceUnit.CM));

        // Distance is used to verify that the RGB values will be accurate
        dist = 0;
        hue = 0;
        saturation = 0;

        // Performing the rolling average calculation
        for(int i = 0; i < colors.size(); i++) {
            // Getting the components of the current color
            final int red   = ((colors.get(colors.size() - 1 - i) & 0x00_ff_00_00) >> 16);
            final int green = ((colors.get(colors.size() - 1 - i) & 0x00_00_ff_00) >> 8);
            final int blue  = ((colors.get(colors.size() - 1 - i) & 0x00_00_00_ff) >> 0);
            
            // Convertin the RGB color to something more natural
            hue += weights[i] * JavaUtil.rgbToHue(red, green, blue);
            saturation += weights[i] * JavaUtil.rgbToSaturation(red, green, blue);

            // Adding the distance
            dist += weights[i] * dists.get(dists.size() - 1 - i);
        }

        // final double value = JavaUtil.rgbToValue(red, green, blue);
    }

    private void addNewColor(int argb, double distance) {
        colors.add(argb);
        dists.add(distance);
         
        if(colors.size() > weights.length) {
            colors.remove(0);
        }
        
        if(dists.size() > weights.length) {
            dists.remove(0);
        }
    }

    protected ArtifactColor calculateArtifactColor(double hue, double sat, double distCm) {
        // If the color cannot be safely determined
        if(distCm >= colorConst.maxDistCm || distCm <= colorConst.minDistCm) {
            return ArtifactColor.UNKNOWN;
        } 

        // Checking that the hue is correct
        boolean isPurple = colorConst.purpleMinHue <= hue && hue <= colorConst.purpleMaxHue;
        boolean isGreen  = colorConst.greenMinHue  <= hue && hue <= colorConst.greenMaxHue;

        // Checking that the saturations are also correct
        isPurple = isPurple && sat > colorConst.minPurpleSaturation;
        isGreen  = isGreen  && sat > colorConst.minGreenSaturation;

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


    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Rolling Colors", colors);
        telemetry.addData("Rolling Dists", dists);
        telemetry.addData("Hue", hue);
        telemetry.addData("saturation", saturation);
    }
}