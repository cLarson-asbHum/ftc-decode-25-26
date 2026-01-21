package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;

public class ArtifactColorLed {
    private final SwitchableLight red;
    private final SwitchableLight green;

    private ArtifactColor lastColor = ArtifactColor.UNKNOWN;
    private boolean greenEnabled = true;
    private boolean redEnabled = true;

    public ArtifactColorLed(SwitchableLight red, SwitchableLight green) {
        this.red = red;
        this.green = green;

        color(ArtifactColor.UNKNOWN);
    }

    public boolean color(ArtifactColor color) {
        // Getting which led needs to be off or on
        final boolean enableGreen = color == ArtifactColor.GREEN;
        final boolean enableRed = color == ArtifactColor.PURPLE;

        // Coloring the leds as necessary
        boolean didAnything = false;
        if(enableGreen != greenEnabled) {
            green.enableLight(enableGreen);
        }

        if(enableRed != redEnabled) {
            red.enableLight(enableRed);
        }

        return false;
    }
}