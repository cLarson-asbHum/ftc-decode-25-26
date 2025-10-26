package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
@TeleOp(group="B - Testing")
public class MotifWebcamTest extends LinearOpMode {
    public static double YAW = 180;
    public static boolean YAW_IN_RADIANS = false; // Determines wheter YAW is in radians

    public static int GAIN = 1;
    public static int EXPOSURE_MS = 33;

    @Override
    public void runOpMode() {
        final WebcamName obeliskViewer = hardwareMap.get(WebcamName.class, "obeliskViewer");

        double lastSetYaw = YAW;
        boolean lastSetYawIsInRadians = YAW_IN_RADIANS;
        double lastSetGain = GAIN;
        double lastSetExposureMs = EXPOSURE_MS;

        final MotifWebcamWrapper motifGetter = new MotifWebcamWrapper(
            obeliskViewer, 
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, -1)
        );
        motifGetter.setGlobalRobotOrientation(yprFromYaw());

        // Waiting for the camera to open
        if (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        setManualExposure(motifGetter, GAIN, EXPOSURE_MS);

        telemetry.setMsTransmissionInterval((int) (1000 / 24));
        telemetry.setNumDecimalPlaces(0, 2);

        // Running the opmode
        while(opModeIsActive() || opModeInInit()) {
            // Pausing the stream if the b button is pressed
            if(gamepad1.b || gamepad2.b) {
                motifGetter.disable();
            } else if (gamepad1.a || gamepad2.a) {
                motifGetter.enable();
            }

            // Updating the YAW if it is changed in Panels (or Dashboard)
            if(lastSetYaw != YAW || lastSetYawIsInRadians != YAW_IN_RADIANS) {
                motifGetter.setGlobalRobotOrientation(yprFromYaw());
                lastSetYaw = YAW;
                lastSetYawIsInRadians = YAW_IN_RADIANS;
            }

            // Updating GAIN and EXPOSURE_MS
            boolean hasUpdatedCameraControls = false;
            if(lastSetGain != GAIN) {
                hasUpdatedCameraControls = true;
                lastSetGain = GAIN;
            }

            if(lastSetExposureMs != EXPOSURE_MS) {
                hasUpdatedCameraControls = true;
                lastSetExposureMs = EXPOSURE_MS;
            }

            if(hasUpdatedCameraControls) {
                setManualExposure(motifGetter, EXPOSURE_MS, GAIN);
            }


            // Getting the observed motif
            telemetry.addData("Current Motif", motifGetter.getMotif().name());
            if(YAW_IN_RADIANS) {
                telemetry.addData("Current Yaw (deg)", AngleUnit.DEGREES.fromRadians(YAW));
            } else {
                telemetry.addData("Current Yaw (deg)", YAW);
            }
            telemetry.addLine();
            telemetry.addData("Stream FPS", motifGetter.getStream().getFps());
            telemetry.addData("Processor Time (ms)", motifGetter.getProcessor().getPerTagAvgPoseSolveTime());
            telemetry.addData("Found Tags", motifGetter.getProcessor().getDetections());
            telemetry.addData("Found Motifs", motifGetter.getMotif().name());
            telemetry.update();
        }
    }

    private boolean setManualExposure(MotifWebcamWrapper motifGetter, int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (motifGetter.getStream() == null) {
            return false;
        }

        // Wait for the camera to be open
        if (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = motifGetter.getStream().getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = motifGetter.getStream().getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    private YawPitchRollAngles yprFromYaw() {
        return new YawPitchRollAngles(
            YAW_IN_RADIANS ? AngleUnit.RADIANS : AngleUnit.DEGREES,
            YAW,
            0,
            0,
            -1 // TODO: Does this timestamp really matter?
        );
    }

    private static class MotifWebcamWrapper extends MotifWebcam {
        public MotifWebcamWrapper(WebcamName name, YawPitchRollAngles stuff) {
            super(name, stuff);
        }

        public AprilTagProcessor getProcessor() {
            return this.processor;
        }

        public VisionPortal getStream() {
            return this.stream;
        }
    }
}