package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.BackdropPixelParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.xml.BackdropPixelParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.BackdropPixelRendering;
import org.firstinspires.ftc.teamcode.robot.device.camera.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;

import java.util.Objects;

// This OpMode gives the drive team a way to check and
// modify the grayscale thresholding of the
// webcam frame that contains the AprilTag and possibly
// a previously placed yellow pixel on the backdrop.
@TeleOp(name = "BackdropPixelViewer", group = "Configure")
//@Disabled
public class BackdropPixelViewer extends LinearOpMode {
    private static final String TAG = BackdropPixelViewer.class.getSimpleName();
    private static final int THRESHOLD_CHANGE = 5;

    private BackdropPixelParametersXML backdropPixelParametersXML;
    private BackdropPixelParameters backdropPixelParameters;

    //**TODO private BackdropPixelImageXML backdropPixelImageXML;
    // private BackdropPixelImageParameters backdropPixelImageParameters;

    private CameraStreamProcessor backdropPixelProcessor;

    private FTCButton increaseThreshold;
    private FTCButton decreaseThreshold;
    private FTCButton requestImageCapture;
    private BackdropPixelRendering backdropPixelRendering;
    private VisionParameters.GrayParameters originalGrayParameters;
    private int currentThresholdLow;
    private boolean grayscaleParametersChanged = false;

    // In this OpMode all of the action takes place during init().
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.ii(TAG, "Initializing the BackdropPixelViewer");

        // Read the parameters for backdrop pixel recognition from the xml file.
        backdropPixelParametersXML = new BackdropPixelParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
        backdropPixelParameters = backdropPixelParametersXML.getBackdropPixelParameters();

        // Get the camera configuration from RobotConfig.xml.
        FTCRobot robot = new FTCRobot(this, RobotConstants.RunType.TELEOP_VISION_PREVIEW);

        // Start the rear webcam with the backdrop pixel processor.
        if (robot.configuredWebcams == null)
            throw new AutonomousRobotException(TAG, "There are no webcams in the current configuration");

        VisionPortalWebcamConfiguration.ConfiguredWebcam rearWebcamConfiguration =
                Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.REAR_WEBCAM),
                        TAG + " The REAR_WEBCAM is not configured");

        backdropPixelProcessor = new CameraStreamProcessor.Builder().build();
        VisionPortalWebcam backdropPixelWebcam = new VisionPortalWebcam(rearWebcamConfiguration,
                RobotConstantsCenterStage.ProcessorIdentifier.BACKDROP_PIXEL,
                Pair.create(backdropPixelProcessor, true));

        if (!backdropPixelWebcam.waitForWebcamStart(2000))
            throw new AutonomousRobotException(TAG, "Backdrop pixel webcam timed out on start");

        rearWebcamConfiguration.setVisionPortalWebcam(backdropPixelWebcam);
        RobotLog.ii(TAG, "BackdropPixelViewer successfully started on the rear webcam");

        //**TODO To avoid parsing RobotAction.XML for the ROI definitions for the
        // backdrop under DRIVE_TO_BACKDROP_APRIL_TAG - which should be the same
        // for all OpModes - put the definitions in a separate file *with the risk
        // that these definitions must be kept in sync*.
        /*
        try {
            //**TODO BackdropPixelImageXML backdropPixelImageXML = new BackdropImageXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
            //**TODO VisionParameters.ImageParameters backdropPixelImageParameters = backdropPixelImageXML.getBackdropPixelImageParameters();
        } catch (ParserConfigurationException | IOException | SAXException |
                 XPathExpressionException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }
        */

        originalGrayParameters = backdropPixelParameters.grayscaleParameters;
        currentThresholdLow = originalGrayParameters.threshold_low;

        backdropPixelRendering = new BackdropPixelRendering(this, backdropPixelImageParameters, originalGrayParameters);
        backdropPixelProcessor.setCameraStreamRendering(backdropPixelRendering);

        increaseThreshold = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        decreaseThreshold = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);
        requestImageCapture = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);

        telemetry.addLine("Press DPAD UP to increase threshold");
        telemetry.addLine("Press DPAD DOWN to decrease threshold");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch play to *END* the OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayerOne();
        }

        if (opModeIsActive()) {
            // If there have been any changes to the grayscale values
            // write them out to TeamPropParameters.xml now.
            if (grayscaleParametersChanged) {
                backdropPixelParametersXML.writeBackdropPixelParametersFile();
                RobotLog.ii(TAG, "Writing BackdropPixelParameters.xml");
                telemetry.addLine("Writing BackdropPixelParameters.xml");
                telemetry.update();
                sleep(1500);
            }
            telemetry.addLine("Ending the BackdropPixelViewer");
            telemetry.update();
        }
    }

    private void updateButtons() {
        increaseThreshold.update();
        decreaseThreshold.update();
        requestImageCapture.update();
    }

    private void updatePlayerOne() {
        updateIncreaseThreshold();
        updateDecreaseThreshold();
        updateRequestImageCapture();
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateIncreaseThreshold() {
        if (increaseThreshold.is(FTCButton.State.TAP)) {
            if (backdropPixelRendering == null || currentThresholdLow == 255)
                return; //**TODO no OpMode has been selected; can't go above maximum

            currentThresholdLow += THRESHOLD_CHANGE;

            VisionParameters.GrayParameters updatedVisionParameters = new VisionParameters.GrayParameters(originalGrayParameters.median_target, currentThresholdLow);
            backdropPixelParametersXML.setBackdropPixelGrayParameters(updatedVisionParameters);
            grayscaleParametersChanged = true;

            backdropPixelRendering.setGrayscaleThresholdParameters(updatedVisionParameters);
            telemetry.addLine("Grayscale median " + originalGrayParameters.median_target);
            telemetry.addLine("Grayscale low threshold " + currentThresholdLow);
            telemetry.update();
        }
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateDecreaseThreshold() {
        if (decreaseThreshold.is(FTCButton.State.TAP)) {
            if (backdropPixelRendering == null || currentThresholdLow == 0)
                return; // no OpMode has been selected; can't go below minimum

            currentThresholdLow -= THRESHOLD_CHANGE;

            VisionParameters.GrayParameters updatedVisionParameters = new VisionParameters.GrayParameters(originalGrayParameters.median_target, currentThresholdLow);
            backdropPixelParametersXML.setBackdropPixelGrayParameters(updatedVisionParameters);
            grayscaleParametersChanged = true;

            backdropPixelRendering.setGrayscaleThresholdParameters(updatedVisionParameters);
            telemetry.addLine("Grayscale median " + originalGrayParameters.median_target);
            telemetry.addLine("Grayscale low threshold " + currentThresholdLow);
            telemetry.update();
        }
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateRequestImageCapture() {
        if (requestImageCapture.is(FTCButton.State.TAP)) {
            if (backdropPixelRendering != null)
                backdropPixelRendering.requestImageCapture();
        }
    }

}