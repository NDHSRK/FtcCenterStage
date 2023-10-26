package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.ArrayList;

// Configuration parameters for all webcams.
public class VisionPortalWebcamConfiguration {
    public static class ConfiguredWebcam {
        // The final fields originate in RobotConfig.xml.
        public final RobotConstantsCenterStage.InternalWebcamId internalWebcamId;
        public final String serialNumber;
        public final int resolutionWidth;
        public final int resolutionHeight;

        // A webcam may support more than one processor but only one may be
        // active at any given time.
        public final ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processorIdentifiers;
        public final CameraCalibration cameraCalibration;

        // The non-final fields have setters which are called during initialization.
        private WebcamName webcamName;
        private VisionPortalWebcam visionPortalWebcam;

        public ConfiguredWebcam(RobotConstantsCenterStage.InternalWebcamId pCameraId,
                                String pSerialNumber,
                                int pResolutionWidth,
                                int pResolutionHeight,
                                ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> pProcessorIdentifiers,
                                CameraCalibration pCameraCalibration) {
            internalWebcamId = pCameraId;
            serialNumber = pSerialNumber;
            resolutionWidth = pResolutionWidth;
            resolutionHeight = pResolutionHeight;
            processorIdentifiers = pProcessorIdentifiers;
            cameraCalibration = pCameraCalibration;
        }

        public void setWebcamName(WebcamName pWebcamName) {
            webcamName = pWebcamName;
        }

        public WebcamName getWebcamName() {
            return webcamName;
        }

        public void setVisionPortalWebcam(VisionPortalWebcam pVisionPortalWebcam) {
            visionPortalWebcam = pVisionPortalWebcam;
        }

        public VisionPortalWebcam getVisionPortalWebcam() {
            return visionPortalWebcam;
        }

    }

    public static class CameraCalibration {
          public final double focalLengthX;
          public final double focalLengthY;
          public final double principalPointX;
          public final double principalPointY;

          public CameraCalibration(double pFocalLengthX, double pFocalLengthY,
                                   double pPrincipalPointX, double pPrincipalPointY) {
              focalLengthX = pFocalLengthX;
              focalLengthY = pFocalLengthY;
              principalPointX = pPrincipalPointX;
              principalPointY = pPrincipalPointY;
          }
    }

}