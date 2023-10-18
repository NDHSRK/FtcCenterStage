package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.EnumMap;

// Configuration parameters for all webcams.
public class VisionPortalWebcamConfiguration {
    public static class ConfiguredWebcam {
        // The final fields originate in RobotConfig.xml.
        public final RobotConstantsCenterStage.InternalWebcamId internalWebcamId;
        public final String serialNumber;
        public final int resolutionWidth;
        public final int resolutionHeight;
        public final ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processorIdentifiers;
        public final CameraCalibration cameraCalibration;

        // The non-final fields all have setters which are called during initialization.
        private WebcamName webcamName;
        private VisionPortalWebcam visionPortalWebcam;
        private EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> processors;

        public ConfiguredWebcam(RobotConstantsCenterStage.InternalWebcamId pCameraId,
                                String pSerialNumber,
                                int pResolutionWidth,
                                int pResolutionHeight,
                                ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> pProcessorIdentifierss,
                                CameraCalibration pCameraCalibration) {
            internalWebcamId = pCameraId;
            serialNumber = pSerialNumber;
            resolutionWidth = pResolutionWidth;
            resolutionHeight = pResolutionHeight;
            processorIdentifiers = pProcessorIdentifierss;
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

        public void setProcessors(EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pProcessors) {
            processors = pProcessors;
        }

        public EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> getProcessors() {
            return processors;
        }
    }

    public static class CameraCalibration {
          public final double focalLengthX;
          public final double focalLengthY;
          public final double opticalCenterX;
          public final double opticalCenterY;

          public CameraCalibration(double pFocalLengthX, double pFocalLengthY,
                                   double pOpticalCenterX, double pOpticalCenterY) {
              focalLengthX = pFocalLengthX;
              focalLengthY = pFocalLengthY;
              opticalCenterX = pOpticalCenterX;
              opticalCenterY = pOpticalCenterY;
          }
    }

}