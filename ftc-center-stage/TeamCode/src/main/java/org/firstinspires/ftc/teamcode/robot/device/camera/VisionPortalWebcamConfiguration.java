package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.ArrayList;
import java.util.EnumMap;

// Configuration parameters for all webcams.
public class VisionPortalWebcamConfiguration {
     public final EnumMap<RobotConstantsCenterStage.InternalWebcamId, ConfiguredWebcam> webcams;

    public VisionPortalWebcamConfiguration(EnumMap<RobotConstantsCenterStage.InternalWebcamId, ConfiguredWebcam> pWebcamMap) {
        webcams = pWebcamMap;
    }

    // Note: all fields originate in RobotConfig.xml except for webcamName,
    // which can only be added after we access the FTC HardwareMap in FTCRobot.
    public static class ConfiguredWebcam {
        private WebcamName webcamName;
        public final RobotConstantsCenterStage.InternalWebcamId webcamId;
        public final String serialNumber;
        public final int resolutionWidth;
        public final int resolutionHeight;
        public final ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processors;
        public final CameraCalibration cameraCalibration;
 
        public ConfiguredWebcam(RobotConstantsCenterStage.InternalWebcamId pCameraId,
                                String pSerialNumber,
                                int pResolutionWidth,
                                int pResolutionHeight,
                                ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> pProcessors,
                                CameraCalibration pCameraCalibration) {
            webcamId = pCameraId;
            serialNumber = pSerialNumber;
            resolutionWidth = pResolutionWidth;
            resolutionHeight = pResolutionHeight;
            processors = pProcessors;
            cameraCalibration = pCameraCalibration;
        }

        public void setWebcamName(WebcamName pWebcamName) {
            webcamName = pWebcamName;
        }

        public WebcamName getWebcamName() {
            return webcamName;
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