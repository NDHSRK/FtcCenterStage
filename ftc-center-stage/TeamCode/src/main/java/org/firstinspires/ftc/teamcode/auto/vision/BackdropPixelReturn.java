package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

// Holds the results of backdrop pixel recognition.
public class BackdropPixelReturn {

    public final RobotConstants.RecognitionResults recognitionResults;
    public final RobotConstantsCenterStage.BackdropPixelOpenSlot backdropPixelOpenSlot;

    public BackdropPixelReturn(RobotConstants.RecognitionResults pRecognitionResults, RobotConstantsCenterStage.BackdropPixelOpenSlot pBackdropPixelOpenSlot) {
        recognitionResults = pRecognitionResults;
        backdropPixelOpenSlot = pBackdropPixelOpenSlot;
    }

    // Constructor for OpenCV errors such as "no such file".
    public BackdropPixelReturn(RobotConstants.RecognitionResults pRecognitionResults) {
        recognitionResults = pRecognitionResults;
        backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.OPEN_SLOT_NPOS;
    }

}