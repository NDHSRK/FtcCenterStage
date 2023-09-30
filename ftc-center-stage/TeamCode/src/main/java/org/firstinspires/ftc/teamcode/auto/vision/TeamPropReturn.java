package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

// Holds the results of image recognition.
public class TeamPropReturn {

    public final RobotConstants.RecognitionResults recognitionResults;
    public final RobotConstantsCenterStage.TeamPropLocation teamPropLocation;

    public TeamPropReturn(RobotConstants.RecognitionResults pRecognitionResults, RobotConstantsCenterStage.TeamPropLocation pTeamPropLocation) {
        recognitionResults = pRecognitionResults;
        teamPropLocation = pTeamPropLocation;
    }

    // Constructor for OpenCV errors such as "no such file".
    public TeamPropReturn(RobotConstants.RecognitionResults pRecognitionResults) {
        recognitionResults = pRecognitionResults;
        teamPropLocation = RobotConstantsCenterStage.TeamPropLocation.SPIKE_NPOS;
    }
}