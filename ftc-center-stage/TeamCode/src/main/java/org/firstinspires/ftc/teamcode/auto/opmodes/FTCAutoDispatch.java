package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

// Use this dispatcher class to place the launching of FTCAuto and all of the error
// handling in one place.
public class FTCAutoDispatch {

    //## We can't send the OpMode in to the FTCAuto constructor because
    // when we run FTCAuto from TeleOp we only use pseudo OpModes that
    // are referenced after TeleOp has already started.
    public static void runAuto(RobotConstants.RunType pRunType,
                               RobotConstantsCenterStage.OpMode pOpMode,
                               RobotConstants.Alliance pAlliance,
                               LinearOpMode pLinear) throws InterruptedException {

        final String TAG = FTCAutoDispatch.class.getSimpleName();

        pLinear.telemetry.setAutoClear(false); // keep our messages on the driver station

        // LCHSAuto, the common class for all autonomous opmodes, needs
        // access to the public data fields and methods in LinearOpMode.
        try {
            RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.AUTO_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

            RobotLogCommon.c(TAG, "Constructing FTCRobot with run type " + pRunType);
            FTCRobot robot = new FTCRobot(pLinear, pRunType);

            RobotLogCommon.c(TAG, "Constructing FTCAuto with alliance " + pAlliance + " running OpMode " + pOpMode);
            FTCAuto ftcAuto = new FTCAuto(pAlliance, pLinear, robot, pRunType);

            pLinear.telemetry.addData(TAG, "Waiting for start ...");
            pLinear.telemetry.update();

            //**TODO Where does control go if there's a crash or
            // a requested termination in init?
            pLinear.waitForStart();

            //## 12/28/2022 Note: if initialization is complete and the play
            // button is showing we're in waitForStart(); if the driver hits
            // the small stop button, isOpModeActive() returns false but
            // runRobot() will be called. See further notes at that location.
            RobotLogCommon.i(TAG, "After waitForStart()");
            pLinear.telemetry.addData(TAG, "Running ...");
            pLinear.telemetry.update();

            ftcAuto.runRobot(pOpMode);
        }
        catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, pLinear);
        }

        finally {
            RobotLogCommon.closeLog();

            // New in FTC SDK 7.2.
            pLinear.terminateOpModeNow();
        }
    }
}
