package org.firstinspires.ftc.teamcode.teleop.opmodes.autoaction;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpWithAlliance;

import java.util.logging.Level;

public class TeleOpTakePicture extends TeleOpWithAlliance {

    private static final String TAG = TeleOpTakePicture.class.getSimpleName();

    // Define buttons that return a boolean.
    private final FTCButton takePictureButton;

    public TeleOpTakePicture(RobotConstants.Alliance pAlliance,
                             LinearOpMode pLinearOpMode, FTCRobot pRobot,
                             @Nullable FTCAuto pAutonomous) {
        super(pAlliance, pLinearOpMode, pRobot, pAutonomous);

        takePictureButton = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_Y);
    }

    @Override
    public void runTeleOp() throws Exception {

        while (linearOpMode.opModeIsActive()) {
            updateButtons();
            updateActions();
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {

        //Game Controller 1
        takePictureButton.update();
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() throws Exception {
        takePicture();
    }

    // When you run an Autonomous OpMode it sets its own logging level.
    // So save and restore the TeleOp logging level; NONE will never
    // be overridden.
    private void takePicture() throws Exception {
        if (takePictureButton.is(FTCButton.State.TAP)) {
            Level teleOpLogLevel = RobotLogCommon.getMostDetailedLogLevel();
            ftcAuto.runRobotWithCameras(RobotConstantsCenterStage.OpMode.TELEOP_TAKE_PICTURE_WEBCAM);
            RobotLogCommon.setMostDetailedLogLevel(teleOpLogLevel); // restore
        }
    }
}
