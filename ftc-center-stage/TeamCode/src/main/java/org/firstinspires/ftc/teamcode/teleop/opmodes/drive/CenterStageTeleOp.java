package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButton;
import org.firstinspires.ftc.teamcode.teleop.common.ParallelDrive;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpWithAlliance;

import java.util.Objects;

public class CenterStageTeleOp extends TeleOpWithAlliance {

    private static final String TAG = CenterStageTeleOp.class.getSimpleName();

    // Define buttons that return a boolean.

    private final FTCButton hangUp;
    private final FTCButton hangDown;
    private final FTCToggleButton toggleSpeed;
    // private final FTCButton intake;
    private final FTCButton intake;
    private final FTCButton outtake;
    // private final FTCButton outtake;
    private final FTCButton minimumGear;
    private final FTCButton maximumGear;
    private final FTCButton increaseGear;
    private final FTCButton decreaseGear;
    private final FTCButton launchByGearValue;
    private final FTCButton zeroAll;

    // Drive train
    private double driveTrainVelocity;

    private double previousDriveTrainVelocity;
    private final double driveTrainVelocityHigh;
    private final double driveTrainVelocityLow;
    private final ParallelDrive parallelDrive;

    public CenterStageTeleOp(RobotConstants.Alliance pAlliance,
                                 LinearOpMode pLinearOpMode, FTCRobot pRobot,
                                 @Nullable FTCAuto pAutonomous) {
                                super(pAlliance, pLinearOpMode, pRobot, pAutonomous);
        RobotLogCommon.c(TAG, "Constructing CenterStageTeleOp");
        RobotLogCommon.setMostDetailedLogLevel(Objects.requireNonNull(robot.teleOpSettings, "robot.teleOpSettings unexpectedly null").logLevel);
 
        driveTrainVelocityHigh = robot.teleOpSettings.driveTrainVelocityHigh;
        driveTrainVelocity = driveTrainVelocityHigh;
        previousDriveTrainVelocity = driveTrainVelocity;
        driveTrainVelocityLow = robot.teleOpSettings.driveTrainVelocityLow;

        // Gamepad 1
        hangDown = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
        hangUp = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);
        toggleSpeed = new FTCToggleButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_A);

        // Gamepad 2
        // Bumpers
        outtake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_RIGHT_BUMPER);
        intake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_LEFT_BUMPER);

        // ABXY Buttons
        launchByGearValue = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_A);
        zeroAll = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_X);

        // D-Pad
        minimumGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_LEFT);
        maximumGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_RIGHT);
        increaseGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);
        decreaseGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_DOWN);

        // Start the drive train in parallel.
        parallelDrive = new ParallelDrive(linearOpMode, robot.driveTrain, driveTrainVelocity);
        RobotLogCommon.c(TAG, "Finished constructing CenterStageTeleOp");
    }

    @Override
    public void runTeleOp() throws InterruptedException {
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs.
            // 12/28/2022 From the FTC SDK documentation: "whether the OpMode is
            // currently active. If this returns false, you should break out of
            // the loop in your runOpMode() method and return to its caller.
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                return;
            }

            //## The drive train thread must be started here because
            // only now does opModeIsActive() return true.
            parallelDrive.startDriveTrain();

            while (linearOpMode.opModeIsActive()) {
                updateButtons();
                updateActions();
            }
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, linearOpMode);
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {

        // Game Controller 1
        toggleSpeed.update();
//        lowDriveTrainVelocity.update();
//        highDriveTrainVelocity.update();
        //**TODO add all updates for Game Controller 1

        // Game Controller 2
       //**TODO add all updates for Game Controller 2
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() throws Exception {
        updateToggleSpeed ();

        if (velocityChanged())
            parallelDrive.setVelocity(driveTrainVelocity);

        //**TODO add all actions
    }

private void updateToggleSpeed () {

        if (toggleSpeed.is(FTCButton.State.TAP)) {
            FTCToggleButton.ToggleState newToggleState = toggleSpeed.toggle();
            if (newToggleState == FTCToggleButton.ToggleState.A) {
                driveTrainVelocity = driveTrainVelocityHigh;
            }
            else driveTrainVelocity = driveTrainVelocityLow;
        }
}
private boolean velocityChanged() {
        if (driveTrainVelocity == previousDriveTrainVelocity)
            return false;

        previousDriveTrainVelocity = driveTrainVelocity;
                return true;
}
}
