package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.ParallelDrive;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpWithAlliance;

import java.util.Objects;

public class CenterStageTeleOp extends TeleOpWithAlliance {

    private static final String TAG = CenterStageTeleOp.class.getSimpleName();

    // Define buttons that return a boolean.
    private final FTCButton lowDriveTrainVelocity;
    private final FTCButton highDriveTrainVelocity;

    private final FTCButton elevatorHeightLowJunction;
    private final FTCButton elevatorHeightMediumJunction;
    private final FTCButton elevatorHeightHighJunction;
    private final FTCButton deliverCone;
    private final FTCButton elevatorAndArmDown;
    private final FTCButton elevatorAndArmSafe;
    private final FTCButton elevatorManual;
    private final FTCButton coneGrabberToggle;
    private final FTCButton deliverConeWithCamera;

    // Drive train
    private double driveTrainVelocity;
    private double previousDriveTrainVelocity = driveTrainVelocity;
    private final double driveTrainVelocityHigh;
    private final double driveTrainVelocityMedium;
    private final double driveTrainVelocityLow;
    private final ParallelDrive parallelDrive;

    public CenterStageTeleOp(RobotConstants.Alliance pAlliance,
                                 LinearOpMode pLinearOpMode, FTCRobot pRobot,
                                 @Nullable FTCAuto pAutonomous) {
                                super(pAlliance, pLinearOpMode, pRobot, pAutonomous);
        RobotLogCommon.c(TAG, "Constructing CenterStageTeleOp");
        RobotLogCommon.setMostDetailedLogLevel(Objects.requireNonNull(robot.teleOpSettings, "robot.teleOpSettings unexpectedly null").logLevel);
 
        driveTrainVelocityHigh = robot.teleOpSettings.driveTrainVelocityHigh;
        driveTrainVelocityMedium = robot.teleOpSettings.driveTrainVelocityMedium;
        driveTrainVelocity = driveTrainVelocityMedium; // set as default
        driveTrainVelocityLow = robot.teleOpSettings.driveTrainVelocityLow;

        // Gamepad 1
        lowDriveTrainVelocity = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
        highDriveTrainVelocity = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);

        // Gamepad 2
        // Bumpers
        elevatorAndArmSafe = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_RIGHT_BUMPER);
        elevatorManual = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_LEFT_BUMPER);

        // ABXY Buttons
        deliverCone = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_A);
        coneGrabberToggle = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_Y);
        deliverConeWithCamera = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_X);

        // D-Pad
        elevatorHeightLowJunction = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_LEFT);
        elevatorHeightMediumJunction = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_RIGHT);
        elevatorHeightHighJunction = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);
        elevatorAndArmDown = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_DOWN);

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
        lowDriveTrainVelocity.update();
        highDriveTrainVelocity.update();
        //**TODO add all updates for Game Controller 1

        // Game Controller 2
       //**TODO add all updates for Game Controller 2
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() throws Exception {
        updateDriveTrainVelocity();

        if (velocityChanged())
            parallelDrive.setVelocity(driveTrainVelocity);

        //**TODO add all actions
    }

    private void updateDriveTrainVelocity() {
        if (highDriveTrainVelocity.is(FTCButton.State.HELD) && !lowDriveTrainVelocity.is(FTCButton.State.HELD)) {
            driveTrainVelocity = driveTrainVelocityHigh;
            //RobotLogCommon.v(TAG, "Drive Train is at HIGH velocity");
        } else if (!highDriveTrainVelocity.is(FTCButton.State.HELD) && lowDriveTrainVelocity.is(FTCButton.State.HELD)) {
            driveTrainVelocity = driveTrainVelocityLow;
            //RobotLogCommon.v(TAG, "Drive Train is at LOW velocity");
        } else {
            driveTrainVelocity = driveTrainVelocityMedium;
            //RobotLogCommon.v(TAG, "Drive Train is at MEDIUM velocity");
        }
    }

    private boolean velocityChanged() {
        if (driveTrainVelocity == previousDriveTrainVelocity)
            return false;

        previousDriveTrainVelocity = driveTrainVelocity;
        return true;
    }

}
