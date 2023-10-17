package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.DriveStick;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

@TeleOp(group = "Drive")
//@Disabled
public class BasicDrive extends TeleOpBase {

    private static final String TAG = BasicDrive.class.getSimpleName();

    private DriveStick driveStick;

    private FTCToggleButton toggleHalfVelocity;
    private double driveMotorVelocity = 1.0;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {
        toggleHalfVelocity = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        driveStick = new DriveStick(this, robot.driveTrain);
    }

   @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();
            updatePlayerTwo();
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {
        toggleHalfVelocity.update();
    }

    // Execute the action(s) controlled by Player 1.  This method
    // should be called once per cycle.
    private void updatePlayerOne() {
        driveStick.updateDrive(driveMotorVelocity);
        toggleVelocity();
    }

    private void updatePlayerTwo() {
        // Placeholder
    }

    private void toggleVelocity() {
        if (toggleHalfVelocity.is(FTCButton.State.TAP)) {
            RobotLogCommon.v(TAG, "Entered updateToggleVelocity");
            if (toggleHalfVelocity.toggle() == FTCToggleButton.ToggleState.A)
                driveMotorVelocity = 1.0;
            else
                driveMotorVelocity = 0.5;
        }
    }

}
