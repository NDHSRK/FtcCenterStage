package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.DriveStick;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

@TeleOp(group = "Drive")
@Disabled
public class BasicDrive extends TeleOpBase {

    private static final String TAG = BasicDrive.class.getSimpleName();

    private FTCToggleButton toggleHalfPower;
    private double driveMotorPower = 1.0;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {
        toggleHalfPower = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        robot.driveTrain.setRunModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        toggleHalfPower.update();
    }

    // Execute the action(s) controlled by Player 1.  This method
    // should be called once per cycle.
    private void updatePlayerOne() {
        robot.driveTrain.runAtPowerAll(DriveStick.updateDrivePower(this, driveMotorPower));
        togglePowerLevel();
    }

    private void updatePlayerTwo() {
        // Placeholder
    }

    private void togglePowerLevel() {
        if (toggleHalfPower.is(FTCButton.State.TAP)) {
            if (toggleHalfPower.toggle() == FTCToggleButton.ToggleState.A)
                driveMotorPower = 1.0;
            else
                driveMotorPower = 0.5;
        }
    }

}
