package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.DriveStick;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButtonNWay;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

import java.util.EnumSet;

@TeleOp(group = "Drive")
//@Disabled
public class BasicDrive extends TeleOpBase {

    enum DrivePower {FULL_POWER, HALF_POWER}

    private FTCToggleButtonNWay<DrivePower> toggleHalfPower;
    private double driveMotorPower = 1.0;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {
        toggleHalfPower = new FTCToggleButtonNWay<>(this, FTCButton.ButtonValue.GAMEPAD_1_A,
                EnumSet.allOf(DrivePower.class));
        robot.driveTrain.setRunModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

   @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();
            updatePlayerTwo();

            telemetry.addLine("Power toggle " + toggleHalfPower.getToggleState());
            telemetry.update();
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
            telemetry.addLine("Toggle tapped");
            DrivePower newDrivePower = (DrivePower) toggleHalfPower.toggle();
            if (newDrivePower == DrivePower.FULL_POWER)
                driveMotorPower = 1.0;
            else if (newDrivePower == DrivePower.HALF_POWER)
                driveMotorPower = 0.5;

            telemetry.addLine("Driver power is now " + newDrivePower);
            telemetry.update();
            sleep(1000);
        }
    }

}
