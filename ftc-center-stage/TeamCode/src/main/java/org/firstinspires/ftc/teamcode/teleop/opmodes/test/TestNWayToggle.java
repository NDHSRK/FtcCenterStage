package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButtonNWay;

import java.util.EnumSet;

@TeleOp(name = "TestNWAYToggle", group = "Test")
//@Disabled
public class TestNWayToggle extends LinearOpMode {

    enum DrivePower {FULL_POWER, HALF_POWER}

    private FTCToggleButtonNWay<DrivePower> toggleHalfPower;
    private double driveMotorPower = 1.0;

    @Override
    public void runOpMode() {
        toggleHalfPower = new FTCToggleButtonNWay<>(this, FTCButton.ButtonValue.GAMEPAD_1_A,
        EnumSet.allOf(DrivePower.class));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();

            telemetry.addLine("Power toggle " + toggleHalfPower.getToggleState());
            telemetry.update();
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {
        toggleHalfPower.update();
    }

    // Execute the action(s) controlled by Player 1. This method
    // should be called once per cycle.
    private void updatePlayerOne() {
        togglePowerLevel();
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
