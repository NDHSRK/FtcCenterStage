package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

@TeleOp(name = "ElevatorCalibration", group = "Test")
// @Disabled
public class ElevatorCalibration extends TeleOpBase {

    private Elevator elevator;
    private DualMotorMotion dualMotorMotion;

    private FTCButton incrementButton;
    private FTCButton decrementButton;
    private int cumulativeClicks = 0;

    private static final int CLICKS_PER_MOVEMENT = 100;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {

        // Set up for RUN_TO_POSITION and hold
        elevator = robot.elevator;
        dualMotorMotion = new DualMotorMotion(this, elevator);

        incrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y); // increment
        decrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A); // decrement

        telemetry.setAutoClear(true);
        telemetry.addData("Direction", "Gampepad 1: Y = up; A = down");
        telemetry.addData("-----", "-----");
        telemetry.update();
    }

    @Override
    public void run() {
        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();
        }
    }

    private void updateButtons() {
        incrementButton.update();
        decrementButton.update();
    }

    private void updatePlayerOne() {
        updateIncrement();
        updateDecrement();
    }

    private void updateIncrement() {
        if (incrementButton.is(FTCButton.State.TAP)) {
            dualMotorMotion.moveDualMotors(cumulativeClicks += CLICKS_PER_MOVEMENT, elevator.getVelocity(),
                    DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            updateEncoderTelemetry();
        }
    }

    private void updateDecrement() {
        if (decrementButton.is(FTCButton.State.TAP)) {
            dualMotorMotion.moveDualMotors(cumulativeClicks -= CLICKS_PER_MOVEMENT, elevator.getVelocity(), DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            updateEncoderTelemetry();
        }
    }

    private void updateEncoderTelemetry() {
        telemetry.addData("current left", elevator.getCurrentPosition(FTCRobot.MotorId.ELEVATOR_LEFT));
        telemetry.addData("current right", elevator.getCurrentPosition(FTCRobot.MotorId.ELEVATOR_RIGHT));
        telemetry.update();
    }

}