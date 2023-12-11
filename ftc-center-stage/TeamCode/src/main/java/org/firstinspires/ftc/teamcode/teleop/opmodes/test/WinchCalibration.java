package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotor;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

@TeleOp(name = "WinchCalibration", group = "Test")
//@Disabled
public class WinchCalibration extends TeleOpBase {

    private SingleMotor winchMotor;
    private SingleMotorMotion winchMotorMotion;

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

        //!! The motor you want to test must be configured in via RobotConfig.xml.
        //!! Get a reference to it from FTCRobot, e.g. singleMotor = robot.boom.
        winchMotor = robot.winch;

        //!! The motor you want to test must be configured in via RobotConfig.xml.
        //!! Get a reference to it from FTCRobot, e.g. singleMotor = robot.boom.

        // Set up for RUN_TO_POSITION and hold
        winchMotorMotion = new SingleMotorMotion(this, winchMotor);

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
            winchMotorMotion.moveSingleMotor(cumulativeClicks += CLICKS_PER_MOVEMENT, winchMotor.getVelocity(),
                    SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            updateEncoderTelemetry();
        }
    }

    private void updateDecrement() {
        if (decrementButton.is(FTCButton.State.TAP)) {
            winchMotorMotion.moveSingleMotor(cumulativeClicks -= CLICKS_PER_MOVEMENT, winchMotor.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            updateEncoderTelemetry();
        }
    }

    //!! Change FTCRobot.MotorId.MOTOR_ID_NPOS to the enum value of the device under test,
    // e.g. FTCRobot.MotorId.BOOM.
    private void updateEncoderTelemetry() {
        telemetry.addData("current", winchMotor.getCurrentPosition(FTCRobot.MotorId.WINCH));
        telemetry.update();
    }

}