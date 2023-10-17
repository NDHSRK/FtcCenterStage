package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrain;

public class DriveStick {

    private final LinearOpMode linear;
    private final DriveTrain driveTrain;

    public DriveStick(LinearOpMode pLinear, DriveTrain pDriveTrain) {
        linear = pLinear;
        driveTrain = pDriveTrain;
    }

    public void updateDrive(double pVelocityFactor) {
        // up on the gamepad stick is negative
        double directionX = linear.gamepad1.left_stick_x;
        double directionY = -linear.gamepad1.left_stick_y;
        double directionR = linear.gamepad1.right_stick_x;

        double lfv = MotionUtils.clip(directionY + directionR + directionX, 0.0);
        double rfv = MotionUtils.clip(directionY - directionR - directionX, 0.0);
        double lbv = MotionUtils.clip(directionY + directionR - directionX, 0.0);
        double rbv = MotionUtils.clip(directionY - directionR + directionX, 0.0);
        driveTrain.driveAllByVelocity(lfv * pVelocityFactor, rfv * pVelocityFactor, lbv * pVelocityFactor, rbv * pVelocityFactor);
    }

}