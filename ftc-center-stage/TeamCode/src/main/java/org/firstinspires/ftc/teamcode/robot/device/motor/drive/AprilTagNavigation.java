/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.EnumMap;
import java.util.List;

// Adapted from the FTC SDK 9.0 sample OpMode RobotAutoDriveToAprilTagOmni.
public class AprilTagNavigation {
    private static final String TAG = AprilTagNavigation.class.getSimpleName();

    // Set the GAIN constants to control the relationship between the measured
    // position error, and how much power is
    // applied to the drive motors to correct the error.
    // Drive = Error * Gain Make these values smaller for smoother control, or
    // larger for a more aggressive response.
    final double SPEED_GAIN = 0.02; // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.
    // (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain". eg: Ramp up to 25% power at a 25 degree Yaw
    // error. (0.25 / 25.0)
    final double TURN_GAIN = 0.01; // Turn Control "Gain". eg: Ramp up to 25% power at a 25 degree error. (0.25 /
    // 25.0)

    final double MAX_AUTO_SPEED = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5; // Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3; // Clip the turn speed to this max value (adjust for your robot)

    private final RobotConstants.Alliance alliance;
    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final VisionPortalWebcam webcam;
    private final EnumMap<FTCRobot.MotorId, Double> powerMap = new EnumMap<>(FTCRobot.MotorId.class);

    public AprilTagNavigation(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                              VisionPortalWebcam pWebcam) {
        alliance = pAlliance;
        linearOpMode = pLinearOpMode;
        robot = pRobot; // robot hardware
        webcam = pWebcam;

        //**TODO This should be called from FTCAuto AFTER Team Prop recognition.
        webcam.setManualExposure(6, 250, 1000); // Use low exposure time to reduce motion blur
    }

    public boolean driveToAprilTag(int pDesiredTagId, double pDesiredDistanceFromTag, DriveTrainConstants.Direction pDirection) {
        double drive = 0; // Desired forward power/speed (-1 to +1)
        double strafe = 0; // Desired strafe power/speed (-1 to +1)
        double turn = 0; // Desired turning power/speed (-1 to +1)

        // Step through the list of detected tags and look for a matching tag.
        AprilTagDetection desiredTag = null;
        List<AprilTagDetection> currentDetections = webcam.getAprilTagData(1000);
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == pDesiredTagId) {
                desiredTag = detection;
                break; // don't look any further.
            }
        }

        //**TODO Need logging in addition to or instead of telemetry

        // If we have found the desired target, drive to target automatically.
        if (desiredTag != null) {
            linearOpMode.telemetry.addLine("Tag Id " + pDesiredTagId + " not found within 1 sec");
            linearOpMode.telemetry.update();
            return false;
        }

        linearOpMode.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
        linearOpMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
        linearOpMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
        linearOpMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

        //**TODO the direction of the robot affects the signs of some of these values
        // Currently set to forward only ...

        // Determine heading, range and Yaw (tag image rotation) error so we can use
        // them to control the robot automatically.
        double rangeError = (desiredTag.ftcPose.range - pDesiredDistanceFromTag);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        linearOpMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        linearOpMode.telemetry.update();

        // **TODO Need means of preventing a stall ...
        // Apply desired axes of motion to the drivetrain.
        while (Math.abs(rangeError) > 0 || Math.abs(headingError) > 0 || Math.abs(yawError) > 0) {
            moveRobot(drive, strafe, turn);
            sleep(20);
        }

        return true;
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    // **TODO Make sure the motors are in the right mode ...
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send power to the wheels.
        powerMap.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, leftFrontPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, rightFrontPower);
        powerMap.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, leftBackPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, rightBackPower);

        robot.driveTrain.setPowerAll(powerMap);
        powerMap.clear();
    }

}
