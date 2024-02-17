/*
From https://docs.revrobotics.com/magnetic-limit-switch/application-examplesApplication Examples

The REV Magnetic Limit Switch comes with two mountable magnets. Because this sensor does not require a contact interface,
the magnet can also be soft mounted almost anywhere with just tape or glue.
The strength of the magnetic field determines the maximum distance the magnet can be from the sensor and still be detected.
Alternate (stronger or weaker) magnets can easily be used to change the trigger range of this sensor.

Hysteresis
When designing a system using the REV Magnetic Limit Switch it is important to consider in the impact of hysteresis.
When the magnetic field approaches the Magnetic Limit Switch, after the field strength increases enough that it crosses
the rising trigger point (Bop) the sensor triggers. As the magnet is then moved away from the sensor, the magnetic field
strength falls but the sensor remains in the triggered state until the field falls below the falling trigger level (BRP).
The difference between these two points is the hysteresis.
For a simple system like stopping an arm at the end of range of motion, the hysteresis might not play much of a role, but
for creating one or more stop points on a linear elevator, this may factor into the software design.

FTC Applications 
Configuring in the Control System 
The Magnetic Limit Switch can be configured as "REV Touch Sensor" or as "Digital Device" as shown in the image below. 
In this example, the Touch Sensor is configured on port 2. It is touched on briefly in the 
that the Magnetic Limit Switch is capable of sending a signal to the Control Hub through the n+1 and n communication
channels. The channel the sensor communicates through is decided by which port it is configured on. In this case, the
Magnetic Limit Switch communicates through the n channel.

Programming Applications 
The code blocks below gives a basic example of how to use the Magnetic Limit Switch to limit the motion range of a
motor using if/else logic. If the magnet is within range of the sensor then the motor stops. Otherwise the motor is
allowed to move. When triggered by proximity to a magnet, the sensor is considered pressed. 
The code assumes the sensor has been named "Limit" and the motor has been named "Motor" in configuration.
*/
package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;

@TeleOp(name = "Find Backdrop AprilTag", group = "Test")
@Disabled
public class FindBackdropAprilTag extends LinearOpMode {
    @Override
    public void runOpMode() {
        BackdropAprilTag backdropAprilTag = new BackdropAprilTag(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        //**TODO Assuming that the output of your Team Prop recognition is an integer
        // from 1 to 6, first convert it to an enumeration value for the AprilTags:
        AprilTagUtils.AprilTagId aprilTagEnum = AprilTagUtils.AprilTagId.getEnumValue(2);

        //**TODO Then call the following method to position your robot in front of
        // the identified AprilTag for delivery of the yellow pixel. The third
        // parameter will always be the same - in your case the camera is on the
        // front of your robot.
        backdropAprilTag.driveToBackdropAprilTag(aprilTagEnum,
                2.0, BackdropAprilTag.Direction.FORWARD);
    }

}