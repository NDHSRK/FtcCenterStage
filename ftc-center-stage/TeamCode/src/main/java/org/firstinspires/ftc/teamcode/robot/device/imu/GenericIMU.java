package org.firstinspires.ftc.teamcode.robot.device.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class GenericIMU {
    private static final String TAG = GenericIMU.class.getSimpleName();

    private IMU imu;

    public GenericIMU(HardwareMap pHardwareMap, XPathAccess pXPath) throws InterruptedException, XPathExpressionException {
        //# This initialization code is derived from the sample SensorIMUOrthogonal.
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = pHardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        RobotLogCommon.c(TAG, "IMU configuration from RobotConfig.xml");
        String logoFacingString = pXPath.getRequiredText("logo_facing_direction");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(logoFacingString);
        RobotLogCommon.d(TAG, "IMU logo facing direction " + logoFacingString);

        String usbFacingString = pXPath.getRequiredText("usb_facing_direction");
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(usbFacingString);
        RobotLogCommon.d(TAG, "IMU USB facing direction " + usbFacingString);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // necessary because sometimes the yaw carries over after a restart
    }

    public IMU getImu() {
        return imu;
    }

}
