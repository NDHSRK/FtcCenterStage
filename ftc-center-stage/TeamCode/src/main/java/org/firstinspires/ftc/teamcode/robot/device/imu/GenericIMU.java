package org.firstinspires.ftc.teamcode.robot.device.imu;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class GenericIMU {
    private static final String TAG = GenericIMU.class.getSimpleName();

    private IMU controlHubIMU;
    private IMU expansionHubIMU; // optional, may be null

    public GenericIMU(HardwareMap pHardwareMap, XPathAccess pXPath) throws InterruptedException, XPathExpressionException {
        //# This initialization code is derived from the sample SensorIMUOrthogonal.
        // Retrieve and initialize the Control Hub IMU.

        // 12/17/2023 Because the IMU on the RobotController started returning a heading
        // of -0.0 during a competition match, we'll set up optional access to the IMU
        // on the Expansion Hub.

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        RobotLogCommon.c(TAG, "Control Hub IMU configuration from RobotConfig.xml");
        String controlHubIMULogoFacingString = pXPath.getRequiredText("control_hub/logo_facing_direction");
        RevHubOrientationOnRobot.LogoFacingDirection controlHubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(controlHubIMULogoFacingString);
        RobotLogCommon.d(TAG, "Control Hub IMU logo facing direction " + controlHubIMULogoFacingString);

        String controlHuUsbFacingString = pXPath.getRequiredText("control_hub/usb_facing_direction");
        RevHubOrientationOnRobot.UsbFacingDirection controlHubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(controlHuUsbFacingString);
        RobotLogCommon.d(TAG, "Control Hub IMU USB facing direction " + controlHuUsbFacingString);

        RevHubOrientationOnRobot controlHubIMUOrientation = new RevHubOrientationOnRobot(controlHubLogoDirection, controlHubUsbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        controlHubIMU = pHardwareMap.get(IMU.class, "imu");
        controlHubIMU.initialize(new IMU.Parameters(controlHubIMUOrientation));
        controlHubIMU.resetYaw(); // necessary because sometimes the yaw carries over after a restart

        // The Expansion Hub IMU is optional.
        String expansionHubIMULogoFacingString = pXPath.getText("expansion_hub/logo_facing_direction", "NONE");
        if (!expansionHubIMULogoFacingString.equals("NONE")) {
            RevHubOrientationOnRobot.LogoFacingDirection expansionHubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(expansionHubIMULogoFacingString);
            RobotLogCommon.d(TAG, "Expansion Hub IMU logo facing direction " + expansionHubIMULogoFacingString);

            String expansionHubUsbFacingString = pXPath.getRequiredText("expansion_hub/usb_facing_direction");
            RevHubOrientationOnRobot.UsbFacingDirection expansionHubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(expansionHubUsbFacingString);
            RobotLogCommon.d(TAG, "Expansion Hub IMU USB facing direction " + expansionHubUsbFacingString);

            RevHubOrientationOnRobot expansionHubIMUOrientation = new RevHubOrientationOnRobot(expansionHubLogoDirection, expansionHubUsbDirection);

            // Now initialize the IMU with this mounting orientation
            // Note: if you choose two conflicting directions, this initialization will cause a code exception.
            expansionHubIMU = pHardwareMap.get(IMU.class, "imu_exp");
            expansionHubIMU.initialize(new IMU.Parameters(expansionHubIMUOrientation));
            expansionHubIMU.resetYaw(); // necessary because sometimes the yaw carries over after a restart
        }
    }

    public IMU getControlHubImu() {
        return controlHubIMU;
    }

    public IMU getExpansionHubImu() {
        return expansionHubIMU;
    }

}
