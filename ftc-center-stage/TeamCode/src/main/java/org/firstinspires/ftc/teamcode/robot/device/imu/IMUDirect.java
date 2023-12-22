package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Direct synchronous access to the IMU. This class is an alternative
// to the IMUReader, which continuously the IMU on a separate thread.
//## Created this class because of unexplained and irregular turning
// errors in 11/04/2023 Meet 0.
public class IMUDirect {
    private final IMU controlHubIMU;
    private final IMU expansionHubIMU; // optional, may be null

    // The IMU must have been previously initialized.
    public IMUDirect(IMU pControlHubIMU, IMU pExpansionHubIMU) {
        controlHubIMU = pControlHubIMU;
        expansionHubIMU = pExpansionHubIMU;
    }

    public void resetIMUYaw() {
        controlHubIMU.resetYaw(); // necessary because sometimes the yaw carries over after a restart
        if (expansionHubIMU != null)
            controlHubIMU.resetYaw();
    }

    //**TODO Switch to Expansion Hub IMU (if present) if the Control Hub
    // IMU returns a heading of -0.0.
    public double getIMUHeading() {
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        return angles.getYaw(DEGREES);
    }

    //**TODO Switch to Expansion Hub IMU (if present) - pitch
    public double getIMUPitch() {
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        return angles.getPitch(DEGREES);
    }

    //**TODO Switch to Expansion Hub IMU (if present) - roll
    public double getIMURoll() {
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        return angles.getRoll(DEGREES);
    }

}
