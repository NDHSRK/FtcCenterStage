package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Direct synchronous access to the IMU. This class is an alternative
// to the IMUReader, which continuously the IMU on a separate thread.
//## Created this class because of unexplained and irregular turning
// errors in 11/04/2023 Meet 0.
public class IMUDirect {
    private final IMU imu;

    // The IMU must have been previously initialized.
    public IMUDirect(IMU pInitializedIMU) {
        imu = pInitializedIMU;
    }

    public void resetIMUYaw() {
        imu.resetYaw(); // necessary because sometimes the yaw carries over after a restart
    }

    public double getIMUHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(DEGREES);
    }

    public double getIMUPitch() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getPitch(DEGREES);
    }

    public double getIMURoll() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getRoll(DEGREES);
    }

}
