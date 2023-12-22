package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Direct synchronous access to the IMU. This class is an alternative
// to the IMUReader, which continuously the IMU on a separate thread.
//## Created this class because of unexplained and irregular turning
// errors in 11/04/2023 Meet 0. Updated this class 12/22/2023 to
// detect when the Control Hub IMU returns -0.0 (as in Meet 3) and to
// switch to the Expansion Hub, if present.
// See https://stackoverflow.com/questions/6724031/how-can-a-primitive-float-value-be-0-0-what-does-that-mean
public class IMUDirect {
    private static final String TAG = IMUDirect.class.getSimpleName();
    private final IMU controlHubIMU;
    private final IMU expansionHubIMU; // optional, may be null
    private YawPitchRollAngles angles;
    private double heading;
    private boolean useControlHubIMU = true;
    private boolean logFirstExpansionHubIMUHeading = true;

    // The IMU must have been previously initialized.
    public IMUDirect(IMU pControlHubIMU, IMU pExpansionHubIMU) {
        controlHubIMU = pControlHubIMU;
        expansionHubIMU = pExpansionHubIMU;
    }

    public void resetIMUYaw() {
        controlHubIMU.resetYaw(); // necessary because sometimes the yaw carries over after a restart
        if (expansionHubIMU != null)
            expansionHubIMU.resetYaw();
    }

    // Switch to Expansion Hub IMU (if present) if the Control Hub
    // IMU returns a heading of -0.0. Log the change.
    public double getIMUHeading() {
        if (useControlHubIMU) {
            angles = controlHubIMU.getRobotYawPitchRollAngles();
            heading = angles.getYaw(DEGREES);
            if (heading == -0.0) {
                useControlHubIMU = false;
                RobotLogCommon.d(TAG, "Control Hub IMU returned heading of -0.0");
                if (expansionHubIMU != null)
                    RobotLogCommon.c(TAG, "Switching to the Expansion Hub IMU");
                else
                    RobotLogCommon.c(TAG, "Expansion Hub IMU not present");
            } else return heading;
        }

        if (!useControlHubIMU && expansionHubIMU != null) {
            angles = expansionHubIMU.getRobotYawPitchRollAngles();
            heading = angles.getYaw(DEGREES);
            if (logFirstExpansionHubIMUHeading) {
                logFirstExpansionHubIMUHeading = false;
                RobotLogCommon.d(TAG, "First heading from the Expansion Hub IMU " + heading);
            }
        }

        return heading;
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
