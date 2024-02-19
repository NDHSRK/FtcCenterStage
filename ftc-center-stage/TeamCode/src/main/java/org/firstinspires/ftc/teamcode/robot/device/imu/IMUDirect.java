package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
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
    private boolean useControlHubIMU = false; //**TODO 2/12/24 switch to expansion hub IMU
    private boolean logFirstExpansionHubIMUHeading = true;

    // The IMU must have been previously initialized.
    public IMUDirect(IMU pControlHubIMU, IMU pExpansionHubIMU) {
        controlHubIMU = pControlHubIMU;
        expansionHubIMU = pExpansionHubIMU;

        // This can happen during testing with a robot that does not have an
        // Expansion Hub.
        if (expansionHubIMU == null && !useControlHubIMU) {
            RobotLogCommon.d(TAG, "Flag set to use expansion hub IMU but it is not in the configuration");
            RobotLogCommon.d(TAG, "Switching to use the Control Hub IMU");
            useControlHubIMU = true;
        }
    }

    public void resetIMUYaw() {
        if (useControlHubIMU) {
            controlHubIMU.resetYaw(); // necessary because sometimes the yaw carries over after a restart
            angles = controlHubIMU.getRobotYawPitchRollAngles();
            heading = angles.getYaw(DEGREES);
            RobotLogCommon.d(TAG, "Control Hub IMU after reset " + heading);
        }
        else {
            expansionHubIMU.resetYaw();
            angles = expansionHubIMU.getRobotYawPitchRollAngles();
            heading = angles.getYaw(DEGREES);
            RobotLogCommon.d(TAG, "Expansion Hub IMU after reset " + heading);
        }
    }

    // Switch to Expansion Hub IMU (if present) if the Control Hub
    // IMU returns a heading of -0.0. Log the change.
    //**TODO 2/12/2024 Monitor - in a scrimmage on 2/10/24 the Control
    // HUB IMU unexpectedly returned -180.0 and only a short time later
    // returned -0.0.
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
                    throw new AutonomousRobotException(TAG, "Control Hub IMU returned heading -0.0 but Expansion Hub IMU not present");
            } else return heading;
        }

        if (!useControlHubIMU) {
            angles = expansionHubIMU.getRobotYawPitchRollAngles();
            heading = angles.getYaw(DEGREES);
            if (logFirstExpansionHubIMUHeading) {
                logFirstExpansionHubIMUHeading = false;
                RobotLogCommon.d(TAG, "First heading from the Expansion Hub IMU " + heading);
            }
        }

        return heading;
    }

    //&& Switch to Expansion Hub IMU (if present) - pitch
    public double getIMUPitch() {
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        return angles.getPitch(DEGREES);
    }

    //&& Switch to Expansion Hub IMU (if present) - roll
    public double getIMURoll() {
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        return angles.getRoll(DEGREES);
    }

}
