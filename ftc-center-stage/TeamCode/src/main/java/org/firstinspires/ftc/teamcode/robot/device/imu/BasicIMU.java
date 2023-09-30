package org.firstinspires.ftc.teamcode.robot.device.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicIMU {

    private static final String TAG = BasicIMU.class.getSimpleName();

    private final BNO055IMU initializedIMU;

    public BasicIMU(HardwareMap pHardwareMap) throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //      parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //      parameters.loggingEnabled = true;
        //      parameters.loggingTag = "IMU";

        // Vanilla use of IMU - not optimized.
        initializedIMU = pHardwareMap.get(BNO055IMU.class, "imu");
        initializedIMU.initialize(parameters);
    }

    public BNO055IMU getInitializedIMU() {
        return initializedIMU;
    }

}
