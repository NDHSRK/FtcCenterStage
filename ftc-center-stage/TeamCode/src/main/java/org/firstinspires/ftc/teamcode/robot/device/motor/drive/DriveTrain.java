package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotorCore;

import java.util.EnumMap;

import javax.xml.xpath.XPathExpressionException;

// Robot Drive Train
public class DriveTrain extends MotorCore {

    public static final String TAG = "DriveTrain";

    private final double clicksPerInch;

    public DriveTrain(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pConfigXPath, "motors");

        // Get the drive train configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the robot's drive train");
        RobotLogCommon.c(TAG, "Motor(s) " + pConfigXPath.getRequiredText("motors/@model"));

        DcMotorEx lf = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("motors/left_front_drive"));
        DcMotorEx rf = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("motors/right_front_drive"));
        DcMotorEx lb = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("motors/left_back_drive"));
        DcMotorEx rb = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("motors/right_back_drive"));

        // Set the direction of each motor.
        lf.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("motors/left_front_drive/@direction")));
        rf.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("motors/right_front_drive/@direction")));
        lb.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("motors/left_back_drive/@direction")));
        rb.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("motors/right_back_drive/@direction")));

        motorMap = new EnumMap<FTCRobot.MotorId, DcMotorEx>(FTCRobot.MotorId.class) {{
            put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, lf);
            put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, rf);
            put(FTCRobot.MotorId.LEFT_BACK_DRIVE, lb);
            put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, rb);
        }};

        // Set the default run mode for the drive train.
        /*
            From https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
            In RUN_USING_ENCODER mode, you should set a velocity (measured in ticks per second),
            rather than a power level. You can still provide a power level in RUN_USING_ENCODER
            mode, but this is not recommended, as it will limit your target speed significantly.
            Setting a velocity from RUN_WITHOUT_ENCODER mode will automatically switch the motor
            to RUN_USING_ENCODER mode.
        */
        //## Note that with either of the run modes RUN_USING_ENCODER or RUN_TO_POSITION,
        // setPower has no effect!!
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBrakeAll();

        double wheelDiameterIn = pConfigXPath.getRequiredDouble("motors/@wheel_diameter_in");
        clicksPerInch = getClicksPerMotorRev() / (wheelDiameterIn * Math.PI);
    }

    public double getClicksPerInch() {
        return clicksPerInch;
    }

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    public void driveAllByVelocity(double lfVelocity, double rfVelocity,
                                   double lbVelocity, double rbVelocity) {
        EnumMap<FTCRobot.MotorId, Double> velocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        velocityMap.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, lfVelocity);
        velocityMap.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, rfVelocity);
        velocityMap.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, lbVelocity);
        velocityMap.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, rbVelocity);

        setVelocityAll(velocityMap);
    }

    public void driveAllByPower(double pLeftFrontPower, double pRightFrontPower,
                                double pLeftBackPower, double pRightBackPower) {
        EnumMap<FTCRobot.MotorId, Double> powerMap = new EnumMap<>(FTCRobot.MotorId.class);
        powerMap.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, pLeftFrontPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, pRightFrontPower);
        powerMap.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, pLeftBackPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, pRightBackPower);

        setPowerAll(powerMap);
    }

}