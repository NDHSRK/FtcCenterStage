package org.firstinspires.ftc.teamcode.teleop.common;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.function.Function;

// Use this dispatcher class to pass the Alliance to instances of TeleOp
// that need it, for example, to set the direction of the carousel motor
// in Freight Frenzy.
// The dispatcher also supports creation of an instance of FTCAuto in case
// there's a need to run Autonomous OpModes in TeleOp for fixed sequences
// of actions.
public class FTCTeleOpDispatch {

    // Here's how this works: the caller of this function, e.g. TeleOpTakePictureLaunch,
    // needs to instantiate its worker class, e.g. TeleOpTakePicture, but the driver has
    // just selected the OpMode so two crucial classes have not yet been instantiated:
    // FTCRobot (for access to the hardware configuration) and, optionally, FTCAuto (for
    // the ability to run autonomous OpModes in TeleOp). So the instantiation of the
    // worker class needs to be deferred. The Java Function interface gives us a way of
    // doing this. The caller is saying "please construct my worker class with arguments
    // that are created here in runTeleOp". Worker classes are generic in that they must
    // implement the TeleOpWithAlliance interface. The arguments to the constructor are
    // packaged in a class, TeleOpWithAllianceParameters, to meet the requirements of the
    // Function<T,R> interface (T = single input parameter to the function, R = return
    // value from the function).
    public static void runTeleOp(RobotConstants.RunType pRunType,
                                 String pTeleOpClass,
                                 RobotConstants.Alliance pAlliance,
                                 LinearOpMode pLinear,
                                 Function<TeleOpWithAllianceParameters, TeleOpWithAlliance> pCreateTeleOpWithAlliance) throws InterruptedException {
        final String TAG = FTCTeleOpDispatch.class.getSimpleName();
        pLinear.telemetry.setAutoClear(false); // keep our messages on the driver station

        // All TeleOp OpModes need access to the public data fields and methods in LinearOpMode.
        try {
            RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.TELEOP_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

            RobotLogCommon.c(TAG, "Preparing to run TeleOp with run type " + pRunType + " from class " + pTeleOpClass);
            RobotLogCommon.c(TAG, "Constructing FTCRobot with run type " + pRunType);
            FTCRobot robot = new FTCRobot(pLinear, pRunType);

            // Support running Autonomous within TeleOp.
            // Construct FTCAuto here to avoid lengthy construction time after "start".
            FTCAuto ftcAuto = null;
            if (pRunType == RobotConstants.RunType.TELEOP_WITH_EMBEDDED_AUTONOMOUS) {
                RobotLogCommon.c(TAG, "Constructing FTCAuto to be embedded within TeleOp");
                ftcAuto = new FTCAuto(pAlliance, pLinear, robot, pRunType);
            } else
                RobotLogCommon.c(TAG, "Running TeleOp without embedded FTCAuto");

            RobotLogCommon.c(TAG, "Constructing TeleOpWithAlliance with alliance " + pAlliance);
            TeleOpWithAlliance teleOpWithAlliance = pCreateTeleOpWithAlliance.apply(new TeleOpWithAllianceParameters(pAlliance, pLinear, robot, ftcAuto));

            pLinear.telemetry.addData(TAG, "Waiting for start ...");
            pLinear.telemetry.update();

            //?? Where does control go if there's a crash in init?
            // See comments in TeleOpBase.
            pLinear.waitForStart();

            pLinear.telemetry.addData(TAG, "Running ...");
            pLinear.telemetry.update();

            RobotLogCommon.c(TAG, "Running runTeleOp in class " + pTeleOpClass);
            teleOpWithAlliance.runTeleOp();
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, pLinear);
        } finally {
            RobotLogCommon.closeLog();
        }
    }

    public static class TeleOpWithAllianceParameters {
        public final RobotConstants.Alliance alliance;
        public final LinearOpMode linearOpMode;
        public final FTCRobot robot;
        @Nullable
        public final FTCAuto ftcAuto;

        public TeleOpWithAllianceParameters(RobotConstants.Alliance pAlliance,
                                            LinearOpMode pLinearOpMode, FTCRobot pRobot,
                                            @Nullable FTCAuto pAutonomous) {
            alliance = pAlliance;
            linearOpMode = pLinearOpMode;
            robot = pRobot;
            ftcAuto = pAutonomous;
        }
    }

}
