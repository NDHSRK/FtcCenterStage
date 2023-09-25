package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;

// Common error handling for Autonomous and TeleOp.
public class FTCErrorHandling {

    // Catch clauses for FTC Autonomous and TeleOp ---

    // Keep this comment here as a warning!
    // You can't call System.exit(1); because the robot controller then exits and
    // can't be restarted from the driver station. The result is that if you get
    // a fatal error during the autonomous run and exit you can't restart the robot
    // for the driver-controlled run.

    // NOTE: from the ftc documentation - "Please do not swallow the InterruptedException,
    // as it is used in cases where the op mode needs to be terminated early."
    // See also https://stackoverflow.com/questions/3590000/what-does-java-lang-thread-interrupt-do
    // which raises questions about how this works in FTC.

    // 12/31/21 By the time you get here the linearOpMode may not be active
    // and/or there may not be enough time to write to our Java log. So in
    // desperation write to the Android log; miraculously, the FTC runtime
    // includes these log entries in its own logs.

    public static void handleFtcErrors(Exception ex, String pTag, LinearOpMode pLinear) throws InterruptedException {
        if (ex instanceof InterruptedException) {
            RobotLogCommon.e(pTag, "** FATAL ** caught InterruptedException; rethrowing");
            Log.e(pTag,  "** FATAL ** caught InterruptedException; rethrowing"); // write to the FTC log
            throw (InterruptedException) ex;
        }

        // For all other exceptions don't **ever** do this in Autonomous:
        // throw(arx); // propagate error
        // - rethrowing shuts down the entire application and prevents
        // TeleOp from starting.

        pLinear.telemetry.setAutoClear(true);
        String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
        if (ex instanceof AutonomousRobotException) {
            RobotLogCommon.e(((AutonomousRobotException) ex).getTag(), " ** FATAL AutonomousRobotException ** " + eMessage);
            Log.e(((AutonomousRobotException) ex).getTag(), " ** FATAL ** " + eMessage);
            holdErrorMessageOnScreen(pLinear, pTag + " Fatal AutonomousRobotException", eMessage);
        } else {
            // Must be an Exception
            RobotLogCommon.e(pTag, " ** FATAL Java Exception ** " + eMessage);
            Log.e(pTag, " ** FATAL Java Exception ** " + eMessage);
            holdErrorMessageOnScreen(pLinear,pTag + " Fatal Java Exception", eMessage);
        }
    }

    private static void holdErrorMessageOnScreen(LinearOpMode pLinear, String pCaption, String pMessage) {
        ElapsedTime errorMessageHoldTime = new ElapsedTime();
        pLinear.telemetry.log().clear();
        pLinear.telemetry.clearAll();

        // The loop is insurance against the FTC runtime timing us out.
        errorMessageHoldTime.reset();
        while (errorMessageHoldTime.seconds() < 3) {
            pLinear.telemetry.addData(pCaption, pMessage);
            pLinear.telemetry.update();
        }
    }

}