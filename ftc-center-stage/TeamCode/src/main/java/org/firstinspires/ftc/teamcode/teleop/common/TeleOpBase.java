package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

// Use this class in TeleOp test routines.
// Note that the runOpMode method below initializes the robot hardware.
public abstract class TeleOpBase extends LinearOpMode {

    private static final String TAG = "TeleOpBase";
    protected FTCRobot robot;

    // Requires inheritors of this class to define a run type, e.g.
    // TELEOP if you want to enable the driver train or TELEOP_NO_DRIVE
    // if you want to test a device such as a servo in isolation.
    protected abstract RobotConstants.RunType getRunType();

    protected abstract void initialize() throws InterruptedException;

    protected abstract void run() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing...", "Please wait until complete");
        telemetry.update();

        try {
            RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.TELEOP_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

            robot = new FTCRobot(this, getRunType());
            initialize(); // give derived class a chance to initialize

            telemetry.addData("Initialized!", "Ready to run");
            telemetry.update();

            //?? Where does control go if there's a crash in init?
            // The error message is written to the Driver Station
            // but it is quickly replaced by this message:
            // "The OpMode which was just initialized ended prematurely ...
            // Did you forget to call waitForStart()?"
            // The error message may not be written to the FTCTeleOpLog
            // (not enough time?) but it is written to the match log, e.g.
            // 12-27 16:20:43.644  1673  1856 E TeleOpBase:  ** FATAL Java Exception ** Unable to find a hardware device with name "drone_launcher" and type Servo

            waitForStart();
            run(); // run derived class
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, this);
        } finally {
            RobotLogCommon.closeLog();
        }
    }

}
