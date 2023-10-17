package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrain;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// In TeleOp run the drive train in a separate thread so that
// other Gamepad-controlled actions (such as servo movements)
// do not make the drive train unresponsive, i.e. unable to
// start, or worse, unable to stop.
public class ParallelDrive {

    private static final String TAG = ParallelDrive.class.getSimpleName();

    private final LinearOpMode linear;
    private final DriveTrain driveTrain;
    private boolean driveTrainActivated = false;

    // Thread-related.
    private CountDownLatch countDownLatch;
    private DriveTrainCallable driveTrainCallable;
    private CompletableFuture<Void> driveTrainFuture;

    public final Lock driveLock = new ReentrantLock();
    private final AtomicReference<Double> velocity = new AtomicReference<>();

    public ParallelDrive(LinearOpMode pLinear, DriveTrain pDriveTrain, double pInitialVelocity) {
        linear = pLinear;
        driveTrain = pDriveTrain;
        velocity.set(pInitialVelocity);
    }

    // Start the drive train thread.
    //!!## WARNING: call this method ONLY when the linearOpMode
    // is active, i.e after waitUntilStart() returns.
    public void startDriveTrain() throws InterruptedException {
        if (driveTrainActivated)
            return; // nothing to do
        driveTrainActivated = true;

        // Start up the drive train as a CompletableFuture.
        RobotLogCommon.i(TAG, "Starting drive train thread");

        countDownLatch = new CountDownLatch(1);
        driveTrainCallable = new DriveTrainCallable();
        driveTrainFuture = Threading.launchAsync(driveTrainCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
        RobotLogCommon.i(TAG, "Wait for CountDownLatch done; drive train thread is running");
    }

    // Turn off when done with the drive train.
    public void stopDriveTrain() throws IOException, InterruptedException, TimeoutException {
        if (!driveTrainActivated)
            return; // nothing to do
        driveTrainActivated = false;

        // Stop the drive train thread and wait for it to complete.
        driveTrainCallable.stopThread();
        Threading.getFutureCompletion(driveTrainFuture);
    }

    public void setVelocity(double pVelocity) {
        velocity.set(pVelocity);
    }

    // Activates the drive train according to buttons on Gamepad 1.
    private class DriveTrainCallable extends AutoWorker<Void> {

        private boolean robotHasMoved = false;

        DriveTrainCallable() {
            super();
        }

        public Void call() {
            RobotLogCommon.i(TAG, "In drive train thread");
            countDownLatch.countDown(); // signal that I'm running

            while (linear.opModeIsActive() && !stopThreadRequested()) {
                double currentVelocityFactor = velocity.get();
                // up on the gamepad stick is negative
                // change to straight power from Math.pow(gamepad1..., 3);
                double directionX = linear.gamepad1.left_stick_x;
                double directionY = -linear.gamepad1.left_stick_y;
                double directionR = linear.gamepad1.right_stick_x;

                double lfv = MotionUtils.clip(directionY + directionR + directionX, 0.0);
                double rfv = MotionUtils.clip(directionY - directionR - directionX, 0.0);
                double lbv = MotionUtils.clip(directionY + directionR - directionX, 0.0);
                double rbv = MotionUtils.clip(directionY - directionR + directionX, 0.0);

                // If the sticks are idle but the robot has previously moved,
                // set the velocity to 0 once. There's no reason to repeatedly
                // set 0 velocity.
                if (lfv == 0.0 && rfv == 0.0 && lbv == 0.0 && rbv == 0.0) {
                    if (robotHasMoved) {
                        robotHasMoved = false;
                        driveTrain.driveAllByVelocity(lfv * currentVelocityFactor, rfv * currentVelocityFactor, lbv * currentVelocityFactor, rbv * currentVelocityFactor);
                    }
                }
                else {
                    // The driver has manipulated the stick(s) but a non-interruptible
                    // operation may be in process; do not move the robot.
                    if (driveLock.tryLock()) {
                        try {
                            robotHasMoved = true;
                            driveTrain.driveAllByVelocity(lfv * currentVelocityFactor, rfv * currentVelocityFactor, lbv * currentVelocityFactor, rbv * currentVelocityFactor);
                        } finally {
                            driveLock.unlock();
                        }
                    }
                }
            }

            return null;
        }
    }

}
