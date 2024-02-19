package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;

// Class that starts up a CompletableFuture whose Callable gets the amount
// of time used in the Autonomous period and posts a panic stop if the time
// used exceeds our safe shutdown limit.
public class AutonomousTimer {

    private static final String TAG = AutonomousTimer.class.getSimpleName();

    private static double SAFE_SHUTDOWN_LIMIT = 26.0; // seconds

    private final LinearOpMode linearOpMode;
    private boolean isRunning = true;

    // Thread-related.
    private final AutoTimerCallable autoTimerCallable;
    private final CompletableFuture<Void> autoTimerFuture;

    private final AtomicBoolean panicStop = new AtomicBoolean();

    public AutonomousTimer(LinearOpMode pLinear) throws InterruptedException {
        linearOpMode = pLinear;

        // Start up the AutonomousTimer as a CompletableFuture.
        RobotLogCommon.i(TAG, "Starting AutonomousTimer thread");

        autoTimerCallable = new AutoTimerCallable();
        autoTimerFuture = Threading.launchAsync(autoTimerCallable);
    }

    public boolean autoTimerIsExpired() {
        return panicStop.get();
    }

    // To be called from the finally block of FTCAuto.
    public void stopAutonomousTimer() throws IOException, InterruptedException, TimeoutException {
        panicStop.set(false);

        if (!isRunning) // already stopped?
            return;

        isRunning = false;
        autoTimerCallable.stopThread(); // Force stop now.
        autoTimerFuture.complete(null);
        // For the Autonomous timer we really don't care about exceptions checkAutoTimerCompletion();
    }

    // If you care about exceptions in any CompletableFuture that
    // contains a loop you need to periodically test the status of
    // the CompletableFuture. Only then will the exception be thrown.
    private void checkAutoTimerCompletion() throws IOException, InterruptedException, TimeoutException {
        if (autoTimerFuture.isDone())
            Threading.getFutureCompletion(autoTimerFuture);
    }

    // Gets the amount of time used in the Autonomous period
    // and posts a panic stop if the time used exceeds our
    // safe shutdown limit. Used to ensure that there's enough
    // time to lower the elevator in the event that the Autonomous
    // period is close to expiring.
    private class AutoTimerCallable extends AutoWorker<Void> {
        AutoTimerCallable() {
            super();
        }

        public Void call() {
            RobotLogCommon.i(TAG, "AutonomousTimer thread is running");
            while (!stopThreadRequested() && !Thread.interrupted()) {
                if (linearOpMode.getRuntime() >= SAFE_SHUTDOWN_LIMIT) {
                    panicStop.set(true);
                    break;
                }

                linearOpMode.sleep(25);
            }

            return null;
        }
    }

}
