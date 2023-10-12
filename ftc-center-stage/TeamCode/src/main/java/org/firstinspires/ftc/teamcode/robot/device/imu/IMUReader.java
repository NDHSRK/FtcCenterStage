package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

// Class that continuously reads IMU data and posts the results.
//## Not designed as a Singleton and not intended that an instance
//## of this class be shared between threads.
public class IMUReader {

    private static final String TAG = "IMUReader";

    private final IMU imu;
    private boolean imuActivated;

    // Thread-related.
    private final CountDownLatch countDownLatch;
    private final IMUReaderCallable imuReaderCallable;
    private final CompletableFuture<Void> imuReaderFuture;

    private final AtomicReference<YawPitchRollAngles> imuOrientation;
    private final ElapsedTime imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final AtomicLong imuReadCount = new AtomicLong();

    // The IMU must have been previously initialized.
    public IMUReader(IMU pInitializedIMU) throws InterruptedException {
        imu = pInitializedIMU;
        imuOrientation = new AtomicReference<>(imu.getRobotYawPitchRollAngles());

        // Start up the IMU reader as a CompletableFuture.
        RobotLogCommon.i(TAG, "Starting IMU reader thread");

        countDownLatch = new CountDownLatch(1);
        imuReaderCallable = new IMUReaderCallable();
        imuReaderFuture = Threading.launchAsync(imuReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
        imuActivated = true;

        RobotLogCommon.i(TAG, "Wait for CountDownLatch done; IMU reader thread is running");
    }

    // To be called from the finally block of FTCAuto.
    public void stopIMUReader() throws IOException, InterruptedException, TimeoutException {
        if (!imuActivated)
            return; // nothing to do
        imuActivated = false;

        // Force stop now.
        imuReaderCallable.stopThread();
        imuReaderFuture.complete(null);
        checkIMUReaderCompletion();
    }

    // Returns the number of imu reads per second.
    public double getIMUReadRate() throws IOException, InterruptedException, TimeoutException {
        checkIMUReaderCompletion();
        long myReadCount = imuReadCount.get();
        double numSec = imuTimer.time() / 1000;
        return (numSec != 0) ? myReadCount / numSec : 0.0;
    }

    public double getIMUHeading() throws IOException, InterruptedException, TimeoutException {
        checkIMUReaderCompletion();
        YawPitchRollAngles angles = imuOrientation.get();
        return angles.getYaw(DEGREES);
    }

    public double getIMUPitch() {
        YawPitchRollAngles angles = imuOrientation.get();
        return angles.getPitch(DEGREES);
    }

    public double getIMURoll() {
        YawPitchRollAngles angles = imuOrientation.get();
        return angles.getRoll(DEGREES);
    }

    // In any CompletableFuture that contains a loop you need to
    // periodically test the status of the CompletableFuture to
    // make sure it has not completed with an exception.
    private void checkIMUReaderCompletion() throws IOException, InterruptedException, TimeoutException {
        if (imuReaderFuture.isDone())
          Threading.getFutureCompletion(imuReaderFuture);
    }

    // Reads the IMU and makes its data available to the main thread.
    private class IMUReaderCallable extends AutoWorker<Void> {

        IMUReaderCallable() {
            super();
        }

        public Void call() {
            RobotLogCommon.i(TAG, "In IMU thread");
            countDownLatch.countDown(); // signal that I'm running
            imuTimer.reset(); // start timing
            imuReadCount.set(0);

            //## the linearOpMode is not usually active when this thread is started so do not test linearOpMode.opModeIsActive() here.
            while (!stopThreadRequested() && !Thread.interrupted()) {
                imuOrientation.set(imu.getRobotYawPitchRollAngles());
                imuReadCount.incrementAndGet();
            }

            return null;
        }
    }

}
