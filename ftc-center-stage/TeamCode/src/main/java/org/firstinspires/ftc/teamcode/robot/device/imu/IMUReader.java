package org.firstinspires.ftc.teamcode.robot.device.imu;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    private final BNO055IMU imu;
    private boolean imuActivated = false;

    // Thread-related.
    private final CountDownLatch countDownLatch;
    private final IMUReaderCallable imuReaderCallable;
    private final CompletableFuture<Void> imuReaderFuture;

    private final AtomicReference<Orientation> imuOrientation;
    private final ElapsedTime imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final AtomicLong imuReadCount = new AtomicLong();

    // The IMU must have been previously initialized.
    public IMUReader(BNO055IMU pInitializedIMU) throws InterruptedException {
        imu = pInitializedIMU;
        imuOrientation = new AtomicReference<>(imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX));

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
        Orientation angles = imuOrientation.get();
        return (DEGREES.normalize(angles.firstAngle));
    }

    //# NOTE: the IMU on our 2018-2019 robot is oriented such that "secondAngle"
    //  reports pitch and "thirdAngle" reports roll. This is unlike the
    //  SensorBNO055IMU sample:
    //           .addData("roll", new Func<String>() {
    //                @Override public String value() {
    //                    return formatAngle(angles.angleUnit, angles.secondAngle);
    public double getIMUPitch() {
        Orientation angles = imuOrientation.get();
        return (DEGREES.normalize(angles.secondAngle));
    }

    public double getIMURoll() {
        Orientation angles = imuOrientation.get();
        return (DEGREES.normalize(angles.thirdAngle));
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
                imuOrientation.set(imu.getAngularOrientation().toAxesReference(INTRINSIC).toAxesOrder(ZYX));
                imuReadCount.incrementAndGet();
            }

            return null;
        }
    }

}
