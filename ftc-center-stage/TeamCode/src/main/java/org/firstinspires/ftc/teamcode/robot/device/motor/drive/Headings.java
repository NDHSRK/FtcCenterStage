package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;

// The original implementations of the functions getActualAngle()
// and normalize() can be found in the Visual Studio 2019
// AngleCalculator project. Any use of these functions in IntelliJ
// or Android Studio projects must be kept in sync.
public class Headings {

    private static final String TAG = "Headings";

    // In our implementation turns always take the shortest distance from the
    // current heading to the target. This normalization, which is standard in
    // the FTC SDK support of the IMU, may result in a change in the turn
    // direction. For example, if the current heading is 0 and the requested
    // turn is -270 (clockwise), normalization will change the turn to +90
    // (counter-clockwise).

    // But in the 2018-19 FTC game the robot needed to make a clockwise turn to
    // free itself from the hook on the Rover. So the software needs to support
    // a "turn in requested direction" or an "unnormalized turn".

    // There is also a question about turn direction: should it be calculated
    // from the desired heading or from the current heading?
    // Example: normalized turn with a desired heading of -175, a turn angle of
    // 175, and a current heading of 170 ---

    // From the desired heading of -175 the shortest distance to the final
    // desired heading after the turn (0) is CCW 175. But from the actual
    // current heading of 170 the shortest distance is CW -170.

    // In keeping with the principle that a normalized turn should travel
    // the shortest distance, the turn will be CW -170. Otherwise the turn
    // would become an unnormalized turn of CCW 190.

    // What about this case?
    // N(ormalized) (desired) 178 (turn) 3 (current) -178

    // The method getActualTurn supports a change of direction in a normalized turn,
    // which in this case would result in a value of -1 (178 - -178 -> 356, which
    // normalizes to -4 -> + current (3) -> -1.

    // The actual turn value comes out as -1, a CCW turn that is the reverse of the
    // requested direction; this is allowed for a normalized turn. But what about
    // this case:
    // U(nnormalized) (desired) 178 (turn) 3 (current) -178
    // An unnormalized turn may never change direction; this function returns an
    // actual turn value of 0.

    // Contains a failsafe: if the difference between the desired heading before the turn
    // and the actual current heading is > 90 degrees, return 0.

    //## The current heading and the turn degrees both follow FTC conventions.
    //## For a normalized turn: return the angle in the FTC range: -180 to +180 (not including +180).
    //## For an unnormalized turn: return the angle in the range of -360 .. 360 (including both);
    //## the range is enforced in the XML schema (IntelliJ only).
    public static TurnData getActualTurn(double pDesiredHeadingBeforeTurn, double pCurrentHeading,
                                         double pTurnDegrees, DriveTrainConstants.TurnNormalization pTurnType) {

        double nDiff = normalize(pDesiredHeadingBeforeTurn - pCurrentHeading, -180, 180);

        // Failsafe against excessive difference between the desired heading and the current heading.
        if (Math.abs(nDiff) > 90) {
            RobotLogCommon.d(TAG, "90-degree failsafe exceeded, returning 0");
            return new TurnData(0.0, pDesiredHeadingBeforeTurn);
        }

        double actualTurn = pTurnDegrees + nDiff;
        double desiredHeadingAfterTurn = normalize(pCurrentHeading + actualTurn, -180, 180);
        if (pTurnType == DriveTrainConstants.TurnNormalization.NORMALIZED) {
            actualTurn = normalize(actualTurn, -180, 180);
        }
        else {
            // For an unnormalized turn the requested turn direction must not be changed.
            if (actualTurn != 0 && (pTurnDegrees * actualTurn) < 0) { // different signs?
                actualTurn = 0;
                desiredHeadingAfterTurn = pDesiredHeadingBeforeTurn;
                RobotLogCommon.d(TAG, "Unnormalized turn changed direction, returning 0");
            }
        }

        return new TurnData(actualTurn, desiredHeadingAfterTurn);
    }

    public static class TurnData {
        public final double actualTurn;
        public final double desiredHeadingAfterTurn;

        public TurnData(double pActualTurn, double pDesiredHeadingAfterTurn) {
            actualTurn = pActualTurn;
            desiredHeadingAfterTurn = pDesiredHeadingAfterTurn;
        }
    }

    // From the Visual Studio 2019 AngleCalculator project
    // Source module: HeadingUtils.cpp
    // Normalizes any number to an arbitrary range
    // by assuming the range wraps around when going below min or above max
    // From https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360
    // Example: double nDiff = headingUtils.normalise(pDesiredHeadingBeforeTurn - pCurrentHeading, -180, 180);
    public static double normalize(double value, double start, double end) {
        double width = end - start;   //
        double offsetValue = value - start;   // value relative to 0

        return (offsetValue - (Math.floor(offsetValue / width) * width)) + start;
        // + start to reset back to start of original range
    }
}
