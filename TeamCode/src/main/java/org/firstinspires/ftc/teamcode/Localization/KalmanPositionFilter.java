package org.firstinspires.ftc.teamcode.Localization;

import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * Estimates the robot's field pose by fusing Pinpoint odometry with Limelight AprilTag poses.
 * The estimate is in Pedro Pathing coordinates: inches from the bottom left corner, heading in radians from 0 to 2PI.
 *
 * Pinpoint drives the prediction step: each loop the change in Pinpoint pose is rotated into the field frame and added
 * to the estimate, so Pinpoint's own origin does not matter. Limelight poses, when a tag is visible, correct the drift
 * that odometry builds up. Pass null for the Limelight pose whenever calculatePosition.getPedroPose() has no position.
 */
public final class KalmanPositionFilter {
    private static final double MILLIMETERS_PER_INCH = 25.4;

    // Limelight measurement noise (1 sigma of about 2 inches and 3 degrees)
    private static final double LIMELIGHT_POSITION_VARIANCE = 2.0 * 2.0;
    private static final double LIMELIGHT_HEADING_VARIANCE = Math.toRadians(3.0) * Math.toRadians(3.0);

    // Odometry drift: variance added per inch driven / radian turned, plus a small amount per second for wheel slip
    private static final double ODOMETRY_POSITION_VARIANCE_PER_INCH = 0.02;
    private static final double ODOMETRY_HEADING_VARIANCE_PER_RADIAN = 0.0001;
    private static final double ODOMETRY_POSITION_VARIANCE_PER_SECOND = 0.01;
    private static final double ODOMETRY_HEADING_VARIANCE_PER_SECOND = 0.000001;

    // Uncertainty of a starting pose passed to the constructor (1 inch and 2 degrees)
    private static final double START_POSITION_VARIANCE = 1.0;
    private static final double START_HEADING_VARIANCE = Math.toRadians(2.0) * Math.toRadians(2.0);

    // Limelight poses further than this from the estimate (chi-squared, 3 degrees of freedom, 99.9%) are treated as bad solves
    private static final double OUTLIER_GATE = 16.27;
    // After this many rejected poses in a row the estimate is assumed lost (e.g. odometry slipped) and is reset to the Limelight
    private static final int RESET_AFTER_REJECTED_POSES = 10;
    // How far back estimates are kept for Limelight latency compensation
    private static final double HISTORY_SECONDS = 1.0;

    private double x;
    private double y;
    private double heading;
    // 3x3 covariance of (x, y, heading), row major
    private final double[] covariance = new double[9];

    private boolean hasFieldPose;
    private boolean hasPinpointMeasurement;
    private double previousPinpointX;
    private double previousPinpointY;
    private double previousPinpointHeading;
    private double elapsedSeconds;
    private int rejectedPosesInARow;
    private final ArrayDeque<PoseSample> history = new ArrayDeque<>();

    /**
     * Creates a filter that does not know where the robot is yet. Until the first Limelight pose arrives the estimate is
     * odometry relative to where the robot started (starting at 0, 0, 0); the first Limelight pose then sets it outright.
     */
    public KalmanPositionFilter() {
        setCovariance(START_POSITION_VARIANCE, START_HEADING_VARIANCE);
    }

    /**
     * Creates a filter that starts at a known Pedro pose, such as an autonomous starting position.
     *
     * @param startPose starting pose in Pedro coordinates
     */
    public KalmanPositionFilter(Pose startPose) {
        x = startPose.x();
        y = startPose.y();
        heading = normalizeRadians(startPose.heading());
        hasFieldPose = true;
        setCovariance(START_POSITION_VARIANCE, START_HEADING_VARIANCE);
    }

    /**
     * Incorporates one Pinpoint measurement without a Limelight pose and returns the filtered pose.
     *
     * @param pinpointXMillimeters Pinpoint X position in millimeters
     * @param pinpointYMillimeters Pinpoint Y position in millimeters
     * @param pinpointHeadingRadians Pinpoint heading in radians
     * @param deltaTimeSeconds time since the previous update, in seconds
     * @return the filtered field position and heading
     */
    public FieldPosition update(
            double pinpointXMillimeters,
            double pinpointYMillimeters,
            double pinpointHeadingRadians,
            double deltaTimeSeconds) {
        return update(pinpointXMillimeters, pinpointYMillimeters, pinpointHeadingRadians, null, 0.0, deltaTimeSeconds);
    }

    /**
     * Incorporates one Pinpoint measurement and, if there is one, a Limelight pose, and returns the filtered pose.
     *
     * @param pinpointXMillimeters Pinpoint X position in millimeters
     * @param pinpointYMillimeters Pinpoint Y position in millimeters
     * @param pinpointHeadingRadians Pinpoint heading in radians
     * @param limelightPedroPose robot pose from calculatePosition.getPedroPose(), or null if the Limelight has no
     *                           position this loop (no tag visible, or the same Limelight frame as last loop)
     * @param limelightLatencySeconds how long ago the Limelight image was captured, in seconds
     * @param deltaTimeSeconds time since the previous update, in seconds
     * @return the filtered field position and heading
     */
    public FieldPosition update(
            double pinpointXMillimeters,
            double pinpointYMillimeters,
            double pinpointHeadingRadians,
            Pose limelightPedroPose,
            double limelightLatencySeconds,
            double deltaTimeSeconds) {
        if (!isFinite(pinpointXMillimeters)
                || !isFinite(pinpointYMillimeters)
                || !isFinite(pinpointHeadingRadians)
                || !isFinite(deltaTimeSeconds)
                || deltaTimeSeconds < 0.0) {
            throw new IllegalArgumentException("Measurements must be finite and delta time must be nonnegative");
        }

        elapsedSeconds += deltaTimeSeconds;
        predict(pinpointXMillimeters, pinpointYMillimeters, pinpointHeadingRadians, deltaTimeSeconds);
        if (isUsablePose(limelightPedroPose)) {
            double latency = isFinite(limelightLatencySeconds) ? Math.max(0.0, limelightLatencySeconds) : 0.0;
            correct(limelightPedroPose, latency);
        }
        recordHistory();

        return new FieldPosition(x, y, toPedroHeading(heading));
    }

    /**
     * @return true once the estimate is in Pedro field coordinates (a starting pose was given or the Limelight has seen a tag)
     */
    public boolean hasFieldPose() {
        return hasFieldPose;
    }

    // Moves the estimate by the change in Pinpoint pose since the last update and grows the uncertainty to match
    private void predict(double pinpointXMillimeters, double pinpointYMillimeters, double pinpointHeading, double deltaTimeSeconds) {
        if (!hasPinpointMeasurement) {
            hasPinpointMeasurement = true;
            previousPinpointX = pinpointXMillimeters;
            previousPinpointY = pinpointYMillimeters;
            previousPinpointHeading = pinpointHeading;
            return;
        }

        double pinpointDeltaX = (pinpointXMillimeters - previousPinpointX) / MILLIMETERS_PER_INCH;
        double pinpointDeltaY = (pinpointYMillimeters - previousPinpointY) / MILLIMETERS_PER_INCH;
        double deltaHeading = normalizeRadians(pinpointHeading - previousPinpointHeading);

        // Pinpoint frame -> robot frame (forward, left), using the heading halfway through the move
        double pinpointMidHeading = previousPinpointHeading + deltaHeading / 2.0;
        double forward = Math.cos(pinpointMidHeading) * pinpointDeltaX + Math.sin(pinpointMidHeading) * pinpointDeltaY;
        double left = -Math.sin(pinpointMidHeading) * pinpointDeltaX + Math.cos(pinpointMidHeading) * pinpointDeltaY;

        // Robot frame -> field frame, using the estimated field heading halfway through the move
        double fieldMidHeading = heading + deltaHeading / 2.0;
        double fieldDeltaX = Math.cos(fieldMidHeading) * forward - Math.sin(fieldMidHeading) * left;
        double fieldDeltaY = Math.sin(fieldMidHeading) * forward + Math.cos(fieldMidHeading) * left;

        x += fieldDeltaX;
        y += fieldDeltaY;
        heading = normalizeRadians(heading + deltaHeading);

        previousPinpointX = pinpointXMillimeters;
        previousPinpointY = pinpointYMillimeters;
        previousPinpointHeading = pinpointHeading;

        // P = F P F^T + Q, where F is the identity plus how a heading error swings the move: dx/dheading = -fieldDeltaY,
        // dy/dheading = fieldDeltaX
        double[] p = covariance;
        double a = -fieldDeltaY;
        double b = fieldDeltaX;
        double p00 = p[0] + 2.0 * a * p[2] + a * a * p[8];
        double p01 = p[1] + a * p[5] + b * p[2] + a * b * p[8];
        double p02 = p[2] + a * p[8];
        double p11 = p[4] + 2.0 * b * p[5] + b * b * p[8];
        double p12 = p[5] + b * p[8];
        double p22 = p[8];

        double distance = Math.hypot(forward, left);
        double positionNoise = ODOMETRY_POSITION_VARIANCE_PER_INCH * distance
                + ODOMETRY_POSITION_VARIANCE_PER_SECOND * deltaTimeSeconds;
        double headingNoise = ODOMETRY_HEADING_VARIANCE_PER_RADIAN * Math.abs(deltaHeading)
                + ODOMETRY_HEADING_VARIANCE_PER_SECOND * deltaTimeSeconds;

        p[0] = p00 + positionNoise;
        p[1] = p01;
        p[2] = p02;
        p[3] = p01;
        p[4] = p11 + positionNoise;
        p[5] = p12;
        p[6] = p02;
        p[7] = p12;
        p[8] = p22 + headingNoise;
    }

    // Pulls the estimate toward a Limelight pose, weighted by how much each is trusted
    private void correct(Pose limelightPose, double latencySeconds) {
        double measuredX = limelightPose.x();
        double measuredY = limelightPose.y();
        double measuredHeading = normalizeRadians(limelightPose.heading());

        if (!hasFieldPose) {
            // First sighting: the odometry-only estimate is in the wrong frame, so take the Limelight pose outright
            resetTo(measuredX, measuredY, measuredHeading, latencySeconds);
            return;
        }

        // Compare against where we thought the robot was when the image was taken, not where it is now
        PoseSample past = poseAt(elapsedSeconds - latencySeconds);
        double[] residual = {
                measuredX - past.x,
                measuredY - past.y,
                normalizeRadians(measuredHeading - past.heading)
        };

        double[] measurementVariance = {LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_HEADING_VARIANCE};
        double[] innovationCovariance = covariance.clone();
        for (int i = 0; i < 3; i++) {
            innovationCovariance[i * 4] += measurementVariance[i];
        }
        double[] innovationInverse = invert3x3(innovationCovariance);
        if (innovationInverse == null) {
            return;
        }

        double mahalanobisSquared = 0.0;
        double[] weightedResidual = multiply3x3By3(innovationInverse, residual);
        for (int i = 0; i < 3; i++) {
            mahalanobisSquared += residual[i] * weightedResidual[i];
        }
        if (mahalanobisSquared > OUTLIER_GATE) {
            rejectedPosesInARow++;
            if (rejectedPosesInARow >= RESET_AFTER_REJECTED_POSES) {
                resetTo(measuredX, measuredY, measuredHeading, latencySeconds);
            }
            return;
        }
        rejectedPosesInARow = 0;

        // K = P S^-1, then x += K r and P = (I - K) P (I - K)^T + K R K^T
        double[] gain = multiply3x3(covariance, innovationInverse);
        double[] correction = multiply3x3By3(gain, residual);
        applyCorrection(past, correction[0], correction[1], correction[2]);

        double[] identityMinusGain = new double[9];
        for (int i = 0; i < 9; i++) {
            identityMinusGain[i] = (i % 4 == 0 ? 1.0 : 0.0) - gain[i];
        }
        double[] newCovariance = multiply3x3(multiply3x3(identityMinusGain, covariance), transpose3x3(identityMinusGain));
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += gain[row * 3 + k] * measurementVariance[k] * gain[col * 3 + k];
                }
                newCovariance[row * 3 + col] += sum;
            }
        }
        System.arraycopy(newCovariance, 0, covariance, 0, 9);
    }

    // Sets the estimate so that it matched the given pose when the image was taken, keeping the odometry motion since then
    private void resetTo(double measuredX, double measuredY, double measuredHeading, double latencySeconds) {
        PoseSample past = poseAt(elapsedSeconds - latencySeconds);
        applyCorrection(past, measuredX - past.x, measuredY - past.y, normalizeRadians(measuredHeading - past.heading));
        setCovariance(LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_HEADING_VARIANCE);
        hasFieldPose = true;
        rejectedPosesInARow = 0;
    }

    // Moves the corrected pose by (deltaX, deltaY, deltaHeading) and carries the current estimate and the history along
    // rigidly, rotating them around the corrected pose. Motion driven since the image was taken then turns with the
    // heading fix instead of keeping its old direction, and later latency lookups stay consistent.
    private void applyCorrection(PoseSample corrected, double deltaX, double deltaY, double deltaHeading) {
        double pivotX = corrected.x;
        double pivotY = corrected.y;
        double cos = Math.cos(deltaHeading);
        double sin = Math.sin(deltaHeading);

        double relativeX = x - pivotX;
        double relativeY = y - pivotY;
        x = pivotX + deltaX + cos * relativeX - sin * relativeY;
        y = pivotY + deltaY + sin * relativeX + cos * relativeY;
        heading = normalizeRadians(heading + deltaHeading);
        for (PoseSample sample : history) {
            relativeX = sample.x - pivotX;
            relativeY = sample.y - pivotY;
            sample.x = pivotX + deltaX + cos * relativeX - sin * relativeY;
            sample.y = pivotY + deltaY + sin * relativeX + cos * relativeY;
            sample.heading = normalizeRadians(sample.heading + deltaHeading);
        }
    }

    private void recordHistory() {
        history.addLast(new PoseSample(elapsedSeconds, x, y, heading));
        while (!history.isEmpty() && history.peekFirst().time < elapsedSeconds - HISTORY_SECONDS) {
            history.removeFirst();
        }
    }

    // Estimated pose at the given time, interpolated from the history (the current pose if the time is newer than it)
    private PoseSample poseAt(double time) {
        PoseSample newer = new PoseSample(elapsedSeconds, x, y, heading);
        Iterator<PoseSample> newestFirst = history.descendingIterator();
        while (newestFirst.hasNext()) {
            PoseSample older = newestFirst.next();
            if (older.time <= time) {
                double span = newer.time - older.time;
                double t = span > 0.0 ? (time - older.time) / span : 0.0;
                return new PoseSample(
                        time,
                        older.x + t * (newer.x - older.x),
                        older.y + t * (newer.y - older.y),
                        normalizeRadians(older.heading + t * normalizeRadians(newer.heading - older.heading)));
            }
            newer = older;
        }
        // Older than anything kept; the oldest sample is the best guess
        return newer;
    }

    private void setCovariance(double positionVariance, double headingVariance) {
        for (int i = 0; i < 9; i++) {
            covariance[i] = 0.0;
        }
        covariance[0] = positionVariance;
        covariance[4] = positionVariance;
        covariance[8] = headingVariance;
    }

    private static boolean isUsablePose(Pose pose) {
        return pose != null && isFinite(pose.x()) && isFinite(pose.y()) && isFinite(pose.heading());
    }

    private static double[] multiply3x3(double[] left, double[] right) {
        double[] result = new double[9];
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += left[row * 3 + k] * right[k * 3 + col];
                }
                result[row * 3 + col] = sum;
            }
        }
        return result;
    }

    private static double[] multiply3x3By3(double[] matrix, double[] vector) {
        double[] result = new double[3];
        for (int row = 0; row < 3; row++) {
            result[row] = matrix[row * 3] * vector[0] + matrix[row * 3 + 1] * vector[1] + matrix[row * 3 + 2] * vector[2];
        }
        return result;
    }

    private static double[] transpose3x3(double[] m) {
        return new double[] {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
    }

    // returns null if the matrix cannot be inverted
    private static double[] invert3x3(double[] m) {
        double c00 = m[4] * m[8] - m[5] * m[7];
        double c01 = m[5] * m[6] - m[3] * m[8];
        double c02 = m[3] * m[7] - m[4] * m[6];
        double determinant = m[0] * c00 + m[1] * c01 + m[2] * c02;
        if (Math.abs(determinant) < 1e-12) {
            return null;
        }
        double inverse = 1.0 / determinant;
        return new double[] {
                c00 * inverse, (m[2] * m[7] - m[1] * m[8]) * inverse, (m[1] * m[5] - m[2] * m[4]) * inverse,
                c01 * inverse, (m[0] * m[8] - m[2] * m[6]) * inverse, (m[2] * m[3] - m[0] * m[5]) * inverse,
                c02 * inverse, (m[1] * m[6] - m[0] * m[7]) * inverse, (m[0] * m[4] - m[1] * m[3]) * inverse
        };
    }

    private static double normalizeRadians(double angle) {
        double normalized = angle % (2.0 * Math.PI);
        if (normalized > Math.PI) {
            normalized -= 2.0 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2.0 * Math.PI;
        }
        return normalized;
    }

    // Pedro headings go from 0 to 2PI
    private static double toPedroHeading(double angle) {
        double heading = angle % (2.0 * Math.PI);
        return heading < 0.0 ? heading + 2.0 * Math.PI : heading;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** Immutable filtered field pose, in Pedro Pathing coordinates. */
    public static final class FieldPosition {
        public final double xInches;
        public final double yInches;
        public final double headingRadians;

        private FieldPosition(double xInches, double yInches, double headingRadians) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = headingRadians;
        }

        /** @return this position as a Pedro Pathing pose, e.g. for follower.setPose() */
        public Pose toPedroPose() {
            return new Pose(xInches, yInches, headingRadians);
        }
    }

    private static final class PoseSample {
        private final double time;
        private double x;
        private double y;
        private double heading;

        private PoseSample(double time, double x, double y, double heading) {
            this.time = time;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
}
