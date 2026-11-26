package org.firstinspires.ftc.teamcode.Localization;

import com.pedropathing.math.Pose;

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
    // Starting size of the history ring buffer (a power of two), it doubles if a second of loops doesn't fit
    private static final int HISTORY_CAPACITY = 256;
    private static final double[] MEASUREMENT_VARIANCE = {
            LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_HEADING_VARIANCE};

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

    // Past estimates as a ring buffer, the oldest is at historyStart
    private double[] historyTime = new double[HISTORY_CAPACITY];
    private double[] historyX = new double[HISTORY_CAPACITY];
    private double[] historyY = new double[HISTORY_CAPACITY];
    private double[] historyHeading = new double[HISTORY_CAPACITY];
    private int historyStart;
    private int historySize;

    // Pose found by the last poseAt() call
    private double pastX;
    private double pastY;
    private double pastHeading;

    // Reused by correct() so a Limelight pose makes no garbage
    private final double[] residual = new double[3];
    private final double[] weightedResidual = new double[3];
    private final double[] correction = new double[3];
    private final double[] innovationCovariance = new double[9];
    private final double[] innovationInverse = new double[9];
    private final double[] gain = new double[9];
    private final double[] identityMinusGain = new double[9];
    private final double[] identityMinusGainTranspose = new double[9];
    private final double[] partialCovariance = new double[9];
    private final double[] newCovariance = new double[9];

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

    // Moves the estimate by this much and trusts the result like a fresh Limelight pose, for a manual relocalize
    public void shiftEstimate(double deltaX, double deltaY, double deltaHeading) {
        applyCorrection(x, y, deltaX, deltaY, normalizeRadians(deltaHeading));
        setCovariance(LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_HEADING_VARIANCE);
        hasFieldPose = true;
        rejectedPosesInARow = 0;
    }

    // Current estimate without a new measurement
    public FieldPosition getPosition() {
        return new FieldPosition(x, y, toPedroHeading(heading));
    }

    // Call after the Pinpoint is reset, the next reading becomes the new reference instead of a jump
    public void resetPinpointReference() {
        hasPinpointMeasurement = false;
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
        double pinpointCos = Math.cos(pinpointMidHeading);
        double pinpointSin = Math.sin(pinpointMidHeading);
        double forward = pinpointCos * pinpointDeltaX + pinpointSin * pinpointDeltaY;
        double left = -pinpointSin * pinpointDeltaX + pinpointCos * pinpointDeltaY;

        // Robot frame -> field frame, using the estimated field heading halfway through the move
        double fieldMidHeading = heading + deltaHeading / 2.0;
        double fieldCos = Math.cos(fieldMidHeading);
        double fieldSin = Math.sin(fieldMidHeading);
        double fieldDeltaX = fieldCos * forward - fieldSin * left;
        double fieldDeltaY = fieldSin * forward + fieldCos * left;

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
        poseAt(elapsedSeconds - latencySeconds);
        residual[0] = measuredX - pastX;
        residual[1] = measuredY - pastY;
        residual[2] = normalizeRadians(measuredHeading - pastHeading);

        System.arraycopy(covariance, 0, innovationCovariance, 0, 9);
        for (int i = 0; i < 3; i++) {
            innovationCovariance[i * 4] += MEASUREMENT_VARIANCE[i];
        }
        if (!invert3x3(innovationCovariance, innovationInverse)) {
            return;
        }

        double mahalanobisSquared = 0.0;
        multiply3x3By3(innovationInverse, residual, weightedResidual);
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
        multiply3x3(covariance, innovationInverse, gain);
        multiply3x3By3(gain, residual, correction);
        applyCorrection(pastX, pastY, correction[0], correction[1], correction[2]);

        for (int i = 0; i < 9; i++) {
            identityMinusGain[i] = (i % 4 == 0 ? 1.0 : 0.0) - gain[i];
        }
        multiply3x3(identityMinusGain, covariance, partialCovariance);
        transpose3x3(identityMinusGain, identityMinusGainTranspose);
        multiply3x3(partialCovariance, identityMinusGainTranspose, newCovariance);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += gain[row * 3 + k] * MEASUREMENT_VARIANCE[k] * gain[col * 3 + k];
                }
                newCovariance[row * 3 + col] += sum;
            }
        }
        System.arraycopy(newCovariance, 0, covariance, 0, 9);
    }

    // Sets the estimate so that it matched the given pose when the image was taken, keeping the odometry motion since then
    private void resetTo(double measuredX, double measuredY, double measuredHeading, double latencySeconds) {
        poseAt(elapsedSeconds - latencySeconds);
        applyCorrection(pastX, pastY, measuredX - pastX, measuredY - pastY, normalizeRadians(measuredHeading - pastHeading));
        setCovariance(LIMELIGHT_POSITION_VARIANCE, LIMELIGHT_HEADING_VARIANCE);
        hasFieldPose = true;
        rejectedPosesInARow = 0;
    }

    // Moves the corrected pose by (deltaX, deltaY, deltaHeading) and carries the current estimate and the history along
    // rigidly, rotating them around the corrected pose. Motion driven since the image was taken then turns with the
    // heading fix instead of keeping its old direction, and later latency lookups stay consistent.
    private void applyCorrection(double pivotX, double pivotY, double deltaX, double deltaY, double deltaHeading) {
        double cos = Math.cos(deltaHeading);
        double sin = Math.sin(deltaHeading);

        double relativeX = x - pivotX;
        double relativeY = y - pivotY;
        x = pivotX + deltaX + cos * relativeX - sin * relativeY;
        y = pivotY + deltaY + sin * relativeX + cos * relativeY;
        heading = normalizeRadians(heading + deltaHeading);
        int mask = historyTime.length - 1;
        for (int i = 0; i < historySize; i++) {
            int index = (historyStart + i) & mask;
            relativeX = historyX[index] - pivotX;
            relativeY = historyY[index] - pivotY;
            historyX[index] = pivotX + deltaX + cos * relativeX - sin * relativeY;
            historyY[index] = pivotY + deltaY + sin * relativeX + cos * relativeY;
            historyHeading[index] = normalizeRadians(historyHeading[index] + deltaHeading);
        }
    }

    private void recordHistory() {
        if (historySize == historyTime.length) {
            growHistory();
        }
        int mask = historyTime.length - 1;
        int newest = (historyStart + historySize) & mask;
        historyTime[newest] = elapsedSeconds;
        historyX[newest] = x;
        historyY[newest] = y;
        historyHeading[newest] = heading;
        historySize++;
        while (historySize > 0 && historyTime[historyStart] < elapsedSeconds - HISTORY_SECONDS) {
            historyStart = (historyStart + 1) & mask;
            historySize--;
        }
    }

    // Doubles the history buffer when it is full, keeping the samples oldest first
    private void growHistory() {
        historyTime = unwrapHistory(historyTime);
        historyX = unwrapHistory(historyX);
        historyY = unwrapHistory(historyY);
        historyHeading = unwrapHistory(historyHeading);
        historyStart = 0;
    }

    private double[] unwrapHistory(double[] old) {
        double[] bigger = new double[old.length * 2];
        int mask = old.length - 1;
        for (int i = 0; i < historySize; i++) {
            bigger[i] = old[(historyStart + i) & mask];
        }
        return bigger;
    }

    // Estimated pose at the given time, interpolated from the history (the current pose if the time is newer than it)
    // The result goes in pastX, pastY and pastHeading
    private void poseAt(double time) {
        double newerTime = elapsedSeconds;
        double newerX = x;
        double newerY = y;
        double newerHeading = heading;
        int mask = historyTime.length - 1;
        for (int i = historySize - 1; i >= 0; i--) {
            int index = (historyStart + i) & mask;
            double olderTime = historyTime[index];
            double olderX = historyX[index];
            double olderY = historyY[index];
            double olderHeading = historyHeading[index];
            if (olderTime <= time) {
                double span = newerTime - olderTime;
                double t = span > 0.0 ? (time - olderTime) / span : 0.0;
                pastX = olderX + t * (newerX - olderX);
                pastY = olderY + t * (newerY - olderY);
                pastHeading = normalizeRadians(olderHeading + t * normalizeRadians(newerHeading - olderHeading));
                return;
            }
            newerTime = olderTime;
            newerX = olderX;
            newerY = olderY;
            newerHeading = olderHeading;
        }
        // Older than anything kept; the oldest sample is the best guess
        pastX = newerX;
        pastY = newerY;
        pastHeading = newerHeading;
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

    // result must be a different array from left and right
    private static void multiply3x3(double[] left, double[] right, double[] result) {
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += left[row * 3 + k] * right[k * 3 + col];
                }
                result[row * 3 + col] = sum;
            }
        }
    }

    // result must be a different array from vector
    private static void multiply3x3By3(double[] matrix, double[] vector, double[] result) {
        for (int row = 0; row < 3; row++) {
            result[row] = matrix[row * 3] * vector[0] + matrix[row * 3 + 1] * vector[1] + matrix[row * 3 + 2] * vector[2];
        }
    }

    private static void transpose3x3(double[] m, double[] result) {
        result[0] = m[0];
        result[1] = m[3];
        result[2] = m[6];
        result[3] = m[1];
        result[4] = m[4];
        result[5] = m[7];
        result[6] = m[2];
        result[7] = m[5];
        result[8] = m[8];
    }

    // returns false (and leaves result alone) if the matrix cannot be inverted
    private static boolean invert3x3(double[] m, double[] result) {
        double c00 = m[4] * m[8] - m[5] * m[7];
        double c01 = m[5] * m[6] - m[3] * m[8];
        double c02 = m[3] * m[7] - m[4] * m[6];
        double determinant = m[0] * c00 + m[1] * c01 + m[2] * c02;
        if (Math.abs(determinant) < 1e-12) {
            return false;
        }
        double inverse = 1.0 / determinant;
        result[0] = c00 * inverse;
        result[1] = (m[2] * m[7] - m[1] * m[8]) * inverse;
        result[2] = (m[1] * m[5] - m[2] * m[4]) * inverse;
        result[3] = c01 * inverse;
        result[4] = (m[0] * m[8] - m[2] * m[6]) * inverse;
        result[5] = (m[2] * m[3] - m[0] * m[5]) * inverse;
        result[6] = c02 * inverse;
        result[7] = (m[1] * m[6] - m[0] * m[7]) * inverse;
        result[8] = (m[0] * m[4] - m[1] * m[3]) * inverse;
        return true;
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
}
