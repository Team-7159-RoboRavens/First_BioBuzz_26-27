package org.firstinspires.ftc.teamcode.Localization;

/**
 * Estimates field position and heading from successive Pinpoint measurements.
 * Position coordinates are in millimeters and heading is in radians.
 */
public final class KalmanPositionFilter {
    private static final double POSITION_MEASUREMENT_VARIANCE = 25.0 * 25.0;
    private static final double POSITION_ACCELERATION_VARIANCE = 250000.0;
    private static final double HEADING_MEASUREMENT_VARIANCE = 0.03 * 0.03;
    private static final double HEADING_ACCELERATION_VARIANCE = 0.25;

    private final ConstantVelocityFilter xFilter =
            new ConstantVelocityFilter(POSITION_MEASUREMENT_VARIANCE, POSITION_ACCELERATION_VARIANCE);
    private final ConstantVelocityFilter yFilter =
            new ConstantVelocityFilter(POSITION_MEASUREMENT_VARIANCE, POSITION_ACCELERATION_VARIANCE);
    private final ConstantVelocityFilter headingFilter =
            new ConstantVelocityFilter(HEADING_MEASUREMENT_VARIANCE, HEADING_ACCELERATION_VARIANCE);

    private boolean hasHeadingMeasurement;
    private double previousHeadingMeasurement;
    private double unwrappedHeadingMeasurement;

    /**
     * Incorporates one position and heading measurement and returns the filtered pose.
     *
     * @param measuredXMillimeters measured field X position in millimeters
     * @param measuredYMillimeters measured field Y position in millimeters
     * @param measuredHeadingRadians measured heading in radians
     * @param deltaTimeSeconds time since the previous measurement, in seconds
     * @return the filtered field position and heading
     */
    public FieldPosition update(
            double measuredXMillimeters,
            double measuredYMillimeters,
            double measuredHeadingRadians,
            double deltaTimeSeconds) {
        if (!isFinite(measuredXMillimeters)
                || !isFinite(measuredYMillimeters)
                || !isFinite(measuredHeadingRadians)
                || !isFinite(deltaTimeSeconds)
                || deltaTimeSeconds < 0.0) {
            throw new IllegalArgumentException("Measurements must be finite and delta time must be nonnegative");
        }

        if (hasHeadingMeasurement) {
            unwrappedHeadingMeasurement += normalizeRadians(
                    measuredHeadingRadians - previousHeadingMeasurement);
        } else {
            unwrappedHeadingMeasurement = measuredHeadingRadians;
        }
        previousHeadingMeasurement = measuredHeadingRadians;
        hasHeadingMeasurement = true;

        return new FieldPosition(
                xFilter.update(measuredXMillimeters, deltaTimeSeconds),
                yFilter.update(measuredYMillimeters, deltaTimeSeconds),
                normalizeRadians(headingFilter.update(unwrappedHeadingMeasurement, deltaTimeSeconds)));
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

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** Immutable filtered field pose. */
    public static final class FieldPosition {
        public final double xMillimeters;
        public final double yMillimeters;
        public final double headingRadians;

        private FieldPosition(double xMillimeters, double yMillimeters, double headingRadians) {
            this.xMillimeters = xMillimeters;
            this.yMillimeters = yMillimeters;
            this.headingRadians = headingRadians;
        }
    }

    private static final class ConstantVelocityFilter {
        private final double measurementVariance;
        private final double accelerationVariance;
        private boolean initialized;
        private double position;
        private double velocity;
        private double positionVariance;
        private double positionVelocityCovariance;
        private double velocityVariance;

        private ConstantVelocityFilter(double measurementVariance, double accelerationVariance) {
            this.measurementVariance = measurementVariance;
            this.accelerationVariance = accelerationVariance;
        }

        private double update(double measurement, double deltaTimeSeconds) {
            if (!initialized) {
                initialized = true;
                position = measurement;
                positionVariance = measurementVariance;
                velocityVariance = accelerationVariance * 0.04;
                return position;
            }

            double dt2 = deltaTimeSeconds * deltaTimeSeconds;
            double dt3 = dt2 * deltaTimeSeconds;
            double dt4 = dt2 * dt2;
            double predictedPosition = position + deltaTimeSeconds * velocity;
            double predictedPositionVariance = positionVariance
                    + 2.0 * deltaTimeSeconds * positionVelocityCovariance
                    + dt2 * velocityVariance
                    + accelerationVariance * dt4 / 4.0;
            double predictedCovariance = positionVelocityCovariance
                    + deltaTimeSeconds * velocityVariance
                    + accelerationVariance * dt3 / 2.0;
            double predictedVelocityVariance = velocityVariance + accelerationVariance * dt2;
            double innovationVariance = predictedPositionVariance + measurementVariance;
            double positionGain = predictedPositionVariance / innovationVariance;
            double velocityGain = predictedCovariance / innovationVariance;
            double residual = measurement - predictedPosition;

            position = predictedPosition + positionGain * residual;
            velocity += velocityGain * residual;
            positionVariance = (1.0 - positionGain) * predictedPositionVariance;
            positionVelocityCovariance = (1.0 - positionGain) * predictedCovariance;
            velocityVariance = predictedVelocityVariance - velocityGain * predictedCovariance;
            return position;
        }

    }
}
