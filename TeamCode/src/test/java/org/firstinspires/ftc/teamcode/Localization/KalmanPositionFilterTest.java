package org.firstinspires.ftc.teamcode.Localization;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class KalmanPositionFilterTest {
    private static final double MM_PER_INCH = 25.4;

    @Test
    public void pinpointOnlyTracksOdometryInInches() {
        KalmanPositionFilter filter = new KalmanPositionFilter();
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = filter.update(10 * MM_PER_INCH, -5 * MM_PER_INCH, 0.5, 0.02);

        assertFalse(filter.hasFieldPose());
        assertEquals(10.0, position.xInches, 0.5);
        assertEquals(-5.0, position.yInches, 0.5);
        assertEquals(0.5, position.headingRadians, 1e-9);
    }

    @Test
    public void pinpointOriginDoesNotMatterWithStartPose() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(72.0, 72.0, Math.PI / 2.0));
        // Pinpoint thinks it is at (1000, 1000) facing 0; the robot then drives 10 inches straight forward
        filter.update(1000.0, 1000.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = filter.update(1000.0 + 10 * MM_PER_INCH, 1000.0, 0.0, 0.02);

        // Forward on the field is +Y because the robot started facing PI/2
        assertEquals(72.0, position.xInches, 1e-6);
        assertEquals(82.0, position.yInches, 1e-6);
        assertEquals(Math.PI / 2.0, position.headingRadians, 1e-9);
    }

    @Test
    public void firstLimelightPoseSetsFieldPose() {
        KalmanPositionFilter filter = new KalmanPositionFilter();
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, 0.0, new Pose(30.0, 100.0, Math.PI), 0.0, 0.02);

        assertTrue(filter.hasFieldPose());
        assertEquals(30.0, position.xInches, 1e-6);
        assertEquals(100.0, position.yInches, 1e-6);
        assertEquals(Math.PI, position.headingRadians, 1e-6);

        // Odometry after the fix moves the robot in the field frame: forward is -X when facing PI
        position = filter.update(10 * MM_PER_INCH, 0.0, 0.0, 0.02);
        assertEquals(20.0, position.xInches, 1e-3);
        assertEquals(100.0, position.yInches, 1e-3);
    }

    @Test
    public void missingLimelightPoseKeepsOdometryEstimate() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, null, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = filter.update(5 * MM_PER_INCH, 0.0, 0.0, null, 0.0, 0.02);

        assertEquals(55.0, position.xInches, 1e-6);
        assertEquals(50.0, position.yInches, 1e-6);
    }

    @Test
    public void limelightPoseIsBlendedWithOdometry() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, 0.0, new Pose(52.0, 49.0, 0.02), 0.0, 0.02);

        assertTrue(position.xInches > 50.0 && position.xInches < 52.0);
        assertTrue(position.yInches > 49.0 && position.yInches < 50.0);
        assertTrue(position.headingRadians > 0.0 && position.headingRadians < 0.02);
    }

    @Test
    public void repeatedLimelightPosesCorrectOdometryDrift() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = null;
        for (int i = 0; i < 200; i++) {
            position = filter.update(0.0, 0.0, 0.0, new Pose(53.0, 50.0, 0.0), 0.0, 0.02);
        }

        assertEquals(53.0, position.xInches, 0.1);
    }

    @Test
    public void limelightLatencyComparesAgainstPastPose() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.1);
        // Drive 10 inches forward over 0.1 s
        filter.update(10 * MM_PER_INCH, 0.0, 0.0, 0.1);

        // The Limelight image is 0.1 s old and correctly shows where the robot was then, so nothing should change
        KalmanPositionFilter.FieldPosition position =
                filter.update(10 * MM_PER_INCH, 0.0, 0.0, new Pose(50.0, 50.0, 0.0), 0.1, 0.0);

        assertEquals(60.0, position.xInches, 1e-6);
    }

    @Test
    public void firstLimelightPoseTurnsMotionSinceImageWithHeadingFix() {
        KalmanPositionFilter filter = new KalmanPositionFilter();
        filter.update(0.0, 0.0, 0.0, 0.1);
        // Drive 10 inches forward over 0.1 s, before the field heading is known
        filter.update(10 * MM_PER_INCH, 0.0, 0.0, 0.1);

        // The 0.1 s old image shows the robot at (30, 100) facing PI, so it has since driven 10 inches toward -X
        KalmanPositionFilter.FieldPosition position =
                filter.update(10 * MM_PER_INCH, 0.0, 0.0, new Pose(30.0, 100.0, Math.PI), 0.1, 0.0);

        assertEquals(20.0, position.xInches, 1e-6);
        assertEquals(100.0, position.yInches, 1e-6);
        assertEquals(Math.PI, position.headingRadians, 1e-6);
    }

    @Test
    public void outlierLimelightPoseIsRejected() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, 0.0, new Pose(120.0, 10.0, 2.0), 0.0, 0.02);

        assertEquals(50.0, position.xInches, 1e-6);
        assertEquals(50.0, position.yInches, 1e-6);
    }

    @Test
    public void persistentLimelightDisagreementResetsEstimate() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = null;
        for (int i = 0; i < 10; i++) {
            position = filter.update(0.0, 0.0, 0.0, new Pose(120.0, 10.0, 2.0), 0.0, 0.02);
        }

        assertEquals(120.0, position.xInches, 1e-6);
        assertEquals(10.0, position.yInches, 1e-6);
        assertEquals(2.0, position.headingRadians, 1e-6);
    }

    @Test
    public void headingWrapsAcrossZero() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, Math.toRadians(1.0)));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, 0.0, new Pose(50.0, 50.0, Math.toRadians(359.0)), 0.0, 0.02);

        // Blending 1 and 359 degrees must land near 0/360, never near 180
        double degrees = Math.toDegrees(position.headingRadians);
        assertTrue(degrees < 1.0 || degrees > 359.0);
        assertTrue(position.headingRadians >= 0.0 && position.headingRadians < 2.0 * Math.PI);
    }

    @Test
    public void nonFiniteLimelightPoseIsIgnored() {
        KalmanPositionFilter filter = new KalmanPositionFilter(new Pose(50.0, 50.0, 0.0));
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, 0.0, new Pose(Double.NaN, 50.0, 0.0), 0.0, 0.02);

        assertEquals(50.0, position.xInches, 1e-6);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNegativeDeltaTime() {
        new KalmanPositionFilter().update(0.0, 0.0, 0.0, -0.01);
    }
}
