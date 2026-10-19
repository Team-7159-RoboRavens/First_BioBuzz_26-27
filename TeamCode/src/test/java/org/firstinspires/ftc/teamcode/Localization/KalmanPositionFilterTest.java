package org.firstinspires.ftc.teamcode.Localization;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class KalmanPositionFilterTest {
    @Test
    public void firstMeasurementInitializesEstimatedPosition() {
        KalmanPositionFilter filter = new KalmanPositionFilter();

        KalmanPositionFilter.FieldPosition position = filter.update(120.0, -75.0, 0.5, 0.02);

        assertEquals(120.0, position.xMillimeters, 0.0);
        assertEquals(-75.0, position.yMillimeters, 0.0);
        assertEquals(0.5, position.headingRadians, 0.0);
    }

    @Test
    public void laterMeasurementsAreSmoothed() {
        KalmanPositionFilter filter = new KalmanPositionFilter();
        filter.update(0.0, 0.0, 0.0, 0.02);

        KalmanPositionFilter.FieldPosition position = filter.update(100.0, 100.0, 0.5, 0.02);

        assertTrue(position.xMillimeters > 0.0 && position.xMillimeters < 100.0);
        assertTrue(position.yMillimeters > 0.0 && position.yMillimeters < 100.0);
        assertTrue(position.headingRadians > 0.0 && position.headingRadians < 0.5);
    }

    @Test
    public void headingMeasurementWrapsAcrossPiBoundary() {
        KalmanPositionFilter filter = new KalmanPositionFilter();
        filter.update(0.0, 0.0, Math.toRadians(179.0), 0.02);

        KalmanPositionFilter.FieldPosition position =
                filter.update(0.0, 0.0, Math.toRadians(-179.0), 0.02);

        assertTrue(Math.abs(position.headingRadians) > Math.toRadians(170.0));
        assertTrue(Math.abs(position.headingRadians) <= Math.PI);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNegativeDeltaTime() {
        new KalmanPositionFilter().update(0.0, 0.0, 0.0, -0.01);
    }
}
