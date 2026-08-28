package org.firstinspires.ftc.teamcode.ButtonMaps;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class PinpointBMTest {

    @Test
    public void getRotationZERO() {
        assertEquals(0, PinpointBM.getRotation(0), 0);
    }

    @Test
    public void getRotationpt1() {
        assertEquals(0, PinpointBM.getRotation(PinpointBM.RIGHT_STICK_TRIGGER), 0);
    }

    @Test
    public void getRotationnpt1() {
        assertEquals(0, PinpointBM.getRotation(-PinpointBM.RIGHT_STICK_TRIGGER), 0);
    }

    @Test
    public void getRotationNEG1() {
        assertEquals(-1, PinpointBM.getRotation(-1), 0);
    }

    @Test
    public void getRotationPOS1() {
        assertEquals(1, PinpointBM.getRotation(1), 0);
    }

    @Test
    public void getRotationrand() {
        assertEquals(0.19753086419753088, PinpointBM.getRotation(0.5), 0);
    }

    @Test
    public void getRotationrandrandn() {
        assertEquals(-0.19753086419753088, PinpointBM.getRotation(-0.5), 0);
    }

    @Test
    public void getMovementZeroZero() {
        double[] xy = PinpointBM.getMovement(0, 0);
        assertEquals(0, xy[0], 0);
    }

}