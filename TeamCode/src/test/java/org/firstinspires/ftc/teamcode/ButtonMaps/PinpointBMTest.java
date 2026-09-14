package org.firstinspires.ftc.teamcode.ButtonMaps;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

// Unit tests for PinpointBM's static input-processing helpers (deadzone/rescale math,
// digital d-pad/bumper mapping) run without any gamepad or hardware dependency.
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
        assertEquals(0, xy[1], 0);
    }

    @Test
    public void getMovementBelowTrigger() {
        double[] xy = PinpointBM.getMovement(0.05, 0.05);
        assertEquals(0, xy[0], 0);
        assertEquals(0, xy[1], 0);
    }

    @Test
    public void getMovementForwardOnly() {
        // Gamepad up is y = -1
        double[] xy = PinpointBM.getMovement(0, -1);
        assertEquals(1, xy[0], 1e-9);
        assertEquals(0, xy[1], 1e-9);
    }

    @Test
    public void getMovementBackwardOnly() {
        // Gamepad down is y = 1
        double[] xy = PinpointBM.getMovement(0, 1);
        assertEquals(-1, xy[0], 1e-9);
        assertEquals(0, xy[1], 1e-9);
    }

    @Test
    public void getMovementRightOnly() {
        double[] xy = PinpointBM.getMovement(1, 0);
        assertEquals(0, xy[0], 1e-9);
        assertEquals(1, xy[1], 1e-9);
    }

    @Test
    public void getMovementLeftOnly() {
        double[] xy = PinpointBM.getMovement(-1, 0);
        assertEquals(0, xy[0], 1e-9);
        assertEquals(-1, xy[1], 1e-9);
    }

    private final double MAX_DIAGONAL = Math.sqrt(2)/2;

    @Test
    public void getMovementForwardRightDiagonal() {
        // Gamepad UP and RIGHT (x = 1, y = -1)
        double[] xy = PinpointBM.getMovement(MAX_DIAGONAL, -MAX_DIAGONAL);
        double expectedPower = 1.0 / Math.sqrt(2); // ~0.7071
        assertEquals(0.7071067811865476, xy[0], 1e-9); // Forward components
        assertEquals(expectedPower, xy[1], 1e-9); // Right components
    }

    @Test
    public void getMovementForwardLeftDiagonal() {
        // Gamepad UP and LEFT (x = -1, y = -1)
        double[] xy = PinpointBM.getMovement(-MAX_DIAGONAL/2, -MAX_DIAGONAL);
        double expectedPower = 1.0 / Math.sqrt(2);
        assertEquals(0.5265924815402927, xy[0], 1e-9);  // Forward
        assertEquals(-0.26329624077014635, xy[1], 1e-9); // Left (Negative Right)
    }

    @Test
    public void getMovementBackwardRightDiagonal() {
        // Gamepad DOWN and RIGHT (x = 1, y = 1)
        double[] xy = PinpointBM.getMovement(MAX_DIAGONAL, MAX_DIAGONAL/3);
        double expectedPower = 1.0 / Math.sqrt(2);
        assertEquals(-0.1625974293983233, xy[0], 1e-9); // Backward (Negative Forward)
        assertEquals(0.4877922881949699, xy[1], 1e-9);  // Right
    }

    @Test
    public void getMovementBackwardLeftDiagonal() {
        // Gamepad DOWN and LEFT (x = -1, y = 1)
        double[] xy = PinpointBM.getMovement(-MAX_DIAGONAL/3, MAX_DIAGONAL);
        double expectedPower = 1.0 / Math.sqrt(2);
        assertEquals(-0.4877922881949699, xy[0], 1e-9); // Backward
        assertEquals(-0.1625974293983233, xy[1], 1e-9); // Left
    }

    @Test
    public void getIntakePowerZero() {
        assertEquals(0, PinpointBM.getIntakePower(0), 0);
    }

    @Test
    public void getIntakePowerAtTrigger() {
        assertEquals(0, PinpointBM.getIntakePower(0.1), 0);
    }

    @Test
    public void getIntakePowerFull() {
        assertEquals(1, PinpointBM.getIntakePower(1), 1e-9);
    }

    @Test
    public void getIntakePowerHalf() {
        assertEquals((0.55 - 0.1) / 0.9, PinpointBM.getIntakePower(0.55), 1e-9);
    }

    @Test
    public void getDpadForwardUp() {
        assertEquals(1, PinpointBM.getDpadForward(true, false), 0);
    }

    @Test
    public void getDpadForwardDown() {
        assertEquals(-1, PinpointBM.getDpadForward(false, true), 0);
    }

    @Test
    public void getDpadForwardNone() {
        assertEquals(0, PinpointBM.getDpadForward(false, false), 0);
    }

    @Test
    public void getDpadForwardBoth() {
        assertEquals(1, PinpointBM.getDpadForward(true, true), 0);
    }

    @Test
    public void getDpadRightRight() {
        assertEquals(1, PinpointBM.getDpadRight(false, true), 0);
    }

    @Test
    public void getDpadRightLeft() {
        assertEquals(-1, PinpointBM.getDpadRight(true, false), 0);
    }

    @Test
    public void getDpadRightNone() {
        assertEquals(0, PinpointBM.getDpadRight(false, false), 0);
    }

    @Test
    public void getDpadRightBoth() {
        assertEquals(-1, PinpointBM.getDpadRight(true, true), 0);
    }

    @Test
    public void getBumperRotateClockwiseRight() {
        assertEquals(1, PinpointBM.getBumperRotateClockwise(false, true), 0);
    }

    @Test
    public void getBumperRotateClockwiseLeft() {
        assertEquals(-1, PinpointBM.getBumperRotateClockwise(true, false), 0);
    }

    @Test
    public void getBumperRotateClockwiseNone() {
        assertEquals(0, PinpointBM.getBumperRotateClockwise(false, false), 0);
    }

    @Test
    public void getBumperRotateClockwiseBoth() {
        assertEquals(-1, PinpointBM.getBumperRotateClockwise(true, true), 0);
    }

}
