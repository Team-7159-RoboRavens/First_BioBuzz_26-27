package org.firstinspires.ftc.teamcode.ButtonMaps;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.junit.Test;

// Unit tests for PinpointBM's static input-processing helpers (deadzone/rescale math,
// digital d-pad/bumper mapping) run without any gamepad or hardware dependency.
public class MechanumdriveTest {
    private final double ERROR = 1e-9;

    @Test
    public void setMotorPowersZero() {
        MotorPowers a = Mechanumdrive.setMotorPowers(0, 0, 0);
        assertEquals(0, a.rf, 0);
        assertEquals(0, a.lf, 0);
        assertEquals(0, a.rb, 0);
        assertEquals(0, a.lb, 0);
    }
    @Test
    public void setMotorPowersZeroForward() {
        MotorPowers a = Mechanumdrive.setMotorPowers(1, 0, 0);
        assertEquals(1, a.rf, ERROR);
        assertEquals(1, a.lf, ERROR);
        assertEquals(1, a.rb, ERROR);
        assertEquals(1, a.lb, ERROR);
    }
    @Test
    public void setMotorPowersRight() {
        MotorPowers a = Mechanumdrive.setMotorPowers(0, 1, 0);
        assertEquals(-1, a.rf, ERROR);
        assertEquals(1, a.lf, ERROR);
        assertEquals(1, a.rb, ERROR);
        assertEquals(-1, a.lb, ERROR);
    }
    @Test
    public void setMotorPowersBack() {
        MotorPowers a = Mechanumdrive.setMotorPowers(-1, 0, 0);
        assertEquals(-1, a.rf, ERROR);
        assertEquals(-1, a.lf, ERROR);
        assertEquals(-1, a.rb, ERROR);
        assertEquals(-1, a.lb, ERROR);
    }
    @Test
    public void setMotorPowersLeft() {
        MotorPowers a = Mechanumdrive.setMotorPowers(0, -1, 0);
        assertEquals(1, a.rf, ERROR);
        assertEquals(-1, a.lf, ERROR);
        assertEquals(-1, a.rb, ERROR);
        assertEquals(1, a.lb, ERROR);
    }
    @Test
    public void setMotorPowersClockWise() {
        MotorPowers a = Mechanumdrive.setMotorPowers(0, 0, 1);
        assertEquals(-1, a.rf, ERROR);
        assertEquals(1, a.lf, ERROR);
        assertEquals(-1, a.rb, ERROR);
        assertEquals(1, a.lb, ERROR);
    }
    @Test
    public void setMotorPowersCounterClockWise() {
        MotorPowers a = Mechanumdrive.setMotorPowers(0, 0, -11);
        assertEquals(1, a.rf, ERROR);
        assertEquals(-1, a.lf, ERROR);
        assertEquals(1, a.rb, ERROR);
        assertEquals(-1, a.lb, ERROR);
    }
    @Test
    public void FOD1() {
        MotorPowers a = Mechanumdrive.fieldOrientedDrive(2, 2, 0, 0);
        assertEquals(1, a.rf, ERROR);
        assertEquals(0, a.lf, ERROR);
        assertEquals(0, a.rb, ERROR);
        assertEquals(1, a.lb, ERROR);
    }
    @Test
    public void FOD2() {
        MotorPowers a = Mechanumdrive.fieldOrientedDrive(1, -2, -1, -Math.PI/2);
        assertEquals(-0.5, a.rf, ERROR);
        assertEquals(0, a.lf, ERROR);
        assertEquals(0.5, a.rb, ERROR);
        assertEquals(-1, a.lb, ERROR);
    }

    @Test
    public void FOD3() {
        MotorPowers a = Mechanumdrive.fieldOrientedDrive(-1, -5, 0, -Math.PI);
        assertEquals(1, a.rf, ERROR);
        assertEquals(0.666666666666667, a.lf, ERROR);
        assertEquals(0.666666666666667, a.rb, ERROR);
        assertEquals(1, a.lb, ERROR);
    }

    @Test
    public void FOD4() {
        MotorPowers a = Mechanumdrive.fieldOrientedDrive(1, -1, 0, Math.PI);
        assertEquals(0, a.rf, ERROR);
        assertEquals(1, a.lf, ERROR);
        assertEquals(1, a.rb, ERROR);
        assertEquals(0, a.lb, ERROR);
    }



}