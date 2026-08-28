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

    

}
