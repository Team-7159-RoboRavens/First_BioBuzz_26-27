package org.firstinspires.ftc.teamcode.ButtonMaps;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class PinpointBMTest {

    @Test
    public void getRotation() {
        assertEquals(0, PinpointBM.getRotation(0), 0);
    }
}