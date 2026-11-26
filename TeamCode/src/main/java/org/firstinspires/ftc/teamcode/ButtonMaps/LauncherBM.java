package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LauncherBM {
    private OpMode opmode;
    private Gamepad g2;
    /**
     * @param op running op mode, used to read gamepad state
     */
    public LauncherBM(OpMode op) {
        opmode = op;
        g2 = opmode.gamepad2;
    }


    // Kept between loops, the dpad only changes it
    private double RPM = 0;

    // Flywheel RPM: g2 dpad up = 183, down = off, right / left = 5 RPM more / less
    public double getState(){
        if (g2.dpadUpWasPressed()){
            RPM = 183;
        }
        if (g2.dpadDownWasPressed()){
            RPM = 0;
        }
        if (g2.dpadRightWasPressed()){
            RPM += 5;
        }
        if (g2.dpadLeftWasPressed()){
            RPM = Math.max(0, RPM - 5);
        }
        return RPM;
    }
}

