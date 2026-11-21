package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LauncherBM {
    private static final long PINPOINT_RESET_NANOS = 400_000_000L;
    private OpMode opmode;
    private Gamepad g1;
    private Gamepad g2;
    /**
     * @param op running op mode, used to read gamepad state
     */
    public LauncherBM(OpMode op) {
        opmode = op;
        g1 = opmode.gamepad1;
        g2 = opmode.gamepad2;
    }


    public double getState(){
        double RPM = 0;
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
            RPM -= 5;
        }
        return RPM;
    }
}

