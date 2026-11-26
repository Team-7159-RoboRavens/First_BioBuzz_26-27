package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.ShootTestButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;

// Working bot teleop with g2 dpad setting the RPM by hand, distance measured to the red HIVE
@TeleOp(name = "ShootTestRed")
public class ShootTestRed extends TeleOpWorkingBot {
    @Override
    protected boolean isRed() { return true; }

    @Override
    protected WorkingBotButtonMap createButtonMap() {
        return new ShootTestButtonMap(this, robot, isRed());
    }
}
