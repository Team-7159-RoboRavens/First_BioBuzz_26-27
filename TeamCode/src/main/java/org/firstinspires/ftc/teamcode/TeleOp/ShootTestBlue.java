package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.ShootTestButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;

// Working bot teleop with g2 dpad setting the RPM by hand, distance measured to the blue HIVE
@TeleOp(name = "ShootTestBlue")
public class ShootTestBlue extends TeleOpWorkingBot {
    @Override
    protected boolean isRed() { return false; }

    @Override
    protected WorkingBotButtonMap createButtonMap() {
        return new ShootTestButtonMap(this, robot, isRed());
    }
}
