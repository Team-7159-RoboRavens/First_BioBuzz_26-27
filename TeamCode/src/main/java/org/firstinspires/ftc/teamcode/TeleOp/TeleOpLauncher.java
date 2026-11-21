package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.LauncherBM;
import org.firstinspires.ftc.teamcode.SoloLauncher;

@TeleOp(name="TeleOpLauncher")
public class TeleOpLauncher extends OpMode {
    SoloLauncher launcher;
    LauncherBM buttonMap;

    @Override
    public void init() {
        launcher = new SoloLauncher(hardwareMap);
        buttonMap = new LauncherBM(this);
    }

    @Override
    public void loop() {
        launcher.setShootPower(buttonMap.getState());
    }
}
