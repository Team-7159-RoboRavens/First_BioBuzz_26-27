package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SimpleParkRed")
public class SimpleParkRed extends SimpleParkBase {
    @Override protected Pose getStartPose() { return RED_START; }
}
