package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SimpleParkBlue")
public class SimpleParkBlue extends SimpleParkBase {
    @Override protected Pose getStartPose() { return BLUE_START; }
}
