package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SimpleParkRedLeft")
public class SimpleParkRedLeft extends SimpleParkBase {
    @Override protected Pose getStartPose() { return RED_START; }
    @Override protected Pose getParkPose() { return RED_LEFT_PARK; }
}
