package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SimpleParkBlueLeft")
public class SimpleParkBlueLeft extends SimpleParkBase {
    @Override protected Pose getStartPose() { return BLUE_START; }
    @Override protected Pose getParkPose() { return BLUE_LEFT_PARK; }
}
