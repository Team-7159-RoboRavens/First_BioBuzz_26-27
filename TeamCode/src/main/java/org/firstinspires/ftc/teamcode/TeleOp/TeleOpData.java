package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;

public class TeleOpData {
    public static double frameTime = 0;
    // Where the last auto ended, teleop starts its field position from here (null if no auto ran)
    public static Pose autoEndPose = null;
}