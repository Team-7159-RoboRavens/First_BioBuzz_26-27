package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThreeCycleAutoBlue")
public class ThreeCycleAutoBlue extends ThreeCycleAutoBase {

    private final Pose startPose = new Pose(133, 117, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(85.23906485671196, 117, Math.toRadians(270));
    private final Pose intakePose = new Pose(133.08898944193066, 133.62971342383105, Math.toRadians(180));
    private final Pose intakeControl = new Pose(64.13273001508298, 126.42458521870286);
    private final Pose shootPose2 = new Pose(92.19909502262443, 10.49778469079939, Math.toRadians(108));
    private final Pose shoot2Control = new Pose(115.1184012066365, 44.029411764705884);
    private final Pose parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(108));
    private final Pose parkControl = new Pose(107.08220211161387, 58.49924585218703);

    @Override protected Pose getStartPose() { return startPose; }
    @Override protected Pose getShootPose1() { return shootPose1; }
    @Override protected Pose getIntakePose() { return intakePose; }
    @Override protected Pose getIntakeControl() { return intakeControl; }
    @Override protected Pose getShootPose2() { return shootPose2; }
    @Override protected Pose getShoot2Control() { return shoot2Control; }
    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}