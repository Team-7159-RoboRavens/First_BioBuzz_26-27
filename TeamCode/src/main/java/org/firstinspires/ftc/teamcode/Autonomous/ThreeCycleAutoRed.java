package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThreeCycleAutoRed")
public class ThreeCycleAutoRed extends ThreeCycleAutoBase {

    private final Pose startPose = new Pose(9, 25, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(59, 25, Math.toRadians(90));
    private final Pose intakePose = new Pose(8.876319758672755, 8.349924585218675, Math.toRadians(0));
    private final Pose intakeControl = new Pose(98.49396681749627, 13.523378582202092);
    private final Pose shootPose2 = new Pose(49.08748114630466, 131.72252073906486, Math.toRadians(100));
    private final Pose shoot2Control = new Pose(31.91251885369533, 96.97586726998493);
    private final Pose parkPose = new Pose(9.855957767722485, 112.22021116138762, Math.toRadians(280));
    private final Pose parkControl = new Pose(45.853695324283564, 110.28883861236804);

    @Override protected Pose getStartPose() { return startPose; }
    @Override protected Pose getShootPose1() { return shootPose1; }
    @Override protected Pose getIntakePose() { return intakePose; }
    @Override protected Pose getIntakeControl() { return intakeControl; }
    @Override protected Pose getShootPose2() { return shootPose2; }
    @Override protected Pose getShoot2Control() { return shoot2Control; }
    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
