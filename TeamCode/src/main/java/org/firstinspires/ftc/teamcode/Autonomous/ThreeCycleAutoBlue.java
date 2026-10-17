package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThreeCycleAutoBlue", group = "Autonomous")
public class ThreeCycleAutoBlue extends ThreeCycleBase {

    @Override
    public void init() {
        // Assign your exact Blue specific coordinate positions
        startPose = new Pose(133, 117, Math.toRadians(180));
        shootPose1 = new Pose(85.23906485671196, 117, Math.toRadians(270));
        intakePose = new Pose(133.08898944193066, 133.62971342383105, Math.toRadians(180));
        intakeControl = new Pose(64.13273001508298, 126.42458521870286);
        shootPose2 = new Pose(92.19909502262443, 10.49778469079939, Math.toRadians(108));
        shoot2Control = new Pose(115.1184012066365, 44.029411764705884);
        parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(108));
        parkControl = new Pose(107.08220211161387, 58.49924585218703);

        // Run the parent class initialization to instantiate hardware maps & mechanisms
        super.init();

        // Feed coordinates into follower, compile paths, and prime the state machine
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS1);
    }
}
