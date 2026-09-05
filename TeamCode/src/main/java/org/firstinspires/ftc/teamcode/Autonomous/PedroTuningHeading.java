package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="PedroTuningHeading")
public class PedroTuningHeading extends OpMode {
    private Follower follower;

    public enum PathState {
        START_POS,
        END_POS
    }

    PathState pathState;
    private Robot robot;
    private final Pose startPose = new Pose(0, 0, 0);

    private final Pose endPose = new Pose(1, 0, 2);

    private PathChain chain1;

    public void buildPaths() {
        chain1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        telemetry.addLine("path built");
    }

    public void updateStates() {
        telemetry.addLine("running");
        switch (pathState) {
            case START_POS:
                telemetry.addLine("start pos");
                follower.followPath(chain1, true);
                pathState = PathState.END_POS;
                break;
            case END_POS:
                telemetry.addLine("end pos");
                if (!follower.isBusy()) {
                    telemetry.addLine("finished chain1");
                }
                break;
            default:
                telemetry.addLine("default");
                break;
        }
    }
    @Override
    public void init() {
        pathState = PathState.START_POS;
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void loop() {
        follower.update();
        updateStates();

        telemetry.addData("Pinpoint Angle", robot.pinpointAngle());
    }
}