package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name="PedroPathingBase")
public class PedroPathingBase extends OpMode {
    private Follower follower;

    public enum PathState {
        START_POS,
        END_POS
    }

    PathState pathState;
    private final Pose startPose = new Pose(0, 0, Math.PI/4);

    private final Pose endPose = new Pose(20, 0, Math.PI/4);

    private Path chain1;

    public void buildPaths() {
        chain1 = line(startPose, endPose).linear(startPose, endPose);
        telemetry.addLine("path built");
    }

    public void updateStates() {
        switch (pathState) {
            case START_POS:
                telemetry.addLine("start pos");
                follower.follow(chain1);
                pathState = PathState.END_POS;
                break;
            case END_POS:
                telemetry.addLine("end pos");
                if (!follower.isBusy()) {
                    follower.stop();
                    //TODO: what if a robot got stuck somewhere and never gets here?
                    telemetry.addLine("finished chain1");
                }
                break;
            default:
                break;
        }
    }
    @Override
    public void init() {
        pathState = PathState.START_POS;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        updateStates();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
    }
}
