package org.firstinspires.ftc.teamcode.Autonomous;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

// Forward 10 in, right 10 in, back 10 in (starting pose is heading 0: forward is +x, right is -y)
@Autonomous(name = "SimplePark")
public class SimplePark extends OpMode {
    private static final double DISTANCE = 10.0; // inches

    private Follower follower;
    private final Pose startPose = new Pose(72, 72, 0);

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        Pose forward = new Pose(startPose.x() + DISTANCE, startPose.y(), startPose.heading());
        Pose right = new Pose(forward.x(), forward.y() - DISTANCE, startPose.heading());
        Pose back = new Pose(right.x() - DISTANCE, right.y(), startPose.heading());

        // Three straight segments, heading held constant
        Path route = path(
                line(startPose, forward).constant(startPose.heading()),
                line(forward, right).constant(startPose.heading()),
                line(right, back).constant(startPose.heading()));
        follower.follow(route);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
    }
}
