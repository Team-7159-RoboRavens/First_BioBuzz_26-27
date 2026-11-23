package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import static com.pedropathing.api.Paths.line;

// Starts in the corner next to the park wall facing away from it, then strafes right into the park
public abstract class SimpleParkBase extends OpMode {
    // How far the robot strafes right, inches
    protected static final double RIGHT_DISTANCE = 6.5;
    // Robot is square, 2.5 in under the 18 in max
    protected static final double ROBOT_SIZE = 18 - 2.5;
    // How far the robot is pushed forward off the park wall at the start, inches
    protected static final double START_OFF_WALL = 3;
    // Inner walls are about 141 in apart, centered in Pedro's 144 in grid, so each wall is 1.5 in in from the grid edge
    protected static final double WALL_INSET = 1.5;

    // Blue starts in the corner of the x = 144 (park) wall and the y = 0 wall, facing -x
    protected static final Pose BLUE_START = new Pose(
            144 - WALL_INSET - ROBOT_SIZE / 2 - START_OFF_WALL,
            WALL_INSET + ROBOT_SIZE / 2,
            Math.toRadians(180));
    // Red is Blue rotated 180 deg about (72, 72)
    protected static final Pose RED_START = new Pose(144 - BLUE_START.x(), 144 - BLUE_START.y(), 0);

    private Follower follower;

    protected abstract Pose getStartPose();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(getStartPose());
        TeleOpData.autoEndPose = getStartPose();
    }

    @Override
    public void start() {
        Pose start = getStartPose();
        // Robot right is the heading turned 90 deg clockwise
        double heading = start.heading();
        Pose end = new Pose(
                start.x() + Math.sin(heading) * RIGHT_DISTANCE,
                start.y() - Math.cos(heading) * RIGHT_DISTANCE,
                heading);
        follower.follow(line(start, end).constant(heading));
    }

    @Override
    public void loop() {
        follower.update();
        TeleOpData.autoEndPose = follower.pose();
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Teleop starts its field position from here
        TeleOpData.autoEndPose = follower.pose();
        follower.stop();
    }
}
