package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import java.util.List;

import static com.pedropathing.api.Paths.line;

// Starts in the corner of the park wall and the side wall facing away from the park wall,
// then drives in a straight line to a park spot PARK_OFF_WALL in off the park wall
public abstract class SimpleParkBase extends OpMode {
    // Robot is square, 2.5 in under the 18 in max
    protected static final double ROBOT_SIZE = 18 - 2.5;
    // Inner walls are about 141 in apart, centered in Pedro's 144 in grid, so each wall is 1.5 in in from the grid edge
    protected static final double WALL_INSET = 1.5;
    // Gap between the robot and the park wall once parked, inches
    protected static final double PARK_OFF_WALL = 3;
    // Park spots along the park wall, same y as the Blue three cycle park poses
    protected static final double LEFT_PARK_Y = 15.5;
    protected static final double RIGHT_PARK_Y = 45.2051282051282;

    // Blue starts in the corner of the x = 144 (park) wall and the y = 0 wall, facing -x
    protected static final Pose BLUE_START = new Pose(
            144 - WALL_INSET - ROBOT_SIZE / 2,
            WALL_INSET + ROBOT_SIZE / 2,
            Math.toRadians(180));
    protected static final Pose BLUE_LEFT_PARK = bluePark(LEFT_PARK_Y);
    protected static final Pose BLUE_RIGHT_PARK = bluePark(RIGHT_PARK_Y);
    // Red is Blue rotated 180 deg about (72, 72)
    protected static final Pose RED_START = toRed(BLUE_START);
    protected static final Pose RED_LEFT_PARK = toRed(BLUE_LEFT_PARK);
    protected static final Pose RED_RIGHT_PARK = toRed(BLUE_RIGHT_PARK);

    private static Pose bluePark(double y) {
        return new Pose(144 - WALL_INSET - ROBOT_SIZE / 2 - PARK_OFF_WALL, y, Math.toRadians(180));
    }

    private static Pose toRed(Pose p) {
        return new Pose(144 - p.x(), 144 - p.y(), p.heading() - Math.PI);
    }

    private Follower follower;
    private List<LynxModule> hubs;

    protected abstract Pose getStartPose();
    protected abstract Pose getParkPose();

    @Override
    public void init() {
        // MANUAL bulk caching, cleared once at the top of every loop
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(getStartPose());
        TeleOpData.autoEndPose = getStartPose();
    }

    @Override
    public void start() {
        Pose start = getStartPose();
        follower.follow(line(start, getParkPose()).constant(start.heading()));
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        Pose pose = follower.pose();
        TeleOpData.autoEndPose = pose;
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Teleop starts its field position from here
        TeleOpData.autoEndPose = follower.pose();
        follower.stop();
    }
}
