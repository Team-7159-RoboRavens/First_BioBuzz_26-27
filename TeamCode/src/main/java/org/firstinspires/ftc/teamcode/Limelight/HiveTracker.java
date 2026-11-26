package org.firstinspires.ftc.teamcode.Limelight;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// Saves the last state read for each CELL of our HIVE, so the autos only read the Limelight a few times a second
public class HiveTracker {
    // Same numbers as friendlyUp()
    public static final int UNKNOWN = calculatePosition.STATE_NONE;
    public static final int MOVING = calculatePosition.STATE_MOVING;
    public static final int DOWN = calculatePosition.STATE_DOWN;
    public static final int UP = calculatePosition.STATE_UP;

    // Seconds between Limelight reads, check() reads right away
    public static double CHECK_INTERVAL = 0.25;
    // A new CELL state has to be read this many frames in a row before it is saved, so one bad frame can't flip it
    public static int STABLE_READS = 2;

    private final Limelight limelight;
    private final calculatePosition tags;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastCheck = -1e9;

    // Index 0 is the CELL at Pedro y < 72, 1 is the CELL at y > 72
    private final int[] saved = {UNKNOWN, UNKNOWN};
    private final double[] savedTime = {-1, -1};
    private final int[] pending = {UNKNOWN, UNKNOWN};
    private final int[] pendingCount = {0, 0};

    // While true every loop reads the Limelight and each new frame is also solved for the robot pose
    private boolean trackPose = false;
    private boolean newPoseFrame = false;
    private Pose framePose = null;

    public HiveTracker(HardwareMap hardwareMap, boolean red) {
        limelight = new Limelight(hardwareMap);
        limelight.useAprilTagPipeline();
        tags = red ? new LimelightRed(limelight, null) : new LimelightBlue(limelight, null);
    }

    // Call every loop, only reads the Limelight every CHECK_INTERVAL unless the pose is tracked
    public void update() {
        if (trackPose || timer.seconds() - lastCheck >= CHECK_INTERVAL) {
            check();
        }
    }

    // Turn on to get a robot pose from every new frame (takePoseFrame / getFramePose), off again to save loop time
    public void setTrackPose(boolean on) {
        trackPose = on;
        newPoseFrame = false;
        framePose = null;
    }

    // True once for each new frame read while the pose is tracked
    public boolean takePoseFrame() {
        boolean fresh = newPoseFrame;
        newPoseFrame = false;
        return fresh;
    }

    // Pedro pose from the last tracked frame, null if its tags didn't give one (not in view, bad fit)
    public Pose getFramePose() {
        return framePose;
    }

    // Reads the Limelight now, call right before deciding where to go
    public void check() {
        lastCheck = timer.seconds();
        limelight.update();
        if (!limelight.isNewFrame()) {
            return;
        }
        tags.updateTagPosition();
        read(0, tags.friendlyCellState(false));
        read(1, tags.friendlyCellState(true));
        if (trackPose) {
            framePose = tags.getPedroPose();
            newPoseFrame = true;
        }
    }

    private void read(int cell, int state) {
        // Tags not in view, keep what was saved
        if (state == UNKNOWN) {
            pendingCount[cell] = 0;
            return;
        }
        if (state == pending[cell]) {
            pendingCount[cell]++;
        } else {
            pending[cell] = state;
            pendingCount[cell] = 1;
        }
        if (pendingCount[cell] >= STABLE_READS) {
            saved[cell] = state;
            savedTime[cell] = timer.seconds();
        }
    }

    // Saved state of the CELL on this side of the field center (Pedro y), UNKNOWN until its tags have been seen
    public int getState(double pedroY) {
        return saved[pedroY > 72 ? 1 : 0];
    }

    public boolean isUp(double pedroY) {
        return getState(pedroY) == UP;
    }

    // Seconds since the state of this CELL was last read, infinite if never
    public double secondsSinceSeen(double pedroY) {
        double t = savedTime[pedroY > 72 ? 1 : 0];
        return t < 0 ? Double.POSITIVE_INFINITY : timer.seconds() - t;
    }

    public static String name(int state) {
        switch (state) {
            case MOVING: return "moving";
            case DOWN: return "down";
            case UP: return "up";
            default: return "unknown";
        }
    }

    public void stop() {
        limelight.stop();
    }
}
