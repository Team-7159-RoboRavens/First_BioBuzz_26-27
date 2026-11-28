package org.firstinspires.ftc.teamcode.Limelight;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
public abstract class calculatePosition {
    static final double INCH = 0.0254;

    // Position of the limelight lens relative to the center of the robot, in meters.
    // +X is toward the front of the robot, +Y is toward the left of the robot.
    //old bot position
//    final double LIMELIGHT_OFFSET_X = 0.1;
//    final double LIMELIGHT_OFFSET_Y = 0;
//    // Height of the limelight lens above the floor, in meters
//    final double LIMELIGHT_HEIGHT = 0.125;
//
//    final double LIMELIGHT_PITCH = 45;
//    final double LIMELIGHT_YAW = 0;

    //working bot position
    final double LIMELIGHT_OFFSET_X = 0.18175;
    final double LIMELIGHT_OFFSET_Y = 0.07039;
    // Height of the limelight lens above the floor, in meters
    final double LIMELIGHT_HEIGHT = 0.23254;

    final double LIMELIGHT_PITCH = 45;
    final double LIMELIGHT_YAW = 0;
    // Trig of the fixed mount angles, worked out once instead of per tag
    private final double LIMELIGHT_PITCH_COS = Math.cos(Math.toRadians(LIMELIGHT_PITCH));
    private final double LIMELIGHT_PITCH_SIN = Math.sin(Math.toRadians(LIMELIGHT_PITCH));
    private final double LIMELIGHT_YAW_RAD = Math.toRadians(LIMELIGHT_YAW);
    final double shooter_offset_y = -0.07646;
    final double shooter_offset_x = 0.21139;
    final boolean TARGET_SPACE_X_IS_VIEWER_RIGHT = true;

    public Limelight limelight;

    public Telemetry telemetry;

    public calculatePosition(Limelight ll, Telemetry telemetry){
        limelight = ll;
        this.telemetry = telemetry;
    }
    //overrid by the color limelights

    //determines if the visible target is on the front side or back side of the field.

    public abstract boolean canSeeTargetFriendly();
    public abstract boolean canSeeTargetEnemy();


    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up
    public abstract int friendlyUp();

    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up
    public abstract int enemyUp();

    //state of one CELL of our HIVE: plusY true for the CELL at field y > 0 (Pedro y > 72), same numbers as friendlyUp()
    public abstract int friendlyCellState(boolean plusY);

    // ---------------- CELL state from the tags ----------------
    public static final int STATE_NONE = 0;
    public static final int STATE_MOVING = 1;
    public static final int STATE_DOWN = 2;
    public static final int STATE_UP = 3;

    //index is tag id - 30, filled in by updateTagPosition()
    boolean[] idVisible = new boolean[16];
    //0 is moving, 1 is down, 2 is up
    int[] idUp = new int[16];

    //reads which HIVE tags are in view and whether their CELL is up, down or moving
    public void updateTagPosition() {
        List<LLResultTypes.FiducialResult> limelightResults = limelight.getFiducialResults();
        if (limelightResults == null) {
            limelightResults = Collections.emptyList();
        }
        for (int i = 0; i < 16; i++) {
            idVisible[i] = false;
        }
        for (LLResultTypes.FiducialResult tag : limelightResults) {
            int id = tag.getFiducialId();
            if (!isHiveTag(id)) {
                continue;
            }
            idVisible[id - 30] = true;
            // 0 is moving, 1 is down, 2 is up
            idUp[id - 30] = getTagHeightState(tag);
        }
    }

    //true if any tag from firstId to lastId is in view
    boolean canSeeTags(int firstId, int lastId) {
        for (int id = firstId; id <= lastId; id++) {
            if (idVisible[id - 30]) {
                return true;
            }
        }
        return false;
    }

    //majority state of the visible tags from firstId to lastId, cellSide +1 / -1 keeps only that CELL, 0 keeps both
    int tagState(int firstId, int lastId, int cellSide) {
        int countUp = 0;
        int countDown = 0;
        int countMoving = 0;
        for (int id = firstId; id <= lastId; id++) {
            if (!idVisible[id - 30] || (cellSide != 0 && getCellSide(id) != cellSide)) {
                continue;
            }
            if (idUp[id - 30] == 0) {
                countMoving++;
            } else if (idUp[id - 30] == 1) {
                countDown++;
            } else {
                countUp++;
            }
        }
        if (countUp + countDown + countMoving == 0) {
            return STATE_NONE;
        }
        if (countUp > countDown && countUp > countMoving) {
            return STATE_UP;
        }
        if (countDown > countUp && countDown > countMoving) {
            return STATE_DOWN;
        }
        // a tie means the tags don't agree, so treat it as moving
        return STATE_MOVING;
    }

    //returs a 3 element array the first value x distance to target second one is y value to target and the third one is an angle relative to target
    public abstract double[] aimState(double[] botPose);

    // ---------------- Shooter aiming ----------------

    // Field walls are 141.233 in apart, centered on the field center
    final double FIELD_WALL = 141.233 / 2 * INCH;
    // Max robot size (18in). The shooting distance is measured with the robot's back flat on the wall, facing the
    // target, with the shooter on the front of the robot lined up with the center of the target
    final double MAX_ROBOT_SIZE = 18 * INCH;

    /**
     * Horizontal distance from the shooter to the target center when the robot's back is flat against the wall
     * behind the target (the wall at y = +-72in) and the shooter on the front of a max size robot is lined up with
     * the target.
     *
     * @param targetY field y of the target center, meters
     * @return shooter to target distance, meters
     */
    double getShootDistance(double targetY){
        return FIELD_WALL - Math.abs(targetY) - MAX_ROBOT_SIZE;
    }

    /**
     * Field heading that points the shooter (not the center of the robot) straight at the target from where the
     * robot is now. The shooter sits shooter_offset_y to the side of the center, so the robot has to turn a little
     * past the target for the shooter's line of fire to pass through it.
     *
     * @param botPose {x, y, heading} of the robot center, meters / radians, field center origin
     * @return field heading in radians, or NaN if the robot is too close to the target to line the shooter up
     */
    double getShooterHeading(double[] botPose, double targetX, double targetY){
        double dx = targetX - botPose[0];
        double dy = targetY - botPose[1];
        double distance = Math.hypot(dx, dy);
        if (distance <= Math.abs(shooter_offset_y)) {
            return Double.NaN;
        }
        // In robot space the target has to end up at y = shooter_offset_y, straight in front of the shooter
        return Math.atan2(dy, dx) - Math.asin(shooter_offset_y / distance);
    }

    /**
     * Where the robot center has to be, and which way it has to face, for the shooter to face the target from
     * getShootDistance(). The robot stays on the same line through the target it is on now, so it only moves
     * closer to / further from the target.
     *
     * @param botPose {x, y, heading} of the robot center, meters / radians, field center origin
     * @return {x, y, heading} of the robot center (meters, meters, radians), or null if the robot is on the target
     */
    double[] getShootPose(double[] botPose, double targetX, double targetY){
        double dx = targetX - botPose[0];
        double dy = targetY - botPose[1];
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-6) {
            return null;
        }
        // With the shooter facing the target, the target is shooterDistance + shooter_offset_x in front of the
        // center and shooter_offset_y to the side, so this is how far the center has to be from the target
        double shooterDistance = getShootDistance(targetY);
        double centerDistance = Math.hypot(shooterDistance + shooter_offset_x, shooter_offset_y);
        double x = targetX - dx / distance * centerDistance;
        double y = targetY - dy / distance * centerDistance;
        double heading = Math.atan2(dy, dx) - Math.asin(shooter_offset_y / centerDistance);
        return new double[] {x, y, heading};
    }

    // ---------------- Position from tag clusters ----------------
    // Every HIVE tag whose CELL state (up / down) is known has a known spot on the field. Instead of trusting each tag's
    // own rotation (noisy on small tags), all visible tags are fit together as one rigid cluster: the camera pose that
    // best lines up where the camera sees the tags with where they are on the field. Any 2+ tags work, so a partly
    // hidden cluster is fine. With only 1 tag it falls back to that tag's own pose.
    // Tags in a cluster have to be at least this far apart (meters), closer tags give a poor heading
    final double CLUSTER_MIN_SPREAD = 0.1;
    // A cluster fit whose average tag miss is above this (meters) is thrown out (a tag's CELL state was read wrong)
    final double CLUSTER_MAX_RESIDUAL = 0.06;
    // Pedro heading = the Limelight / shooter heading + this. The shooter and Limelight face the robot's back
    public static final double PEDRO_HEADING_OFFSET = Math.PI;

    // Cluster points reused by every getPosition() so a solve makes no garbage, they grow if more tags are seen
    private double[] clusterSeenForward = new double[16];
    private double[] clusterSeenLeft = new double[16];
    private double[] clusterFieldX = new double[16];
    private double[] clusterFieldY = new double[16];
    // {x, y, heading} from the last getExpectedCoordinate() call
    private final double[] expectedCoordinate = new double[3];

    //returns {x, y, heading} of the robot on the field (meters, meters, radians, heading is the shooter direction).
    //On failure returns a single element array with the fail type:
    //{0} no results list, {1} no tags visible, {2} no visible tag gave a usable position
    public double[] getPosition(){
        List<LLResultTypes.FiducialResult> limelightResults = limelight.getFiducialResults();
        if (limelightResults == null){
            return new double[]{0};
        }
        if (limelightResults.isEmpty()) {
            return new double[]{1};
        }
        // Cluster points: tag seen from the level camera (forward, left) and the tag on the field (x, y)
        int clusterCount = 0;
        double sumX = 0;
        double sumY = 0;
        // headings are averaged as unit vectors so that +179 and -179 degrees average to 180 instead of 0
        double sumCos = 0;
        double sumSin = 0;
        int count = 0;
        int countFail1 = 0;
        int countFail2 = 0;
        for (LLResultTypes.FiducialResult tag : limelightResults){
            int id = tag.getFiducialId();
            if (!isHiveTag(id)) {
                continue;
            }
            int heightState = getTagHeightState(tag);
            if (heightState == 0) {
                countFail1++;
                continue;
            }
            boolean cellUp = heightState == 2;
            Pose3D tagPoseCamera = tag.getTargetPoseCameraSpace();
            if (tagPoseCamera != null) {
                Position p = tagPoseCamera.getPosition();
                double forward = p.z * LIMELIGHT_PITCH_COS + p.y * LIMELIGHT_PITCH_SIN;
                if (clusterCount == clusterSeenForward.length) {
                    growCluster();
                }
                clusterSeenForward[clusterCount] = forward;
                clusterSeenLeft[clusterCount] = -p.x;
                clusterFieldX[clusterCount] = getTagFieldX(id);
                clusterFieldY[clusterCount] = getTagFieldY(id, cellUp);
                clusterCount++;
            }
            if (!getExpectedCoordinate(tag, cellUp)) {
                countFail2++;
                continue;
            }
            sumX += expectedCoordinate[0];
            sumY += expectedCoordinate[1];
            sumCos += Math.cos(expectedCoordinate[2]);
            sumSin += Math.sin(expectedCoordinate[2]);
            count++;
        }
        if (telemetry != null) {
            telemetry.addData("caculate Position Fail Count 1: ", countFail1);
            telemetry.addData("caculate Position Fail Count 2: ", countFail2);
        }
        double[] cluster = fitCluster(clusterSeenForward, clusterSeenLeft, clusterFieldX, clusterFieldY, clusterCount);
        if (cluster != null) {
            return getRobotCoordinate(cluster[0], cluster[1], cluster[2]);
        }
        if (count == 0) {
            return new double[]{2};
        }
        double limelightX = sumX/count;
        double limelightY = sumY/count;
        double limelightAngle = Math.atan2(sumSin, sumCos);
        return getRobotCoordinate(limelightX, limelightY, limelightAngle);
    }

    // Best fit camera {x, y, heading} on the field for the cluster, or null with too few / too close tags or a bad fit
    double[] fitCluster(List<double[]> seen, List<double[]> field) {
        int n = seen.size();
        double[] seenForward = new double[n];
        double[] seenLeft = new double[n];
        double[] tagX = new double[n];
        double[] tagY = new double[n];
        for (int i = 0; i < n; i++) {
            seenForward[i] = seen.get(i)[0];
            seenLeft[i] = seen.get(i)[1];
            tagX[i] = field.get(i)[0];
            tagY[i] = field.get(i)[1];
        }
        return fitCluster(seenForward, seenLeft, tagX, tagY, n);
    }

    // Same fit on the first n points of plain arrays, used by getPosition() so no lists are made
    private double[] fitCluster(double[] seenForward, double[] seenLeft, double[] tagX, double[] tagY, int n) {
        if (n < 2) {
            return null;
        }
        double seenX = 0, seenY = 0, fieldX = 0, fieldY = 0;
        for (int i = 0; i < n; i++) {
            seenX += seenForward[i];
            seenY += seenLeft[i];
            fieldX += tagX[i];
            fieldY += tagY[i];
        }
        seenX /= n;
        seenY /= n;
        fieldX /= n;
        fieldY /= n;
        double spread = 0;
        double dot = 0;
        double cross = 0;
        for (int i = 0; i < n; i++) {
            double ax = seenForward[i] - seenX;
            double ay = seenLeft[i] - seenY;
            double bx = tagX[i] - fieldX;
            double by = tagY[i] - fieldY;
            spread = Math.max(spread, Math.hypot(bx, by));
            dot += ax * bx + ay * by;
            cross += ax * by - ay * bx;
        }
        // spread is the furthest tag from the middle, so the cluster is about twice that wide
        if (2 * spread < CLUSTER_MIN_SPREAD) {
            return null;
        }
        double heading = Math.atan2(cross, dot);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double cameraX = fieldX - (cos * seenX - sin * seenY);
        double cameraY = fieldY - (sin * seenX + cos * seenY);
        double residual = 0;
        for (int i = 0; i < n; i++) {
            double a0 = seenForward[i];
            double a1 = seenLeft[i];
            residual += Math.hypot(cameraX + cos * a0 - sin * a1 - tagX[i], cameraY + sin * a0 + cos * a1 - tagY[i]);
        }
        residual /= n;
        if (telemetry != null) {
            telemetry.addData("tag cluster", "%d tags, miss %.3f m", n, residual);
        }
        if (residual > CLUSTER_MAX_RESIDUAL) {
            return null;
        }
        return new double[] {cameraX, cameraY, heading};
    }

    // Doubles the cluster point arrays, only if the same tag shows up more than once in a frame
    private void growCluster() {
        int size = clusterSeenForward.length * 2;
        clusterSeenForward = Arrays.copyOf(clusterSeenForward, size);
        clusterSeenLeft = Arrays.copyOf(clusterSeenLeft, size);
        clusterFieldX = Arrays.copyOf(clusterFieldX, size);
        clusterFieldY = Arrays.copyOf(clusterFieldY, size);
    }

    //returns the robot's pose in Pedro Pathing coordinates, or null if getPosition() failed
    public Pose getPedroPose(){
        return toPedroPose(getPosition());
    }

    //Converts a {x, y, heading} array from getPosition() to a Pedro Pathing pose, or null if it is a fail code.
    //Pedro's origin is the bottom left corner
    public static Pose toPedroPose(double[] fieldPosition){
        if (fieldPosition == null || fieldPosition.length < 3) {
            return null;
        }
        double x = fieldPosition[0] / INCH + 72;
        double y = fieldPosition[1] / INCH + 72;
        // Pedro headings go from 0 to 2PI, and Pedro's front is the opposite end from the Limelight
        double heading = (fieldPosition[2] + PEDRO_HEADING_OFFSET) % (2 * Math.PI);
        if (heading < 0) {
            heading += 2 * Math.PI;
        }
        return new Pose(x, y, heading);
    }

    //converts the limelight's field position/heading into the robot center's field position/heading
    private double[] getRobotCoordinate(double limelightX, double limelightY, double limelightAngle) {
        double robotHeading = limelightAngle - LIMELIGHT_YAW_RAD;
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        // rotate the robot-relative lens offset into the field frame and remove it
        double robotX = limelightX - (LIMELIGHT_OFFSET_X * cos - LIMELIGHT_OFFSET_Y * sin);
        double robotY = limelightY - (LIMELIGHT_OFFSET_X * sin + LIMELIGHT_OFFSET_Y * cos);
        return new double[] {robotX, robotY, robotHeading};
    }

    //returns all visiblie ids;
    int[] getVisibleIds(List<LLResultTypes.FiducialResult> limelightResults){
        int size = limelightResults.size();
        int[] visibleTagIds = new int[size];

        for (int i = 0; i < size; i++) {
            visibleTagIds[i] = limelightResults.get(i).getFiducialId();
        }
        return visibleTagIds;
    }

    //true for the 16 hive tags (30-45) that this class knows the field position of
    boolean isHiveTag(int id) {
        return id >= 30 && id <= 45;
    }

    //puts the expected {x, y, heading} of the limelight on the field based on a specific april tag in expectedCoordinate,
    //returns false if the pose is unavailable. cellUp is true if the CELL this tag is on is the raised CELL of its HIVE
    private boolean getExpectedCoordinate(LLResultTypes.FiducialResult tag, boolean cellUp){
        // Camera position in the tag's coordinate system (Limelight "target space"):
        // X is to the right when looking at the tag, Y is toward the bottom edge of the tag, Z points straight out of the tag face.
        Pose3D cameraPoseTag = tag.getCameraPoseTargetSpace();
        // Tag position in the camera's coordinate system: X is right, Y is down, Z is out of the lens.
        Pose3D tagPoseCamera = tag.getTargetPoseCameraSpace();
        if (cameraPoseTag == null || tagPoseCamera == null) {
            return false;
        }
        Position cameraInTag = cameraPoseTag.getPosition();
        if (cameraInTag.z <= 0) {
            // camera can't be behind the tag; bad solve
            return false;
        }

        int id = tag.getFiducialId();
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id, cellUp);
        double tagZ = cellUp ? TAG_HEIGHT_HIGH_STATE : TAG_HEIGHT_LOW_STATE;

        // Directions of the tag's axes on the field. "side" is +1 for the CELL away from the audience and -1 for the
        // audience CELL, and tilt is how far the outer end of the CELL is raised (+30 deg up, -30 deg down).
        double side = getCellSide(id);
        double tiltSin = cellUp ? TILT_UP_SIN : TILT_DOWN_SIN;
        double tiltCos = cellUp ? TILT_UP_COS : TILT_DOWN_COS;
        // Z: out of the tag face, which is the bottom of the CELL (mostly down, leaning outward when up / inward when down)
        double outY = side * tiltSin;
        double outZ = -tiltCos;
        // Y: toward the bottom edge of the tag, which the manual says points toward the center of the field
        double downY = -side * tiltCos;
        double downZ = -tiltSin;
        // X: the tag's right side for someone underneath looking up at it
        double rightX = TARGET_SPACE_X_IS_VIEWER_RIGHT ? -side : side;

        double limelightX = tagX + cameraInTag.x * rightX;
        double limelightY = tagY + cameraInTag.y * downY + cameraInTag.z * outY;
        double limelightZ = tagZ + cameraInTag.y * downZ + cameraInTag.z * outZ;
        // The lens height is fixed, so a solved height far from it means the tag's rotation solve flipped or the HIVE
        // is not resting at its 30 degree tilt. getTagFieldZ() can't catch this because it only uses the tag's position.
        // This is the lens position, so it does not depend on LIMELIGHT_PITCH.
        if (Math.abs(limelightZ - LIMELIGHT_HEIGHT) > ACCEPTABLE_CAMERA_HEIGHT_OFFSET) {
            if (telemetry != null) {
                telemetry.addData("tag " + id + " rejected, limelight height off by: ", limelightZ - LIMELIGHT_HEIGHT);
            }
            return false;
        }

        // Horizontal bearing to the tag as seen by the camera (positive = tag is to the right of center).
        // The camera's down/forward axes are tilted by the mount pitch, so level them before taking the angle.
        Position tagInCamera = tagPoseCamera.getPosition();
        double forwardLevel = tagInCamera.z * LIMELIGHT_PITCH_COS + tagInCamera.y * LIMELIGHT_PITCH_SIN;
        double bearingRight = Math.atan2(tagInCamera.x, forwardLevel);

        // The camera is pointed bearingRight to the left of the line from the camera to the tag
        double headingToTag = Math.atan2(tagY - limelightY, tagX - limelightX);
        double cameraHeadingRad = headingToTag + bearingRight;

        expectedCoordinate[0] = limelightX;
        expectedCoordinate[1] = limelightY;
        expectedCoordinate[2] = cameraHeadingRad;
        return true;
    }

    // ---------------- HIVE geometry (manual section 9.6 / 9.9) ----------------

    // HIVE Center to Center is 25.5in, so each HIVE is 12.75in from the center line (red at -X, blue at +X)
    final double HIVE_CENTER_X = 12.75 * INCH;
    // Pivot axis height above the TILES
    final double PIVOT_HEIGHT = 43.95 * INCH;
    // Each HIVE rests tipped 30 degrees one way or the other
    final double HIVE_TILT = 30;
    // Where the tag centers are on the HIVE, measured from the pivot with the HIVE level:
    // along the HIVE toward the CELL, and below the pivot (the bottom face of the CELL is just under the pivot).
    // Manual: CELLS are 18.84in apart and 12.04in deep, tag centers are 9.938 - 2.75 = 7.19in from the front of the CELL,
    // giving ~14.27in along the HIVE. These two values reproduce the 0.894549m low height measured earlier exactly.
    final double TAG_ALONG_HIVE = 14.25 * INCH;
    final double TAG_BELOW_PIVOT = 1.854 * INCH;

    // Sticker offsets from the CELL center line (the HIVE arm) to the individual tag centers (manual Figure 9-15)
    final double OUTER_OFFSET = 6.5 * INCH;
    final double INNER_OFFSET = 2.75 * INCH;

    // Heights of the tag centers above the floor when their CELL is down / up, in meters (~35.2in / ~49.5in)
    final double TAG_HEIGHT_LOW_STATE = PIVOT_HEIGHT
            - TAG_ALONG_HIVE * Math.sin(Math.toRadians(HIVE_TILT)) - TAG_BELOW_PIVOT * Math.cos(Math.toRadians(HIVE_TILT));
    final double TAG_HEIGHT_HIGH_STATE = PIVOT_HEIGHT
            + TAG_ALONG_HIVE * Math.sin(Math.toRadians(HIVE_TILT)) - TAG_BELOW_PIVOT * Math.cos(Math.toRadians(HIVE_TILT));

    // Trig of the HIVE tilt for a raised / lowered CELL, worked out once instead of per tag
    private final double TILT_UP_SIN = Math.sin(Math.toRadians(HIVE_TILT));
    private final double TILT_UP_COS = Math.cos(Math.toRadians(HIVE_TILT));
    private final double TILT_DOWN_SIN = Math.sin(-Math.toRadians(HIVE_TILT));
    private final double TILT_DOWN_COS = Math.cos(-Math.toRadians(HIVE_TILT));
    // How far the tags sit from the center line when their CELL is up / down, before the side sign
    private final double TAG_ALONG_UP = TAG_ALONG_HIVE * Math.cos(Math.toRadians(HIVE_TILT))
            + TAG_BELOW_PIVOT * Math.sin(Math.toRadians(HIVE_TILT));
    private final double TAG_ALONG_DOWN = TAG_ALONG_HIVE * Math.cos(Math.toRadians(HIVE_TILT))
            + (-TAG_BELOW_PIVOT) * Math.sin(Math.toRadians(HIVE_TILT));

    //+1 for the CELL on the side of the field away from the audience, -1 for the audience side CELL
    private double getCellSide(int id) {
        if ((id >= 30 && id <= 33) || (id >= 42 && id <= 45)) {
            return 1;
        }
        return -1;
    }

    //left/right position of the tag (manual Figure 9-17 shows the order looking down at the field from above)
    private double getTagFieldX(int id) {
        final double RED = -HIVE_CENTER_X;
        final double BLUE = HIVE_CENTER_X;

        switch(id) {
            // RED SCORING TAGS (away from audience), left to right: 33 32 31 30
            case 33: return RED - OUTER_OFFSET;
            case 32: return RED - INNER_OFFSET;
            case 31: return RED + INNER_OFFSET;
            case 30: return RED + OUTER_OFFSET;

            // RED AUDIENCE TAGS, left to right: 34 35 36 37
            case 34: return RED - OUTER_OFFSET;
            case 35: return RED - INNER_OFFSET;
            case 36: return RED + INNER_OFFSET;
            case 37: return RED + OUTER_OFFSET;

            // BLUE SCORING TAGS (away from audience), left to right: 45 44 43 42
            case 45: return BLUE - OUTER_OFFSET;
            case 44: return BLUE - INNER_OFFSET;
            case 43: return BLUE + INNER_OFFSET;
            case 42: return BLUE + OUTER_OFFSET;

            // BLUE AUDIENCE TAGS, left to right: 38 39 40 41
            case 38: return BLUE - OUTER_OFFSET;
            case 39: return BLUE - INNER_OFFSET;
            case 40: return BLUE + INNER_OFFSET;
            case 41: return BLUE + OUTER_OFFSET;

            default: return 0.0;
        }
    }

    //distance of the tag from the center line toward/away from the audience; the raised CELL's tags sit a little further out
    private double getTagFieldY(int id, boolean cellUp) {
        return getCellSide(id) * (cellUp ? TAG_ALONG_UP : TAG_ALONG_DOWN);
    }

    //in meters how far above or below limeight can think the target is from a stable state
    final double ACCEPTABLE_OFFSET = 0.15;

    //in meters how far the limelight height solved from a tag can be from LIMELIGHT_HEIGHT before the tag is ignored.
    //A good solve can be off by up to ACCEPTABLE_OFFSET (the same error seen from the other side), so don't go below it
    //without checking the "limelight height" telemetry first
    final double ACCEPTABLE_CAMERA_HEIGHT_OFFSET = 0.15;

    //returns null if the tag is moving aka not in a stable state
    Double getTagFieldZ(LLResultTypes.FiducialResult limelightResults){
        int state = getTagHeightState(limelightResults);
        if (state == 0) {
            return null;
        }
        return state == 2 ? TAG_HEIGHT_HIGH_STATE : TAG_HEIGHT_LOW_STATE;
    }

    //same check as getTagFieldZ() without boxing a Double: 0 if moving (or no pose), 1 if at the low height, 2 if high
    private int getTagHeightState(LLResultTypes.FiducialResult limelightResults){
        // Tag position relative to the camera lens: X right, Y down, Z out of the lens
        Pose3D targetPoseRaw = limelightResults.getTargetPoseCameraSpace();
        if (targetPoseRaw == null){
            return 0;
        }
        Position rawPosition = targetPoseRaw.getPosition();

        // The camera is tilted up by LIMELIGHT_PITCH, so straight up is a mix of the camera's -Y and +Z
        double heightAboveCamera = -rawPosition.y * LIMELIGHT_PITCH_COS + rawPosition.z * LIMELIGHT_PITCH_SIN;
        double calculatedTagFieldZ = LIMELIGHT_HEIGHT + heightAboveCamera;

        // Check which of the two stable states the measurement is closer to
        double distanceToLowState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_LOW_STATE);
        double distanceToHighState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_HIGH_STATE);

        if (distanceToLowState < distanceToHighState) {
            if (distanceToLowState > ACCEPTABLE_OFFSET){
                return 0;
            }
            return 1;
        } else {
            if (distanceToHighState > ACCEPTABLE_OFFSET){
                return 0;
            }
            return 2;
        }
    }

}
