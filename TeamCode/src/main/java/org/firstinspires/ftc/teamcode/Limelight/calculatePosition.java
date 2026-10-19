package org.firstinspires.ftc.teamcode.Limelight;

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
    final double LIMELIGHT_OFFSET_X = 0;
    final double LIMELIGHT_OFFSET_Y = 0.1;
    // Height of the limelight lens above the floor, in meters
    final double LIMELIGHT_HEIGHT = 0.125;

    final double LIMELIGHT_PITCH = 45;
    final double LIMELIGHT_YAW = 0;

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
    public abstract void updateTagPosition();

    //returs a 3 element array the first value x distance to target second one is y value to target and the third one is an angle relative to target
    public abstract double[] aimState(double[] botPose);

    //returns {x, y, heading} of the robot on the field (meters, meters, radians).
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
        double sumX = 0;
        double sumY = 0;
        // headings are averaged as unit vectors so that +179 and -179 degrees average to 180 instead of 0
        double sumCos = 0;
        double sumSin = 0;
        int count = 0;
        int countFail1 = 0;
        int countFail2 = 0;
        for (LLResultTypes.FiducialResult tag : limelightResults){
            if (!isHiveTag(tag.getFiducialId())) {
                continue;
            }
            Double tagZ = getTagFieldZ(tag);
            if (tagZ == null) {
                countFail1++;
                continue;
            }
            double[] coordinate = getExpectedCoordinate(tag, tagZ == TAG_HEIGHT_HIGH_STATE);
            if (coordinate == null) {
                countFail2++;
                continue;
            }
            sumX += coordinate[0];
            sumY += coordinate[1];
            sumCos += Math.cos(coordinate[2]);
            sumSin += Math.sin(coordinate[2]);
            count++;
        }
        telemetry.addData("caculate Position Fail Count 1: ", countFail1);
        telemetry.addData("caculate Position Fail Count 2: ", countFail2);
        if (count == 0) {
            return new double[]{2};
        }
        double limelightX = sumX/count;
        double limelightY = sumY/count;
        double limelightAngle = Math.atan2(sumSin, sumCos);
        return getRobotCoordinate(limelightX, limelightY, limelightAngle);
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
        // Pedro headings go from 0 to 2PI
        double heading = fieldPosition[2] % (2 * Math.PI);
        if (heading < 0) {
            heading += 2 * Math.PI;
        }
        return new Pose(x, y, heading);
    }

    //converts the limelight's field position/heading into the robot center's field position/heading
    private double[] getRobotCoordinate(double limelightX, double limelightY, double limelightAngle) {
        double robotHeading = limelightAngle - Math.toRadians(LIMELIGHT_YAW);
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

    //returns the expected {x, y, heading} of the limelight on the field based on a specific april tag, or null if the pose is unavailable
    //cellUp is true if the CELL this tag is on is the raised CELL of its HIVE
    private double[] getExpectedCoordinate(LLResultTypes.FiducialResult tag, boolean cellUp){
        // Camera position in the tag's coordinate system (Limelight "target space"):
        // X is to the right when looking at the tag, Y is toward the bottom edge of the tag, Z points straight out of the tag face.
        Pose3D cameraPoseTag = tag.getCameraPoseTargetSpace();
        // Tag position in the camera's coordinate system: X is right, Y is down, Z is out of the lens.
        Pose3D tagPoseCamera = tag.getTargetPoseCameraSpace();
        if (cameraPoseTag == null || tagPoseCamera == null) {
            return null;
        }
        Position cameraInTag = cameraPoseTag.getPosition();
        if (cameraInTag.z <= 0) {
            // camera can't be behind the tag; bad solve
            return null;
        }

        int id = tag.getFiducialId();
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id, cellUp);
        double tagZ = cellUp ? TAG_HEIGHT_HIGH_STATE : TAG_HEIGHT_LOW_STATE;

        // Directions of the tag's axes on the field. "side" is +1 for the CELL away from the audience and -1 for the
        // audience CELL, and tilt is how far the outer end of the CELL is raised (+30 deg up, -30 deg down).
        double side = getCellSide(id);
        double tilt = cellUp ? Math.toRadians(HIVE_TILT) : -Math.toRadians(HIVE_TILT);
        // Z: out of the tag face, which is the bottom of the CELL (mostly down, leaning outward when up / inward when down)
        double outY = side * Math.sin(tilt);
        double outZ = -Math.cos(tilt);
        // Y: toward the bottom edge of the tag, which the manual says points toward the center of the field
        double downY = -side * Math.cos(tilt);
        double downZ = -Math.sin(tilt);
        // X: the tag's right side for someone underneath looking up at it
        double rightX = TARGET_SPACE_X_IS_VIEWER_RIGHT ? -side : side;

        double limelightX = tagX + cameraInTag.x * rightX;
        double limelightY = tagY + cameraInTag.y * downY + cameraInTag.z * outY;
        double limelightZ = tagZ + cameraInTag.y * downZ + cameraInTag.z * outZ;
        telemetry.addData("tag " + id + " limelight height: ", limelightZ);

        // Horizontal bearing to the tag as seen by the camera (positive = tag is to the right of center).
        // The camera's down/forward axes are tilted by the mount pitch, so level them before taking the angle.
        Position tagInCamera = tagPoseCamera.getPosition();
        double pitchRad = Math.toRadians(LIMELIGHT_PITCH);
        double forwardLevel = tagInCamera.z * Math.cos(pitchRad) + tagInCamera.y * Math.sin(pitchRad);
        double bearingRight = Math.atan2(tagInCamera.x, forwardLevel);

        // The camera is pointed bearingRight to the left of the line from the camera to the tag
        double headingToTag = Math.atan2(tagY - limelightY, tagX - limelightX);
        double cameraHeadingRad = headingToTag + bearingRight;

        return new double[] {limelightX, limelightY, cameraHeadingRad};
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
        double tilt = Math.toRadians(HIVE_TILT);
        double along = TAG_ALONG_HIVE * Math.cos(tilt)
                + (cellUp ? TAG_BELOW_PIVOT : -TAG_BELOW_PIVOT) * Math.sin(tilt);
        return getCellSide(id) * along;
    }

    //in meters how far above or below limeight can think the target is from a stable state
    final double ACCEPTABLE_OFFSET = 0.15;

    //returns null if the tag is moving aka not in a stable state
    Double getTagFieldZ(LLResultTypes.FiducialResult limelightResults){
        // Tag position relative to the camera lens: X right, Y down, Z out of the lens
        Pose3D targetPoseRaw = limelightResults.getTargetPoseCameraSpace();
        if (targetPoseRaw == null){
            return null;
        }
        Position rawPosition = targetPoseRaw.getPosition();

        // The camera is tilted up by LIMELIGHT_PITCH, so straight up is a mix of the camera's -Y and +Z
        double pitchRad = Math.toRadians(LIMELIGHT_PITCH);
        double heightAboveCamera = -rawPosition.y * Math.cos(pitchRad) + rawPosition.z * Math.sin(pitchRad);
        double calculatedTagFieldZ = LIMELIGHT_HEIGHT + heightAboveCamera;

        // Check which of the two stable states the measurement is closer to
        double distanceToLowState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_LOW_STATE);
        double distanceToHighState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_HIGH_STATE);

        telemetry.addData("vertical height: ", calculatedTagFieldZ);
        if (distanceToLowState < distanceToHighState) {
            if (distanceToLowState > ACCEPTABLE_OFFSET){
                return null;
            }
            return TAG_HEIGHT_LOW_STATE;
        } else {
            if (distanceToHighState > ACCEPTABLE_OFFSET){
                return null;
            }
            return TAG_HEIGHT_HIGH_STATE;
        }
    }

}
