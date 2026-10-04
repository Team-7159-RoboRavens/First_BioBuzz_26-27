package org.firstinspires.ftc.teamcode.Limelight;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public abstract class calculatePosition {
    final double LIMELIGHT_OFFSET_X = 0;
    final double LIMELIGHT_OFFSET_Y = 0;
    final double LIMELIGHT_HEIGHT = 0;

    final double LIMELIGHT_OFFSET_DISTANCE = Math.sqrt(Math.pow(LIMELIGHT_OFFSET_X, 2) + Math.pow(LIMELIGHT_OFFSET_Y, 2));


    //rotation of the limelight on the x-axis of the robot relative to the ground
    final double LIMELIGHT_OFFSET_X_ANGLE = 90;
    final double LIMELIGHT_OFFSET_Z_ANGLE = 0;

    public Limelight limelight;

    public Telemetry telemetry;

    public calculatePosition(Limelight ll, Telemetry telemetry){
        limelight = ll;
        this.telemetry = telemetry;
    }
    //overid by the color limelights

    //determines if the visible target is on the front side or back side of the field.

    public abstract boolean canSeeTargetFriendly();
    public abstract boolean canSeeTargetEnemy();


    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up
    public abstract int friendlyUp();

    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up
    public abstract int enemyUp();
    public abstract void updateTagPosition();

    //returns 2d coordinate of robot in field
    public double[] getPosition(){
        List<LLResultTypes.FiducialResult> limelightResults = limelight.getFiducialResults();
        if (limelightResults == null){
            return new double[]{0};
        }
        int length = limelightResults.size();
        if (length == 0) {
            return new double[]{1};
        }
        //list of coordinates x, y, anlge z axis
        List<double[]> coordinates = new ArrayList<>();
        int countFail1 = 0;
        int countFail2 = 0;
        for (int i = 0; i < length; i++){
            Double tagZ = getTagFieldZ(limelightResults.get(i));
            if (tagZ != null) {
                double[] coordinate = getExpectedCoordinate(tagZ, limelightResults.get(i));
                if (coordinate != null) {
                    coordinates.add(coordinate);
                }
                else{
                    countFail2++;
                }
            }
            else {
                countFail1 ++;
            }
        }
        telemetry.addData("caculate Position Fail Count 1: ", countFail1);
        telemetry.addData("caculate Position Fail Count 2: ", countFail2);
        double sumX = 0;
        double sumY = 0;
        double sumAngle = 0;
        double count = coordinates.size();
        if (count == 0) {
            return new double[]{2};
        }
        for (double[] coordinate : coordinates){
            sumX += coordinate[0];
            sumY += coordinate[1];
            sumAngle += coordinate[2];
        }
        double limelightX = sumX/count;
        double limelightY = sumY/count;
        double limelightAngle = sumAngle/count;
        return getRobotCoordinate(limelightX, limelightY, limelightAngle);
    }

    private double[] getRobotCoordinate(double limelightX, double limelightY, double limelightAngle) {
        double robotAngleZ = limelightAngle + Math.toRadians(LIMELIGHT_OFFSET_Z_ANGLE);
        double robotX = limelightX - LIMELIGHT_OFFSET_DISTANCE * Math.cos(robotAngleZ);
        double robotY = limelightY - LIMELIGHT_OFFSET_DISTANCE * Math.sin(robotAngleZ);
        return new double[] {robotX, robotY, robotAngleZ};
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

    //returns the expected position of the limelight on the field based on a specific april tag;
    private double[] getExpectedCoordinate(double tagZ, LLResultTypes.FiducialResult limelightResults){

        Pose3D targetPoseRaw = limelightResults.getTargetPoseCameraSpace();
        if (targetPoseRaw == null) {
            return null;
        }

        int id = limelightResults.getFiducialId();

        // Define txRad using the Limelight horizontal offset converted to radians
        double txRad = Math.toRadians(limelightResults.getTargetXDegrees());

        // 1. EXTRACT 3D TRANSLATION FROM CAMERA TO TAG
        // In Limelight Camera Space: X is Right, Y is Down, Z is Forward
        Position rawPosition = targetPoseRaw.getPosition();
        double camX = rawPosition.x;
        double camY = rawPosition.y;
        double camZ = rawPosition.z;


        double cameraPitchRad = Math.toRadians(LIMELIGHT_OFFSET_X_ANGLE);
        double cameraRollRad  = Math.toRadians(0.0);
        double cameraYawRad   = Math.toRadians(LIMELIGHT_OFFSET_Z_ANGLE);

        // Precompute sines and cosines for the manual 3D rotation matrix calculation
        double cosP = Math.cos(cameraPitchRad);
        double sinP = Math.sin(cameraPitchRad);
        double cosY = Math.cos(cameraYawRad);
        double sinY = Math.sin(cameraYawRad);
        double cosR = Math.cos(cameraRollRad);
        double sinR = Math.sin(cameraRollRad);

        // Rotate the raw camera translation vector to align with the robot's horizontal frame of reference.
        double translationRobotToTagX = (cosY * cosP) * camX
                + (cosY * sinP * sinR - sinY * cosR) * camY
                + (cosY * sinP * cosR + sinY * sinR) * camZ;

        double translationRobotToTagY = (sinY * cosP) * camX
                + (sinY * sinP * sinR + cosY * cosR) * camY
                + (sinY * sinP * cosR - cosY * sinR) * camZ;

        // Even if tilted, the actual horizontal distance on the floor is the hypotenuse of the corrected X and Z.
        // (Assuming standard robotics conventions where X is forward and Y is left after rotation alignment)
        double groundDistance = Math.hypot(translationRobotToTagX, translationRobotToTagY);

        // Tags physical position on the field
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id);
        double tagHeadingRad = Math.toRadians(getTagFieldHeading(id));

        // Extract the rotation around the vertical Z-axis
        double skewZRad = targetPoseRaw.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        // Absolute angle of the path to the tag relative to the field
        double pathHeadingRad = skewZRad + tagHeadingRad;

        //absolute heading of the limelight on the z axis
        double cameraHeadingRad = pathHeadingRad + txRad;



        // 5. Project outwards from the tag face to find where the camera is standing
        double limelightX = tagX + (groundDistance * Math.cos(pathHeadingRad));
        double limelightY = tagY + (groundDistance * Math.sin(pathHeadingRad));

        return new double[] {limelightX, limelightY, cameraHeadingRad};
    }

    //ground distance to tag ty is degrees the target is offset along the vertical axis from the center of limelights vision
    private double getGroundDistanceToTag(double ty, int id, double tagHeight) {

        double heightDifference = tagHeight - LIMELIGHT_HEIGHT;
        double totalAngleRadians = Math.toRadians(LIMELIGHT_OFFSET_X_ANGLE + ty);

        // Ground distance (d)
        return heightDifference / Math.tan(totalAngleRadians);
    }
    private double getTagFieldX(int id) {
        // Exact center locations of the two Hive Cell assemblies on the X-axis
        final double LEFT_CELL_CENTER = -0.6768846;
        final double RIGHT_CELL_CENTER = 0.6768846;

        // Precise sticker offsets from the cell center to individual tag CENTERS
        final double OUTER_OFFSET = 5.5 * 0.0254;  // 0.1397 meters
        final double INNER_OFFSET = 3.25 * 0.0254; // 0.08255 meters

        switch(id) {
            // RED ALLIANCE HIVES
            case 30: return LEFT_CELL_CENTER - OUTER_OFFSET;
            case 31: return LEFT_CELL_CENTER - INNER_OFFSET;
            case 32: return LEFT_CELL_CENTER + INNER_OFFSET;
            case 33: return LEFT_CELL_CENTER + OUTER_OFFSET;

            case 34: return LEFT_CELL_CENTER - OUTER_OFFSET;
            case 35: return LEFT_CELL_CENTER - INNER_OFFSET;
            case 36: return LEFT_CELL_CENTER + INNER_OFFSET;
            case 37: return LEFT_CELL_CENTER + OUTER_OFFSET;

            // BLUE ALLIANCE HIVES
            case 38: return RIGHT_CELL_CENTER + OUTER_OFFSET;
            case 39: return RIGHT_CELL_CENTER + INNER_OFFSET;
            case 40: return RIGHT_CELL_CENTER - INNER_OFFSET;
            case 41: return RIGHT_CELL_CENTER - OUTER_OFFSET;

            case 42: return RIGHT_CELL_CENTER + OUTER_OFFSET;
            case 43: return RIGHT_CELL_CENTER + INNER_OFFSET;
            case 44: return RIGHT_CELL_CENTER - INNER_OFFSET;
            case 45: return RIGHT_CELL_CENTER - OUTER_OFFSET;

            default: return 0.0;
        }
    }

    private double getTagFieldY(int id) {
        // Exact distance from center of field to the center of the tags depends on side of the field not color
        final double FORWARD_Y = 0.289960;
        final double BACK_Y = -0.289960;

        switch(id) {
            case 30: case 31: case 32: case 33:
            case 42: case 43: case 44: case 45:
                return FORWARD_Y;

            case 38: case 39: case 40: case 41:
            case 34: case 35: case 36: case 37:
                return BACK_Y;

            default: return 0.0;
        }
    }
    final double TAG_HEIGHT_LOW_STATE = 0.894549 - LIMELIGHT_HEIGHT;
    final double TAG_HEIGHT_HIGH_STATE = 1.338096 - LIMELIGHT_HEIGHT;

    //in meters how far above or below limeight can think the target is from a stable state
    final double EXCEPTABLE_OFFSET = 0.15;

    //returns null if the tag is moving aka not in a stable state
    Double getTagFieldZ(LLResultTypes.FiducialResult limelightResults){
        //the target pose relative to the camera lens
        Pose3D targetPoseRaw = limelightResults.getTargetPoseCameraSpace();
        if (targetPoseRaw == null){
            return null;
        }
        Position rawPosition = targetPoseRaw.getPosition();

        // In Limelight Camera Space, index 1 is Y (Vertical distance).
        // Because Camera Y points DOWN, a tag above the camera has a negative Y value.
        double targetCamZ = -rawPosition.y;
        //forward distance to the tag
        double targetCamY = rawPosition.z;
        if (targetCamY < 0){
            targetCamY = -targetCamY;
        }

        double targetXOffSet = limelight.getTargetYOffset();

        // 3. Extrapolate what the tag's field height must be based on where the camera sees it
        double calculatedTagFieldZ;
        if (LIMELIGHT_OFFSET_X_ANGLE < 90) {
            double calculatedTagFieldDistance1 = targetCamY / Math.cos(Math.toRadians(LIMELIGHT_OFFSET_X_ANGLE));
            double calculatedTagFieldDistance2 = targetCamZ / Math.sin(Math.toRadians(LIMELIGHT_OFFSET_X_ANGLE));
            calculatedTagFieldZ = (calculatedTagFieldDistance1 + calculatedTagFieldDistance2)/2 * Math.cos(Math.toRadians(90-LIMELIGHT_OFFSET_X_ANGLE));
        }
        else {
            calculatedTagFieldZ = targetCamZ;
        }

        // 4. Check which of the two stable states the measurement is closer to
        double distanceToLowState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_LOW_STATE);
        double distanceToHighState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_HIGH_STATE);

        telemetry.addData("vertical height: ", calculatedTagFieldZ);
        if (distanceToLowState < distanceToHighState) {

            if (distanceToLowState > EXCEPTABLE_OFFSET){
                return null;
            }
            return TAG_HEIGHT_LOW_STATE;
        } else {
            if (distanceToHighState > EXCEPTABLE_OFFSET){
                return null;
            }
            return TAG_HEIGHT_HIGH_STATE;
        }
    }
    //angle of the tag on the z axis
    private double getTagFieldHeading(int id){
        return 0;
    }

}
