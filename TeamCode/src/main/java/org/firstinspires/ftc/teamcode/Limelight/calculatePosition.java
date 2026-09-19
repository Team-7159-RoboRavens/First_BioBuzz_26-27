package org.firstinspires.ftc.teamcode.Limelight;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public abstract class calculatePosition {
    final double LIMELIGHT_OFFSET_X = 0;
    final double LIMELIGHT_OFFSET_Y = 0;
    final double LIMELIGHT_HEIGHT = 0;

    final double LIMELIGHT_OFFSET_DISTANCE = Math.sqrt(Math.pow(LIMELIGHT_OFFSET_X, 2) + Math.pow(LIMELIGHT_OFFSET_Y, 2));


    //rotation of the limelight on the x-axis of the robot relative to the ground
    final double LIMELIGHT_OFFSET_X_ANGLE = 90;
    final double LIMELIGHT_OFFSET_Z_ANGLE = 90;

    double robotAngleZ;
    public Limelight limelight;

    public calculatePosition(Limelight ll){
        limelight = ll;
    }
    //overid by the color limelights

    //determines if the visible target is on the front side or back side of the field.
    public abstract boolean viewingFrontSideFriendly();
    public abstract boolean viewingFrontSideEnemy();
    public abstract boolean canSeeTargetFriendly();
    public abstract boolean canSeeTargetEnemy();
    public abstract boolean friendlyUp();
    public abstract boolean enemyUp();

    //returns 2d coordinate of robot in field
    public double[] getPosition(){
        List<LLResultTypes.FiducialResult> limelightResults = limelight.getFiducialResults();
        int length = limelightResults.size();
        if (length == 0) {
            return new double[]{};
        }
        //list of coordinates x, y, anlge z axis
        List<double[]> coordinates = new ArrayList<>();
        for (int i = 0; i < length; i++){
            Double tagZ = getTagFieldZ(limelightResults.get(i));
            if (tagZ != null) {
                coordinates.add(getExpectedCoordinate(tagZ, limelightResults.get(i)));
            }
        }
        double sumX = 0;
        double sumY = 0;
        double sumAngle = 0;
        double count = coordinates.size();
        if (count == 0) {
            return new double[]{};
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
        double robotX = limelightX - LIMELIGHT_OFFSET_DISTANCE * Math.cos(robotAngleZ + LIMELIGHT_OFFSET_Z_ANGLE);
        double robotY = limelightY - LIMELIGHT_OFFSET_DISTANCE * Math.sin(robotAngleZ + LIMELIGHT_OFFSET_Z_ANGLE);
        return new double[] {robotX, robotY};
    }

    //returns all visiblie ids;
    private int[] getVisibleIds(List<LLResultTypes.FiducialResult> limelightResults){
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

        // 1. EXTRACT 3D TRANSLATION FROM CAMERA TO TAG
        // In Limelight Camera Space: X is Right, Y is Down, Z is Forward
        Position rawPosition = targetPoseRaw.getPosition();
        Translation3d translationCamToTag = new Translation3d(
                rawPosition.getX(),
                rawPosition.getY(),
                rawPosition.getZ()
        );


        double cameraPitchRad = Math.toRadians(LIMELIGHT_OFFSET_X_ANGLE);
        double cameraRollRad  = Math.toRadians(0.0);
        double cameraYawRad   = Math.toRadians(LIMELIGHT_OFFSET_Z_ANGLE);

        Rotation3d cameraMountRotation = new Rotation3d(cameraRollRad, cameraPitchRad, cameraYawRad);

        // Rotate the raw camera translation vector to align with the robot's horizontal frame of reference.
        Translation3d translationRobotToTag = translationCamToTag.rotateBy(cameraMountRotation);

        // Even if tilted, the actual horizontal distance on the floor is the hypotenuse of the corrected X and Z.
        // (Assuming standard robotics conventions where X is forward and Y is left after rotation alignment)
        double groundDistance = Math.hypot(translationRobotToTag.getX(), translationRobotToTag.getY());

        // Tags physical position on the field
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id);
        double tagHeadingRad = Math.toRadians(getTagFieldHeading(id));

        // Extract the rotation around the vertical Z-axis
        double skewZRad = targetPoseRaw.getOrientation().getZ();

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
    private double getTagFieldX(int id){
        return 0;
    }
    private double getTagFieldY(int id){
        return 0;
    }
    final double TAG_HEIGHT_LOW_STATE = 11.5;
    final double TAG_HEIGHT_HIGH_STATE = 23.5;

    //in meters how far above or below limeight can think the target is from a stable state
    final double EXCEPTABLE_OFFSET = 0.05;

    //returns null if the tag is moving aka not in a stable state
    private Double getTagFieldZ(LLResultTypes.FiducialResult limelightResults){
        //the target pose relative to the camera lens
        Pose3D targetPoseRaw = limelightResults.getTargetPoseCameraSpace();
        if (targetPoseRaw == null){
            return null;
        }
        Position rawPosition = targetPoseRaw.getPosition();

        // In Limelight Camera Space, index 1 is Y (Vertical distance).
        // Because Camera Y points DOWN, a tag above the camera has a negative Y value.
        double targetCamZ = rawPosition.y;
        //forward distance to the tag
        double targetCamY = rawPosition.z;

        double targetXOffSet = limelight.getTargetYOffset();

        // 3. Extrapolate what the tag's field height must be based on where the camera sees it
        double calculatedTagFieldZ1 = LIMELIGHT_HEIGHT - targetCamY / Math.cos(LIMELIGHT_OFFSET_X_ANGLE + targetXOffSet);
        double calculatedTagFieldZ2 = LIMELIGHT_HEIGHT - targetCamZ / Math.sin(LIMELIGHT_OFFSET_X_ANGLE + targetXOffSet);
        double calculatedTagFieldZ = (calculatedTagFieldZ1 + calculatedTagFieldZ2)/2;

        // 4. Check which of the two stable states the measurement is closer to
        double distanceToLowState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_LOW_STATE);
        double distanceToHighState = Math.abs(calculatedTagFieldZ - TAG_HEIGHT_HIGH_STATE);

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
