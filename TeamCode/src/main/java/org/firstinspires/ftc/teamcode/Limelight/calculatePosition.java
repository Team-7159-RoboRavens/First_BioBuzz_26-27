package org.firstinspires.ftc.teamcode.Limelight;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;

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
        int[] ids = getVisibleIds();
        if (ids.length == 0){
            return new double[] {};
        }
        //list of coordinates x, y, anlge z axis
        List<double[]> coordinates = new ArrayList<>();
        for (int id: ids){
            coordinates.add(getExpectedCoordinate(id));
        }
        double sumX = 0;
        double sumY = 0;
        double sumAngle = 0;
        double count = 0;
        for (double[] coordinate : coordinates){
            sumX += coordinate[0];
            sumY += coordinate[1];
            sumAngle += coordinate[2];
            count ++;
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
    private int[] getVisibleIds(){
        return new int[] {0};
    }

    //returns the expected position of the limelight on the field based on a specific april tag;
    private double[] getExpectedCoordinate(int id){
        //LimelightHelpers.LimelightTarget_Fiducial[] targets = limelight.latestResult.targetingResults.targets_Fiducials;

        //angle from target to limelight
        double txRad = 0.0;
        double tyRad = 0.0;

        //distance on the ground from target to tag
        double groundDistance = getGroundDistanceToTag(tyRad, id);

        //tags physical position on the field
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id);
        // angle the tag phases on the field either forward or backward;
        double tagHeadingRad = Math.toRadians(getTagFieldHeading(id));

        //angle of the limelight relative to the tag on the z-axis
        double skewZRad = 0;

        //absolute angle of the path to the tag relative to the field
        double pathHeadingRad = skewZRad + tagHeadingRad;

        //absolute heading of the limelight on the z axis
        double cameraHeadingRad = pathHeadingRad + txRad;

        // 5. Project outwards from the tag face to find where the camera is standing
        double limelightX = tagX + (groundDistance * Math.cos(pathHeadingRad));
        double limelightY = tagY + (groundDistance * Math.sin(pathHeadingRad));

        return new double[] {limelightX, limelightY, cameraHeadingRad};
    }
    private double getGroundDistanceToTag(double ty, int id) {
        double tagHeight = getTagFieldZ(id); // h2: Look up the tag's official height

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
    private double getTagFieldZ(int id){
        return 0;
    }
    //angle of the tage on the z axis
    private double getTagFieldHeading(int id){
        return 0;
    }

}
