package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class LimelightBlue extends calculatePosition{

    boolean[] idVisible = new boolean[16];
    int[] idUp = new int[16];

    public LimelightBlue(Limelight ll, Telemetry t) {
        super(ll, t);
    }

    @Override
    public boolean canSeeTargetFriendly() {
        for (int i = 8; i < 16; i++) {
            if (idVisible[i]) {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean canSeeTargetEnemy() {
        for (int i = 0; i < 8; i++) {
            if (idVisible[i]) {
                return true;
            }
        }
        return false;
    }


    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up
    @Override
    public int friendlyUp() {
        if (!canSeeTargetFriendly()){
            return 0;
        }
        int countUp = 0;
        int countDown = 0;
        int countMoving = 0;
        for (int i = 8; i < 16; i++){
            if (idVisible[i]) {
                if (idUp[i] == 0) {
                    countMoving++;
                }
                else if (idUp[i] == 1) {
                    countDown++;
                }
                else {
                    countUp++;
                }
            }
        }
        if (countUp > countDown && countUp > countMoving){
            return 3;
        }
        if (countDown > countMoving){
            return 2;
        }
        return 1;
    }

    @Override
    public int enemyUp() {
        if (!canSeeTargetEnemy()){
            return 0;
        }
        int countUp = 0;
        int countDown = 0;
        int countMoving = 0;
        for (int i = 0; i < 8; i++){
            if (idVisible[i]) {
                if (idUp[i] == 0) {
                    countMoving++;
                }
                else if (idUp[i] == 1) {
                    countDown++;
                }
                else {
                    countUp++;
                }
            }
        }
        if (countUp > countDown && countUp > countMoving){
            return 3;
        }
        if (countDown > countMoving){
            return 2;
        }
        return 1;
    }

    @Override
    public void updateTagPosition() {
        List<LLResultTypes.FiducialResult> limelightResults = limelight.getFiducialResults();
        if (limelightResults == null) {
            limelightResults = new ArrayList<>();
        }
        int[] ids = getVisibleIds(limelightResults);
        for (int i = 0; i < 16; i ++){
            idVisible[i] = false;
        }
        for (int id : ids){
            if (isHiveTag(id)) {
                idVisible[id - 30] = true;
            }
        }
        for (LLResultTypes.FiducialResult tag : limelightResults){
            if (!isHiveTag(tag.getFiducialId())) {
                continue;
            }
            Double up = getTagFieldZ(tag);
            int val = 0;// 0 is moving
            if (up != null){
                val = 1;// 1 is down
                if (up == TAG_HEIGHT_HIGH_STATE){
                    val = 2;// 2 is up
                }
            }
            idUp[tag.getFiducialId()-30] = val;
        }
    }

    @Override
    public double[] aimState(double[] botPose) {
        if (friendlyUp() != 3) {
            return null;
        }
        double x, y, angle;
        double targetOffsetY = 0.3370036948997776 - 0.075;
        if (botPose[1] < 0){
            y = botPose[1] * -1;
        }
        else{
            y = botPose[1];
        }
        y -= targetOffsetY;
        if (botPose[1] > 0){
            x = HIVE_CENTER_X - botPose[0];
        }
        else {
            x = botPose[0] - HIVE_CENTER_X;
        }
        double fieldAngleToTarget = getShooterHeading(botPose, HIVE_CENTER_X, getTargetY(botPose));
        if (Double.isNaN(fieldAngleToTarget)) {
            return null;
        }
        //radians the robot has to turn for the shooter to face the target, positive is counterclockwise
        angle = Math.IEEEremainder(fieldAngleToTarget - botPose[2], 2 * Math.PI);

        return new double[] {x, y, angle};
    }

    //the target is the CELL on the same side of the center line as the robot
    private double getTargetY(double[] botPose) {
        double targetOffsetY = 0.3370036948997776 - 0.075;
        return botPose[1] > 0 ? targetOffsetY : -targetOffsetY;
    }

    /**
     * Pose the robot has to drive to so the shooter faces the target from the shooting distance.
     *
     * @param botPose {x, y, heading} of the robot center, meters / radians, field center origin
     * @return {x, y, heading} (meters, meters, radians), or null if the target can't be aimed at
     */
    public double[] getShootPose(double[] botPose) {
        return getShootPose(botPose, true);
    }

    /**
     * @param botPose {x, y, heading} of the robot center, meters / radians, field center origin
     * @param requireTargetUp true to return null unless a raised friendly CELL is in view right now. Pass false
     *                        when re-solving each loop from the Kalman pose after the target was already checked,
     *                        so the drive keeps going when the tags leave the Limelight's view
     * @return {x, y, heading} (meters, meters, radians), or null if the target can't be aimed at
     */
    public double[] getShootPose(double[] botPose, boolean requireTargetUp) {
        if (requireTargetUp && friendlyUp() != 3) {
            return null;
        }
        return getShootPose(botPose, HIVE_CENTER_X, getTargetY(botPose));
    }

    /**
     * @param botPose {x, y, heading} of the robot center, meters / radians, field center origin
     * @return horizontal distance from the shooter to the target center, meters
     */
    public double getShooterDistance(double[] botPose) {
        double targetY = getTargetY(botPose);
        double cos = Math.cos(botPose[2]);
        double sin = Math.sin(botPose[2]);
        double shooterX = botPose[0] + shooter_offset_x * cos - shooter_offset_y * sin;
        double shooterY = botPose[1] + shooter_offset_x * sin + shooter_offset_y * cos;
        return Math.hypot(HIVE_CENTER_X - shooterX, targetY - shooterY);
    }

    /** @return shooter to target distance the drive is aiming for, meters */
    public double getWantedShooterDistance(double[] botPose) {
        return getShootDistance(getTargetY(botPose));
    }
}
