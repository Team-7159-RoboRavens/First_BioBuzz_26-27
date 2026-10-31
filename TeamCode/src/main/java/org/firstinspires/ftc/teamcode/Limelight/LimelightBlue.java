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
        //the target is the CELL on the same side of the center line as the robot
        double targetY = botPose[1] > 0 ? targetOffsetY : -targetOffsetY;
        double fieldAngleToTarget = Math.atan2(targetY - botPose[1], HIVE_CENTER_X - botPose[0]);
        //radians the robot has to turn to face the target, positive is counterclockwise
        angle = Math.IEEEremainder(fieldAngleToTarget - botPose[2], 2 * Math.PI);

        return new double[] {x, y, angle};
    }
}
