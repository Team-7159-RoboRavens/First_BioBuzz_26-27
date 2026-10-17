package org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

public class LimelightRed extends calculatePosition{

    boolean[] idVisible = new boolean[16];
    int[] idUp = new int[16];

    public LimelightRed(Limelight ll, Telemetry t) {
        super(ll, t);
    }

    @Override
    public boolean canSeeTargetFriendly() {
        for (int i = 0; i < 8; i++) {
            if (idVisible[i]) {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean canSeeTargetEnemy() {
        for (int i = 8; i < 16; i++) {
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
    public int enemyUp() {
        if (!canSeeTargetEnemy()){
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
        if (botPose[1] < 0){
            y = botPose[1] * -1;
        }
        else{
            y = botPose[1];
        }
        y -= 0.3370036948997776 - 0.075;
        if (botPose[1] > 0){
            x = -12.75 - botPose[0];
        }
        else {
            x = botPose[0] + 12.75;
        }

        return new double[] {x, y, botPose[2]};
    }
}
