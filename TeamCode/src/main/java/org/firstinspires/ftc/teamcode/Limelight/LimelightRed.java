package org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightRed extends calculatePosition{

    public LimelightRed(Limelight ll, Telemetry t) {
        super(ll, t);
    }

    // Our HIVE's tags are 30-37, the other HIVE's are 38-45
    @Override
    public boolean canSeeTargetFriendly() {
        return canSeeTags(30, 37);
    }

    @Override
    public boolean canSeeTargetEnemy() {
        return canSeeTags(38, 45);
    }

    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up, both CELLs together
    @Override
    public int friendlyUp() {
        return tagState(30, 37, 0);
    }

    @Override
    public int enemyUp() {
        return tagState(38, 45, 0);
    }

    @Override
    public int friendlyCellState(boolean plusY) {
        return tagState(30, 37, plusY ? 1 : -1);
    }

    @Override
    public double[] aimState(double[] botPose) {
        if (friendlyCellState(botPose[1] > 0) != STATE_UP) {
            return null;
        }
        double x, y;
        if (botPose[1] < 0){
            y = botPose[1] * -1;
        }
        else{
            y = botPose[1];
        }
        y -= 0.3370036948997776 - 0.075;
        if (botPose[1] > 0){
            x = -HIVE_CENTER_X - botPose[0];
        }
        else {
            x = botPose[0] + HIVE_CENTER_X;
        }
        // Same as Blue: the turn for the shooter to face the CELL on the robot's side (positive is counterclockwise)
        double targetY = botPose[1] > 0 ? 0.3370036948997776 - 0.075 : -(0.3370036948997776 - 0.075);
        double fieldAngleToTarget = getShooterHeading(botPose, -HIVE_CENTER_X, targetY);
        if (Double.isNaN(fieldAngleToTarget)) {
            return null;
        }
        double angle = Math.IEEEremainder(fieldAngleToTarget - botPose[2], 2 * Math.PI);

        return new double[] {x, y, angle};
    }
}
