package org.firstinspires.ftc.teamcode.Limelight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightBlue extends calculatePosition{
    // Field y of the target CELL center from the center line, meters
    private static final double TARGET_OFFSET_Y = 0.3370036948997776 - 0.075;

    public LimelightBlue(Limelight ll, Telemetry t) {
        super(ll, t);
    }

    // Our HIVE's tags are 38-45, the other HIVE's are 30-37
    @Override
    public boolean canSeeTargetFriendly() {
        return canSeeTags(38, 45);
    }

    @Override
    public boolean canSeeTargetEnemy() {
        return canSeeTags(30, 37);
    }

    //returns 0 for no targets detected 1 for moving 2 for down and 3 for up, both CELLs together
    @Override
    public int friendlyUp() {
        return tagState(38, 45, 0);
    }

    @Override
    public int enemyUp() {
        return tagState(30, 37, 0);
    }

    @Override
    public int friendlyCellState(boolean plusY) {
        return tagState(38, 45, plusY ? 1 : -1);
    }

    @Override
    public double[] aimState(double[] botPose) {
        if (friendlyCellState(botPose[1] > 0) != STATE_UP) {
            return null;
        }
        double x, y, angle;
        if (botPose[1] < 0){
            y = botPose[1] * -1;
        }
        else{
            y = botPose[1];
        }
        y -= TARGET_OFFSET_Y;
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
        return botPose[1] > 0 ? TARGET_OFFSET_Y : -TARGET_OFFSET_Y;
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
        if (requireTargetUp && friendlyCellState(botPose[1] > 0) != STATE_UP) {
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
