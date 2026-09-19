package org.firstinspires.ftc.teamcode.Limelight;

public class LimelightBlue extends calculatePosition{
    @Override
    public boolean viewingFrontSideFriendly() {
        return false;
    }

    @Override
    public boolean viewingFrontSideEnemy() {
        return false;
    }

    @Override
    public boolean canSeeTargetFriendly() {
        return false;
    }

    @Override
    public boolean canSeeTargetEnemy() {
        return false;
    }

    //determines if the limelight can detect one of the blue targets
}
