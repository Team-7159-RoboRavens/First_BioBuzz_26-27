package org.firstinspires.ftc.teamcode.Limelight;

public class LimelightRed extends calculatePosition{

    @Override
    public boolean viewingFrontSideFriendly() {
        return false;
    }

    @Override
    public boolean viewingFrontSideEnemy() {
        return false;
    }

    @Override
    public boolean canSeeTargetFriendly(){
        return false;
    }

    @Override
    public boolean canSeeTargetEnemy() {
        return false;
    }
}
