package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestBM;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.util.ElapsedTime;


// TeleOp op mode using the Pinpoint-aware button map, with odometry telemetry.

@TeleOp(name = "limelightTest")
public class limelightTest extends OpMode {

    Robot robot;
    WheelTestBM bm;
    private ElapsedTime lastFrameTime = new ElapsedTime();

    public double averageFrameTime = 0;
    private long frameCount;
    Limelight ll;

    LimelightBlue llB;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 7);
        ll = new Limelight(robot);
        bm = new WheelTestBM(this, robot);
        llB = new LimelightBlue(ll);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }


        averageFrameTime = averageFrameTime * frameCount + lastFrameTime.milliseconds();
        averageFrameTime /= frameCount + 1;
        lastFrameTime.reset();
        frameCount ++;
        TeleOpData.frameTime = averageFrameTime;

        telemetry.addData("Frame time:", averageFrameTime);

        ll.update();
        double[] botPos = llB.getPosition();

        boolean canSeeFriendly = llB.canSeeTargetFriendly();
        boolean canSeeEnemy = llB.canSeeTargetEnemy();
        telemetry.addData("Limelight can see friendly: " , canSeeFriendly);
        telemetry.addData("Limelight can see enemy: " , canSeeEnemy);
        if (botPos.length < 3){
            telemetry.addData("no limelight results" , botPos.length);
        }
        else {
            telemetry.addData("Limelight x: " , botPos[0]);
            telemetry.addData("Limelight y: " , botPos[1]);
            telemetry.addData("Limelight orientation: " , botPos[2]);
        }
        if (canSeeFriendly){
            int isUp = llB.friendlyUp();
            telemetry.addData("Limelight friendly state: " , isUp);
        }
        if (canSeeEnemy){
            int isUp = llB.enemyUp();
            telemetry.addData("Limelight enemy state: " , isUp);
        }

    }
}