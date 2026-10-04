package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
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

import java.util.List;


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
        llB = new LimelightBlue(ll, telemetry);
        ll.useAprilTagPipeline();
        ll.useColorPipeline();
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        double time = lastFrameTime.milliseconds();
        time *= 1000000;
        if (time < 0.5) {
            averageFrameTime = averageFrameTime * frameCount + time;
            averageFrameTime /= frameCount + 1;
            frameCount++;
            TeleOpData.frameTime = averageFrameTime;
        }
        lastFrameTime.reset();

        telemetry.addData("Frame time:", averageFrameTime);

        ll.update();
        double[] botPos = llB.getPosition();

        telemetry.addData("Limelight Connected", ll.limelight.isConnected());
        LLResult latestResult = ll.latestResult;
        if (latestResult == null || !latestResult.isValid()){
            telemetry.addData("nothing detected" , 0);
        }
        else {
            llB.updateTagPosition();
            boolean canSeeFriendly = llB.canSeeTargetFriendly();
            boolean canSeeEnemy = llB.canSeeTargetEnemy();
            telemetry.addData("Limelight can see friendly: ", canSeeFriendly);
            telemetry.addData("Limelight can see enemy: ", canSeeEnemy);
            if (botPos.length < 3) {
                telemetry.addData("no limelight results length: ", botPos.length);
                if (botPos.length == 1) {
                    telemetry.addData("fale type: ", botPos[0]);
                }

            } else {
                telemetry.addData("Limelight x: ", botPos[0]);
                telemetry.addData("Limelight y: ", botPos[1]);
                telemetry.addData("Limelight orientation: ", botPos[2]);
            }
            if (canSeeFriendly) {
                int isUp = llB.friendlyUp();
                telemetry.addData("Limelight friendly state: ", isUp);
            }
            if (canSeeEnemy) {
                int isUp = llB.enemyUp();
                telemetry.addData("Limelight enemy state: ", isUp);
            }
        }

    }
}