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
        llB = new LimelightBlue(ll);
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


        LLStatus status = ll.limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());


        if (ll.hasValidResult() && status.getPipelineIndex() == 1) {
            int count = ll.getColorBlobCount();
            double y = ll.getColorBlobYOffset(0);
            double x = ll.getColorBlobXOffset(0);
            double xSize = ll.getColorBlobXSize(0);
            double ySize = ll.getColorBlobYSize(0);
            telemetry.addData("Limelight Targets", "Count: %d", count);
            telemetry.addData("Limelight Offset", "X: %.1f , Y: %.1f", x, y);
            telemetry.addData("Limelight Size", "W: %.1f , H: %.1f", xSize, ySize);
        }
        else {
            telemetry.addData("Limelight", "no color result");
        }

        if (ll.hasValidResult() && status.getPipelineIndex() == 0 ) {

            int id = ll.getAprilTagId();
            double y = ll.getAprilTagAngleYOffset();
            double d2D = ll.getAprilTagDistance2D();
            double x = ll.getAprilTagAngleXOffset();
            double d3D = ll.getAprilTagDistance3D();
            telemetry.addData("Limelight Targets", "id: %d", id);
            telemetry.addData("Limelight Offset", "X: %.1f , Y: %.1f", x, y);
            telemetry.addData("Limelight ID", "ID: %.1f", id);
            telemetry.addData("Limelight Distance", "2D: %.1f , 3D: %.1f", d2D, d3D);
        }
        else {
            telemetry.addData("Limelight", "no limelight result");
        }

        telemetry.addData("Limelight Connected", ll.limelight.isConnected());
        LLResult latestResult = ll.latestResult;
        if (latestResult == null || !latestResult.isValid()){
            telemetry.addData("nothing detected" , 0);
        }
        else {
            boolean canSeeFriendly = llB.canSeeTargetFriendly();
            boolean canSeeEnemy = llB.canSeeTargetEnemy();
            telemetry.addData("Limelight can see friendly: ", canSeeFriendly);
            telemetry.addData("Limelight can see enemy: ", canSeeEnemy);
            if (botPos.length < 3) {
                telemetry.addData("no limelight results", botPos.length);
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