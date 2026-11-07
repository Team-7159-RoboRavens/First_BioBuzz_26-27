package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Multi-Color Tuner", group = "Tuning")
public class HuskyLensTeleOp extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            // Keep track of how many of your 3 colors are visible
            int color1Count = 0;
            int color2Count = 0;
            int color3Count = 0;

            for (HuskyLens.Block block : blocks) {
                int ballArea = block.width * block.height;

                // Separate the telemetry logic by the specific ID numbers you trained
                if (block.id == 1) {
                    color1Count++;
                    telemetry.addLine("[ID 1 - Blue] Block #" + color1Count);
                    telemetry.addData("   Size", "W: %d | H: %d", block.width, block.height);
                    telemetry.addData("   AREA", ballArea);
                }
                else if (block.id == 2) {
                    color2Count++;
                    telemetry.addLine("[ID 2 - Red] Block #" + color2Count);
                    telemetry.addData("   Size", "W: %d | H: %d", block.width, block.height);
                    telemetry.addData("   AREA", ballArea);
                }
                else if (block.id == 3) {
                    color3Count++;
                    telemetry.addLine("[ID 3 - Yellow] Block #" + color3Count);
                    telemetry.addData("   Size", "W: %d | H: %d", block.width, block.height);
                    telemetry.addData("   AREA", ballArea);
                }
            }

            // Summary data at the bottom of the screen
            telemetry.addLine("\n=== DETECTION SUMMARY ===");
            telemetry.addData("Total Blocks on Screen", blocks.length);
            telemetry.addData("ID 1 Seen", color1Count);
            telemetry.addData("ID 2 Seen", color2Count);
            telemetry.addData("ID 3 Seen", color3Count);

            telemetry.update();
            sleep(100);
        }
    }
}
