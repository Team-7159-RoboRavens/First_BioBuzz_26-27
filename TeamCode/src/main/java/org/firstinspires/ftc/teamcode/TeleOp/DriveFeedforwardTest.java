package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Locale;

// Measures KS for InputHandling: the volts each drive wheel needs to start moving, with the robot on the field tiles.
// Ramps all wheels forward slowly, then backward, and records the volts when each wheel starts turning.
// Moves the robot only an inch or two. Put the 4 numbers into KS, then try VELOCITY_CONTROL = true
@TeleOp(name = "DriveFeedforwardTest", group = "Tuning")
public class DriveFeedforwardTest extends LinearOpMode {
    // Power added per second while ramping
    public static double RAMP_RATE = 0.05;
    // A wheel counts as moving above this, RPM
    public static double MOVING_RPM = 5;
    // Gives up on a direction past this power
    public static double MAX_TEST_POWER = 0.4;
    private static final double TICKS = 1075.2;
    private static final String[] NAMES = {"lf", "rf", "lb", "rb"};

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, 10);
        // The ramp needs every small power step sent
        robot.powerResendThreshold = 0;
        DcMotorEx[] motors = robot.getDriveMotors();
        telemetry.addLine("Robot on the tiles with a few inches of room in front and behind, then press start");
        telemetry.update();
        waitForStart();

        double[] forward = ramp(robot, motors, 1);
        sleep(500);
        double[] backward = ramp(robot, motors, -1);
        robot.setPowers(new MotorPowers());
        // Also in the robot log (tag DriveKS), so the numbers aren't only on the driver station
        for (int i = 0; i < 4; i++) {
            Log.i("DriveKS", String.format(Locale.US, "%s forward %.3f backward %.3f average %.3f V",
                    NAMES[i], forward[i], backward[i], (forward[i] + backward[i]) / 2));
        }

        while (opModeIsActive()) {
            telemetry.addLine("KS in volts, forward / backward / average");
            for (int i = 0; i < 4; i++) {
                telemetry.addData(NAMES[i], "%.3f / %.3f / %.3f",
                        forward[i], backward[i], (forward[i] + backward[i]) / 2);
            }
            telemetry.addLine("NaN = didn't move before MAX_TEST_POWER");
            telemetry.update();
            idle();
        }
    }

    // Volts at the moment each wheel started moving, NaN if it never did
    private double[] ramp(Robot robot, DcMotorEx[] motors, int direction) {
        double[] result = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        int found = 0;
        ElapsedTime timer = new ElapsedTime();
        MotorPowers rampPowers = new MotorPowers();
        while (opModeIsActive() && found < 4) {
            for (LynxModule hub : robot.hubs) {
                hub.clearBulkCache();
            }
            double power = RAMP_RATE * timer.seconds();
            if (power > MAX_TEST_POWER) {
                break;
            }
            // Voltage is its own hub read, so only read it in a loop where a wheel starts moving
            double volts = Double.NaN;
            for (int i = 0; i < 4; i++) {
                double rpm = motors[i].getVelocity() * 60 / TICKS;
                if (Double.isNaN(result[i]) && Math.abs(rpm) > MOVING_RPM) {
                    if (Double.isNaN(volts)) {
                        volts = robot.getBatteryVoltage();
                    }
                    result[i] = power * volts;
                    found++;
                }
            }
            double p = direction * power;
            rampPowers.lf = p;
            rampPowers.rf = p;
            rampPowers.lb = p;
            rampPowers.rb = p;
            robot.setPowers(rampPowers);
            telemetry.addData("Ramping", "%s, power %.3f, %d of 4 moving", direction > 0 ? "forward" : "backward", power, found);
            telemetry.update();
        }
        robot.setPowers(new MotorPowers());
        return result;
    }
}
