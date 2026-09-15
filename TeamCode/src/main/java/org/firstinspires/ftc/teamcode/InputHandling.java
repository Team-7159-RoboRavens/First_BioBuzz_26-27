package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBase;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseNewBot;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class InputHandling {
 
    private final double MAX_RPM = 300;
 
    // Voltage the max rpm was measured at; used to scale maxRPM to current battery voltage
    private final double INITIAL_VOLTAGE = 12.5;
 
    // Volts per RPM/s; only ratios matter
    private final double[] KA = {0.002398, 0.002058, 0.001870, 0.002433};
 
    // Volts per RPM, from steady-state RPM at fixed power
    private final double[] KV = {0.034303, 0.033910, 0.032635, 0.035025};
 
    // Static friction volts, placeholder; needs a break-away test
    private final double[] KS = {0.35, 0.35, 0.35, 0.35};
 
    // Heading correction, fraction of maxRPM per radian of error
    private final double TURN_ADJUST_SCALE = 1.0;
 
    // Heading correction, fraction of maxRPM per rad/s of drift
    private final double TURN_ADJUST_DERIVATIVE = 0.10;
 
    // Heading errors below this are ignored, radians
    private final double HEADING_DEADBAND = 0.02;
 
    // Heading correction clamp, fraction of maxRPM
    private final double MAX_TURN_CORRECTION = 0.3;
 
    // Frames averaged before heading lock engages
    private final int LOCK_SAMPLES = 5;
 
    // Encoder CPR at the output shaft
    private final double TICKS = 383.6;
 
    private final int SECONDS_PER_MINUTE = 60;
 
    // Fraction of remaining error closed per frame; lower this first if it buzzes
    private final double FRACTION_UNDER_SHOOT = 0.7;
 
    // Threshold for "driver is turning"
    private final double POWER_EPSILON = 1e-3;
 
    // Friction sign blend width, RPM
    private final double FRICTION_RAMP_RPM = 20.0;
 
    // Friction sign fade width vs error
    private final double FRICTION_RAMP_DIFF = 10.0;
 
    // Voltage filter coefficient, ~1s time constant at 50Hz
    private final double VOLTAGE_FILTER = 0.02;
 
    private final double MIN_VALID_VOLTAGE = 6.0;
 
    private double maxRPM = MAX_RPM;
    private double frameTime = 0.02;
 
    private Robot robot;
    private VoltageSensor voltageSensor;
 
    private DcMotorEx lf, rf, lb, rb;
    private DcMotorEx[] motors;
 
    private int lockHeading;
    private double lockAngle;
    private double lastHeadingError;
    private double[] sinSamples = new double[LOCK_SAMPLES];
    private double[] cosSamples = new double[LOCK_SAMPLES];
    private boolean holdHeading;
 
    // Instantaneous reading, used for volts -> power conversion
    private double batteryVoltage = 12.0;
 
    // Slow-filtered reading, used to scale maxRPM so sag doesn't shrink the target
    private double filteredVoltage = 12.0;

    public InputHandling(Robot a) {
        robot = a;
        voltageSensor = robot.voltageSensor;
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
        motors = new DcMotorEx[]{lf, rf, lb, rb};

        double v = voltageSensor.getVoltage();
        if (v > MIN_VALID_VOLTAGE) {
            batteryVoltage = v;
            filteredVoltage = v;
        }
        maxRPM = MAX_RPM * filteredVoltage / INITIAL_VOLTAGE;
    }
 
    // Call once per loop, before getMotorPowers
    public void updateVoltage() {
        double v = voltageSensor.getVoltage();
        if (v > MIN_VALID_VOLTAGE) {
            batteryVoltage = v;
            filteredVoltage += VOLTAGE_FILTER * (v - filteredVoltage);
        }
        maxRPM = MAX_RPM * filteredVoltage / INITIAL_VOLTAGE;
    }
 
    public void setLockHeading(boolean enable) {
        holdHeading = enable;
        if (!enable) {
            lockHeading = 0;
        }
    }
 
    public MotorPowers getMotorPowers(MotorPowers power) {
        updateVoltage();
        frameTime = TeleOpData.frameTime;

        if (frameTime < 1e-3 || frameTime > 0.2) {
            frameTime = 0.02;
        }
 
        double[] targetRPM = {
            power.lf * maxRPM,
            power.rf * maxRPM,
            power.lb * maxRPM,
            power.rb * maxRPM
        };
 
        applyHeadingHold(power, targetRPM);
 
        // Set BulkCachingMode.MANUAL in Robot or this is 4 separate USB round trips
        double[] RPM = new double[4];
        for (int i = 0; i < 4; i++) {
            RPM[i] = motors[i].getVelocity() * SECONDS_PER_MINUTE / TICKS;
        }
 
        return solve(RPM, targetRPM);
    }
 
    private void applyHeadingHold(MotorPowers power, double[] targetRPM) {
 
        // Mecanum rotation component of the driver's request
        double turnComponent = (power.lf + power.lb) - (power.rf + power.rb);
 
        if (Math.abs(turnComponent) < POWER_EPSILON) {
            if (lockHeading < LOCK_SAMPLES) {
                double a = robot.pinpointAngle();
                sinSamples[lockHeading] = Math.sin(a);
                cosSamples[lockHeading] = Math.cos(a);
                lockHeading++;
                if (lockHeading == LOCK_SAMPLES) {
                    double s = 0, c = 0;
                    for (int i = 0; i < LOCK_SAMPLES; i++) {
                        s += sinSamples[i];
                        c += cosSamples[i];
                    }
                    lockAngle = Math.atan2(s, c);
                    lastHeadingError = 0;
                }
            }
        } else {
            lockHeading = 0;
        }
 
        if (!holdHeading || lockHeading < LOCK_SAMPLES) {
            return;
        }
 
        double error = wrapAngle(robot.pinpointAngle() - lockAngle);
 
        double correction = 0;
        if (Math.abs(error) > HEADING_DEADBAND) {
            double derivative = wrapAngle(error - lastHeadingError) / frameTime;
            correction = -(TURN_ADJUST_SCALE * error + TURN_ADJUST_DERIVATIVE * derivative);
            correction = clamp(correction, -MAX_TURN_CORRECTION, MAX_TURN_CORRECTION);
        }
        lastHeadingError = error;
 
        targetRPM[0] += correction * maxRPM;
        targetRPM[2] += correction * maxRPM;
        targetRPM[1] -= correction * maxRPM;
        targetRPM[3] -= correction * maxRPM;
 
        double max = Math.abs(targetRPM[maxIndex(targetRPM)]);
        if (max > maxRPM) {
            double scale = maxRPM / max;
            for (int i = 0; i < 4; i++) {
                targetRPM[i] *= scale;
            }
        }
    }
 
    // Largest shared s such that every motor stays inside the battery voltage
    private MotorPowers solve(double[] RPM, double[] targetRPM) {
        

        //volts required to accelerate for given motor
        double[] m = new double[4];
        // voltage required to maintain current RPM
        double[] b = new double[4];
 
        for (int i = 0; i < 4; i++) {
            double diff = targetRPM[i] - RPM[i];
            m[i] = KA[i] * diff;
            b[i] = KV[i] * RPM[i] + KS[i] * frictionSign(RPM[i], diff);
        }
 
        double s = FRACTION_UNDER_SHOOT / frameTime;
 
        for (int i = 0; i < 4; i++) {
            if (Math.abs(m[i]) < 1e-9) {
                continue;
            }
            double rail = (m[i] > 0) ? batteryVoltage : -batteryVoltage;
            double limit = (rail - b[i]) / m[i];
            if (limit < s) {
                s = limit;
            }
        }
 
        if (s < 0) {
            s = 0;
        }
 
        MotorPowers p = new MotorPowers();
        double[] v = new double[4];
        for (int i = 0; i < 4; i++) {
            v[i] = clamp(m[i] * s + b[i], -batteryVoltage, batteryVoltage);
        }
 
        p.lf = clamp(v[0] / batteryVoltage, -1, 1);
        p.rf = clamp(v[1] / batteryVoltage, -1, 1);
        p.lb = clamp(v[2] / batteryVoltage, -1, 1);
        p.rb = clamp(v[3] / batteryVoltage, -1, 1);
        return p;
    }
 
    // Friction direction in [-1,1]; sign(RPM) when moving, fades toward diff near rest
    private double frictionSign(double RPM, double diff) {
        double moving = clamp(Math.abs(RPM) / FRICTION_RAMP_RPM, 0, 1);
        double breakaway = clamp(diff / FRICTION_RAMP_DIFF, -1, 1);
        return moving * Math.signum(RPM) + (1 - moving) * breakaway;
    }
 
    private static double wrapAngle(double a) {
        while (a > Math.PI) {
            a -= 2 * Math.PI;
        }
        while (a < -Math.PI) {
            a += 2 * Math.PI;
        }
        return a;
    }
 
    private static double clamp(double v, double lo, double hi) {
        if (v < lo) {
            return lo;
        }
        if (v > hi) {
            return hi;
        }
        return v;
    }
 
    private int maxIndex(double[] a) {
        double max = Math.abs(a[0]);
        int index = 0;
        for (int i = 1; i < a.length; i++) {
            double c = Math.abs(a[i]);
            if (c > max) {
                index = i;
                max = c;
            }
        }
        return index;
    }
}