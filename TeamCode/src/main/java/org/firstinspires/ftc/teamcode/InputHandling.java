package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Heading hold: keeps the robot facing the same way while nothing is asking it to turn.
// Velocity control (off by default): turns stick power into wheel RPM so the drive feels the same at any battery voltage
public class InputHandling {

    // ---------------- Tuning: heading hold ----------------
    public static boolean HEADING_HOLD = true;
    // Turn power per radian of heading error
    public static double HOLD_KP = 1.0;
    // Turn power per rad/s of turn rate, damps the pull back
    public static double HOLD_KD = 0.10;
    // Heading errors below this are ignored, radians
    public static double HOLD_DEADBAND = 0.02;
    // Most turn power the hold adds
    public static double HOLD_MAX_POWER = 0.3;
    // Lock only once the robot has stopped spinning from the last turn, rad/s
    public static double LOCK_TURN_RATE = 0.5;
    // Locks anyway after this long, in case it never settles below LOCK_TURN_RATE, seconds
    public static double LOCK_TIMEOUT = 0.4;

    // ---------------- Tuning: velocity control ----------------
    // Off until KS is measured with DriveFeedforwardTest, it has never been run on the robot
    public static boolean VELOCITY_CONTROL = false;
    // Battery is read every this many loops, each read is a separate hub command
    public static int VOLTAGE_READ_EVERY = 5;

    private static final double MAX_RPM = 300;
    // Voltage MAX_RPM was measured at
    private static final double INITIAL_VOLTAGE = 12.5;
    // Volts per RPM/s
    private static final double[] KA = {0.002398, 0.002058, 0.001870, 0.002433};
    // Volts per RPM, from steady-state RPM at fixed power
    private static final double[] KV = {0.034303, 0.033910, 0.032635, 0.035025};
    // Static friction volts, placeholder until DriveFeedforwardTest is run
    private static final double[] KS = {0.35, 0.35, 0.35, 0.35};
    // Encoder CPR at the output shaft
    private static final double TICKS = 1075.2;
    // Fraction of the remaining RPM error closed per loop, lower this first if it buzzes
    private static final double FRACTION_UNDER_SHOOT = 0.7;
    private static final double FRICTION_RAMP_RPM = 20.0;
    private static final double FRICTION_RAMP_DIFF = 10.0;
    // Voltage filter coefficient per read
    private static final double VOLTAGE_FILTER = 0.1;
    private static final double MIN_VALID_VOLTAGE = 6.0;

    // Below this the driver (or aim code) counts as not turning
    private static final double TURN_EPSILON = 1e-3;

    private final Robot robot;
    private final VoltageSensor voltageSensor;
    private final DcMotorEx[] motors;

    private boolean locked = false;
    private double lockAngle = 0;
    private long settleStartNanos = 0;
    private boolean settling = false;

    private long lastNanos = 0;
    private double frameTime = 0.02;
    private int voltageCounter = 0;
    private double maxRPM = MAX_RPM;
    private double batteryVoltage = 12.0;
    private double filteredVoltage = 12.0;
    // getMotorPowers() refills this every loop instead of making a new one
    private final MotorPowers heldPowers = new MotorPowers();

    public InputHandling(Robot a) {
        robot = a;
        voltageSensor = robot.voltageSensor;
        motors = new DcMotorEx[]{robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack};
        double v = voltageSensor.getVoltage();
        if (v > MIN_VALID_VOLTAGE) {
            batteryVoltage = v;
            filteredVoltage = v;
        }
        maxRPM = MAX_RPM * filteredVoltage / INITIAL_VOLTAGE;
    }

    public void setLockHeading(boolean enable) {
        HEADING_HOLD = enable;
        if (!enable) {
            reset();
        }
    }

    // Drops the lock, the next straight drive locks onto the heading at that time
    public void reset() {
        locked = false;
        settling = false;
    }

    public boolean isLocked() {
        return locked;
    }

    public double getLockAngle() {
        return lockAngle;
    }

    // Call once per loop after pinpointUpdate, with the button map's powers
    public MotorPowers getMotorPowers(MotorPowers power) {
        long now = System.nanoTime();
        if (lastNanos != 0) {
            frameTime = (now - lastNanos) / 1e9;
        }
        lastNanos = now;
        if (frameTime < 1e-3 || frameTime > 0.2) {
            frameTime = 0.02;
        }

        MotorPowers out = heldPowers;
        out.lf = power.lf;
        out.rf = power.rf;
        out.lb = power.lb;
        out.rb = power.rb;
        if (HEADING_HOLD) {
            applyHeadingHold(out, now);
        }
        if (VELOCITY_CONTROL) {
            out = velocityControl(out);
        }
        return out;
    }

    private void applyHeadingHold(MotorPowers p, long now) {
        // Clockwise part of the request, (lf + lb) - (rf + rb) = 4 * rotate in Mechanumdrive
        double turn = (p.lf + p.lb) - (p.rf + p.rb);
        double heading = robot.pinpointAngle();
        // Counterclockwise, rad/s
        double turnRate = robot.getPinpointHeadingVelocity();
        if (!Double.isFinite(heading) || !Double.isFinite(turnRate)) {
            return;
        }

        if (Math.abs(turn) > TURN_EPSILON) {
            reset();
            return;
        }
        if (!locked) {
            if (!settling) {
                settling = true;
                settleStartNanos = now;
            }
            boolean timedOut = (now - settleStartNanos) / 1e9 > LOCK_TIMEOUT;
            if (Math.abs(turnRate) < LOCK_TURN_RATE || timedOut) {
                locked = true;
                lockAngle = heading;
            }
            return;
        }

        // Positive = turned counterclockwise past the lock, needs clockwise power
        double error = Math.IEEEremainder(heading - lockAngle, 2 * Math.PI);
        double correction = 0;
        if (Math.abs(error) > HOLD_DEADBAND) {
            correction = HOLD_KP * error + HOLD_KD * turnRate;
            correction = clamp(correction, -HOLD_MAX_POWER, HOLD_MAX_POWER);
        }

        p.lf += correction;
        p.lb += correction;
        p.rf -= correction;
        p.rb -= correction;

        double max = Math.max(Math.max(Math.abs(p.lf), Math.abs(p.rf)), Math.max(Math.abs(p.lb), Math.abs(p.rb)));
        if (max > 1) {
            p.lf /= max;
            p.rf /= max;
            p.lb /= max;
            p.rb /= max;
        }
    }

    private void updateVoltage() {
        voltageCounter++;
        if (voltageCounter < VOLTAGE_READ_EVERY) {
            return;
        }
        voltageCounter = 0;
        double v = voltageSensor.getVoltage();
        if (v > MIN_VALID_VOLTAGE) {
            batteryVoltage = v;
            filteredVoltage += VOLTAGE_FILTER * (v - filteredVoltage);
        }
        maxRPM = MAX_RPM * filteredVoltage / INITIAL_VOLTAGE;
    }

    private MotorPowers velocityControl(MotorPowers p) {
        updateVoltage();
        double[] targetRPM = {p.lf * maxRPM, p.rf * maxRPM, p.lb * maxRPM, p.rb * maxRPM};
        // Bulk cached, no extra hub reads
        double[] RPM = new double[4];
        for (int i = 0; i < 4; i++) {
            RPM[i] = motors[i].getVelocity() * 60 / TICKS;
        }
        return solve(RPM, targetRPM);
    }

    // Largest shared s such that every motor stays inside the battery voltage
    private MotorPowers solve(double[] RPM, double[] targetRPM) {
        // Volts per unit of s to accelerate, and volts to hold the current RPM
        double[] m = new double[4];
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

        MotorPowers out = new MotorPowers();
        out.lf = clamp((m[0] * s + b[0]) / batteryVoltage, -1, 1);
        out.rf = clamp((m[1] * s + b[1]) / batteryVoltage, -1, 1);
        out.lb = clamp((m[2] * s + b[2]) / batteryVoltage, -1, 1);
        out.rb = clamp((m[3] * s + b[3]) / batteryVoltage, -1, 1);
        return out;
    }

    // Friction direction in [-1,1]; sign(RPM) when moving, fades toward diff near rest
    private double frictionSign(double RPM, double diff) {
        double moving = clamp(Math.abs(RPM) / FRICTION_RAMP_RPM, 0, 1);
        double breakaway = clamp(diff / FRICTION_RAMP_DIFF, -1, 1);
        return moving * Math.signum(RPM) + (1 - moving) * breakaway;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
