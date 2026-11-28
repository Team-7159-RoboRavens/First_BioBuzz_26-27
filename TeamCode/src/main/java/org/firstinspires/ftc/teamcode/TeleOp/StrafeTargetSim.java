package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;

// Pretend robot strafing back and forth along the blue strafe line (g1 LT in teleop) while aimed at the HIVE.
// Gives the flywheel RPM teleop would ask for at each moment, so tests can run it with the robot standing still
public class StrafeTargetSim {
    // Pretend strafe speed along the line, in/s, and how fast it turns around, in/s^2
    public static double SPEED = 22;
    public static double ACCEL = 60;
    // How much of each new aim turn rate is used, like teleop's filtered turn rate. The raw per-loop turn rate
    // fed back into the shot math and pinned the target at MAX_RPM
    public static double TURN_BLEND = 0.2;

    public final double minX, maxX, lineY;
    private double x, vx, heading, turnRate;
    private int direction = 1;
    private double target;
    private double minTarget = Double.POSITIVE_INFINITY, maxTarget = Double.NEGATIVE_INFINITY;

    public StrafeTargetSim(double fallbackRPM) {
        double hiveX = 72 + ShotCalculator.HIVE_X_FROM_CENTER;
        lineY = 72 + WorkingBotButtonMap.STRAFE_LINE_DISTANCE;
        minX = hiveX - WorkingBotButtonMap.STRAFE_RIGHT;
        maxX = hiveX + WorkingBotButtonMap.STRAFE_LEFT;
        x = minX;
        ShotCalculator.Shot first = ShotCalculator.solve(x, lineY, Math.PI / 2, 0, 0, 0, 0, 0, false);
        heading = first.valid ? first.heading : Math.PI / 2;
        target = first.valid ? first.rpm : fallbackRPM;
    }

    // Moves the pretend robot and returns the RPM teleop would ask for there
    public double step(double dt) {
        double end = WorkingBotButtonMap.END_TOLERANCE;
        if (direction > 0 && x >= maxX - end) {
            direction = -1;
        } else if (direction < 0 && x <= minX + end) {
            direction = 1;
        }
        double dv = direction * SPEED - vx;
        double maxDv = ACCEL * dt;
        dv = Math.max(-maxDv, Math.min(maxDv, dv));
        double ax = dv / dt;
        vx += dv;
        x += vx * dt;
        // Robot is assumed to be aimed, so it turns with the aim heading (worked out without turning)
        ShotCalculator.Shot aim = ShotCalculator.solve(x, lineY, heading, vx, 0, ax, 0, 0, false);
        if (aim.valid) {
            double turn = Math.IEEEremainder(aim.heading - heading, 2 * Math.PI);
            // The first loop can be 2 ms, which made a huge turn rate
            turnRate += TURN_BLEND * (turn / Math.max(0.01, dt) - turnRate);
            heading = aim.heading;
        }
        ShotCalculator.Shot shot = ShotCalculator.solve(x, lineY, heading, vx, 0, ax, 0, turnRate, false);
        if (shot.valid) {
            // Keeps the last target on a bad solve, dropping to a fixed RPM for one loop restarted the ramp
            target = shot.rpm;
        }
        minTarget = Math.min(minTarget, target);
        maxTarget = Math.max(maxTarget, target);
        return target;
    }

    public double getX() {
        return x;
    }

    public double getVx() {
        return vx;
    }

    public double getMinTarget() {
        return minTarget;
    }

    public double getMaxTarget() {
        return maxTarget;
    }
}
