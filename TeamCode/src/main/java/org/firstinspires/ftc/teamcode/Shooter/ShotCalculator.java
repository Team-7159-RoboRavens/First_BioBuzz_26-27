package org.firstinspires.ftc.teamcode.Shooter;

// Works out where to point the shooter and what RPM to use so the ball lands in our HIVE's CELL on the robot's side,
// including while the robot is moving. Everything is in Pedro coordinates: inches from the corner, (72, 72) is the center.
public class ShotCalculator {
    // ---------------- Tuning: ball flight ----------------
    // Angle the ball leaves the shooter at, degrees up from flat
    public static double LAUNCH_ANGLE_DEG = 70;
    // Ball speed leaving the shooter for each shooter RPM, in/s. Sets the flight time for moving shots: at 1.13 the
    // ball is in the air about 0.43 s at 32.7 in and 0.61 s at 46.2 in, and close shots go in near the top of the arc.
    // Lower it if moving shots land behind the opening (in the direction the robot moves), raise it if they land past it
    public static double BALL_SPEED_PER_RPM = 1.13;

    // ---------------- Tuning: RPM ----------------
    // Tested shots: distance ("dist" in teleop telemetry, inches) and the RPM that scored. Keep sorted by distance.
    // RPM is a straight line between the two nearest points, past the ends it follows the end slope
    public static double[] RPM_TABLE_DISTANCE = {32.7, 34.9, 46.2};
    public static double[] RPM_TABLE_RPM = {189, 191, 195};
    // Added to every shot. Raise if every shot lands short, lower if every shot goes long
    public static double RPM_OFFSET = 0;
    public static double MIN_RPM = 120;
    public static double MAX_RPM = 320;

    // ---------------- Tuning: aim point ----------------
    // Where the opening is. Our HIVE is this far from the center line along x (blue +x, red -x), inches
    public static double HIVE_X_FROM_CENTER = 12.75;
    // The CELL opening is this far from the field center along y on the robot's side, inches
    public static double CELL_Y_FROM_CENTER = 10.3;
    // When the shooter is off to one side of the opening, aim this many inches toward the other side for each unit of
    // tan(angle off straight). The opening has depth, so its middle as the ball sees it moves. Make it negative if
    // side shots miss on the far side instead
    public static double OPENING_SHIFT = 1.0;
    // Largest aim shift from the opening, inches
    public static double MAX_OPENING_SHIFT = 3;
    // Turns every shot this many degrees counterclockwise, for a shooter that pulls to one side
    // 5.71 deg = 5 in left at 50 in (atan(5 / 50)), shots were landing right
    public static double AIM_TRIM_DEG = 5.71;

    // ---------------- Tuning: shooting while moving ----------------
    // Seconds from deciding to shoot until the ball leaves. The robot's position and speed are predicted this far ahead
    public static double RELEASE_DELAY = 0.15;
    // How much of the robot's speed is aimed against (1 = all of it, 0 = aim as if standing still).
    // Raise if moving shots land past the opening in the direction the robot moves, lower if they land behind it
    public static double VELOCITY_COMPENSATION = 1.0;
    // How much of the robot's acceleration is used for the RELEASE_DELAY prediction (0 to ignore it)
    public static double ACCELERATION_COMPENSATION = 1.0;
    // How much of the extra shooter speed from the robot turning is aimed against (0 to ignore it). The shooter is
    // off the robot's center, so while turning it moves sideways even when the center stands still
    public static double ROTATION_COMPENSATION = 1.0;

    // ---------------- Robot geometry ----------------
    // Shooter position from the robot center, measured in the direction the shooter points: forward and to the
    // left, inches. The shooter faces the robot's back, so this is 8.32 in toward the back and 3.01 in to the robot's
    // left (looking from the intake end). Measure it to where the ball leaves
    public static double SHOOTER_FORWARD = 8.32;
    public static double SHOOTER_LEFT = -3.01;
    // Pedro heading = shooter heading + this, the shooter points out the back of the robot
    public static double PEDRO_HEADING_OFFSET = Math.PI;

    public static class Shot {
        public boolean valid;
        // Pedro heading the robot should face, radians
        public double heading;
        public double rpm;
        // Flat distance from the shooter to the aim point, inches (the aim point moves with the robot's speed)
        public double distance;
        // Flat distance from the shooter to the opening itself, inches
        public double openingDistance;
        public double aimX;
        public double aimY;
        // Seconds the ball is expected to be in the air
        public double flightTime;
        // How fast the aim heading changes as the robot moves, rad/s counterclockwise, for the turn feedforward
        public double headingRate;
    }

    /**
     * @param x robot center, Pedro inches
     * @param heading current Pedro heading, radians
     * @param vx robot field velocity, in/s
     * @param ax robot field acceleration, in/s^2
     * @param omega robot turn rate, rad/s counterclockwise
     * @param red true to aim at the red HIVE
     */
    public static Shot solve(double x, double y, double heading,
                             double vx, double vy, double ax, double ay, double omega, boolean red) {
        Shot shot = new Shot();
        double delay = RELEASE_DELAY;
        double accelX = ax * ACCELERATION_COMPENSATION;
        double accelY = ay * ACCELERATION_COMPENSATION;
        // Where the robot is and how fast it goes when the ball leaves
        double releaseX = x + vx * delay + 0.5 * accelX * delay * delay;
        double releaseY = y + vy * delay + 0.5 * accelY * delay * delay;
        double centerVx = (vx + accelX * delay) * VELOCITY_COMPENSATION;
        double centerVy = (vy + accelY * delay) * VELOCITY_COMPENSATION;
        double releaseHeading = heading + omega * delay;

        double targetX = 72 + (red ? -HIVE_X_FROM_CENTER : HIVE_X_FROM_CENTER);
        double side = releaseY >= 72 ? 1 : -1;
        double targetY = 72 + side * CELL_Y_FROM_CENTER;

        // Shooter position when the ball leaves
        double shooterHeading = releaseHeading - PEDRO_HEADING_OFFSET;
        double cos = Math.cos(shooterHeading);
        double sin = Math.sin(shooterHeading);
        double offsetX = SHOOTER_FORWARD * cos - SHOOTER_LEFT * sin;
        double offsetY = SHOOTER_FORWARD * sin + SHOOTER_LEFT * cos;
        double shooterX = releaseX + offsetX;
        double shooterY = releaseY + offsetY;
        // The ball gets the shooter's speed, not the center's: turning swings the shooter around the center
        double spin = omega * ROTATION_COMPENSATION;
        double releaseVx = centerVx - spin * offsetY;
        double releaseVy = centerVy + spin * offsetX;
        double tanOffAngle = (shooterX - targetX) / Math.max(1, Math.abs(shooterY - targetY));
        double shift = Math.max(-MAX_OPENING_SHIFT, Math.min(MAX_OPENING_SHIFT, OPENING_SHIFT * tanOffAngle));
        double openingX = targetX - shift;
        shot.openingDistance = length(openingX - shooterX, targetY - shooterY);

        // The ball keeps the shooter's speed, so aim at where the opening seems to be from the moving shooter.
        // Speed toward the HIVE shortens that distance, sideways speed moves the aim to the side. The RPM for the
        // shortened distance comes from the table, then the flight time from that RPM, repeated until they agree
        double aimX = openingX;
        double aimY = targetY;
        double distance = 0;
        double flightTime = 0;
        for (int i = 0; i < 10; i++) {
            distance = length(aimX - shooterX, aimY - shooterY);
            flightTime = flightTime(distance);
            if (Double.isNaN(flightTime)) {
                shot.valid = false;
                return shot;
            }
            aimX = openingX - releaseVx * flightTime;
            aimY = targetY - releaseVy * flightTime;
        }
        distance = length(aimX - shooterX, aimY - shooterY);

        // Turn so the shooter (off to the side of the center) lines up with the aim point
        double centerDx = aimX - releaseX;
        double centerDy = aimY - releaseY;
        double centerDistance = length(centerDx, centerDy);
        if (centerDistance <= Math.abs(SHOOTER_LEFT) + 1) {
            shot.valid = false;
            return shot;
        }
        double aimShooterHeading = Math.atan2(centerDy, centerDx) - Math.asin(SHOOTER_LEFT / centerDistance);
        shot.heading = aimShooterHeading + PEDRO_HEADING_OFFSET + Math.toRadians(AIM_TRIM_DEG);
        // The aim point stays put in the field while the speed holds, so the heading to it turns at this rate
        double moveVx = vx + accelX * delay;
        double moveVy = vy + accelY * delay;
        shot.headingRate = (centerDy * moveVx - centerDx * moveVy) / (centerDistance * centerDistance);

        double rpm = tableRPM(distance) + RPM_OFFSET;
        shot.rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
        shot.distance = distance;
        shot.flightTime = flightTime;
        shot.aimX = aimX;
        shot.aimY = aimY;
        shot.valid = true;
        return shot;
    }

    // RPM from the tested shots table for this distance
    public static double tableRPM(double distance) {
        double[] d = RPM_TABLE_DISTANCE;
        double[] r = RPM_TABLE_RPM;
        int n = Math.min(d.length, r.length);
        if (n == 0) {
            return MIN_RPM;
        }
        if (n == 1) {
            return r[0];
        }
        // Segment to use, the end ones are extended past the table
        int i = 0;
        while (i < n - 2 && distance > d[i + 1]) {
            i++;
        }
        double span = d[i + 1] - d[i];
        if (span == 0) {
            return r[i];
        }
        return r[i] + (r[i + 1] - r[i]) * (distance - d[i]) / span;
    }

    // Seconds the ball is in the air for this flat distance when shot at the table RPM, NaN if it can't be worked out
    public static double flightTime(double distance) {
        double rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, tableRPM(distance) + RPM_OFFSET));
        double flatSpeed = BALL_SPEED_PER_RPM * rpm * Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG));
        if (distance <= 0 || flatSpeed <= 0) {
            return Double.NaN;
        }
        return distance / flatSpeed;
    }

    // Same as Math.hypot without its slow overflow handling, these are field distances in inches
    private static double length(double dx, double dy) {
        return Math.sqrt(dx * dx + dy * dy);
    }
}
