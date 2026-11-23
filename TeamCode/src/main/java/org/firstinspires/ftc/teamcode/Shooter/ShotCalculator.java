package org.firstinspires.ftc.teamcode.Shooter;

// Works out where to point the shooter and what RPM to use so the ball lands in our HIVE's CELL on the robot's side,
// including while the robot is moving. Everything is in Pedro coordinates: inches from the corner, (72, 72) is the center.
public class ShotCalculator {
    // ---------------- Tuning: ball flight ----------------
    // Angle the ball leaves the shooter at, degrees up from flat. Steeper = higher arc and needs more speed for the same distance
    public static double LAUNCH_ANGLE_DEG = 70;
    // Height the ball leaves the shooter, inches off the floor. Measure on the robot
    public static double SHOOTER_HEIGHT = 14;
    // Height of the middle of the CELL opening the ball goes into, inches off the floor. Measure on the field.
    // Raising it asks for more RPM at long range than at short range
    public static double TARGET_HEIGHT = 50;

    // ---------------- Tuning: RPM ----------------
    // An RPM known to score from REFERENCE_DISTANCE (shooter to opening, flat distance, inches). This is the auto's
    // first shot, 192 RPM from SHOOT_POSE_1_0, which is 33.4 in from the opening. Every other distance is scaled from
    // this with the ball flight math above. If SHOOT_POSE_1_0 moves, update the distance (shown as "dist" in teleop telemetry)
    public static double REFERENCE_RPM = 192;
    public static double REFERENCE_DISTANCE = 33.4;
    // Added to every shot. Raise if every shot lands short, lower if every shot goes long
    public static double RPM_OFFSET = 0;
    // Extra RPM per inch past REFERENCE_DISTANCE (negative closer in). Raise if far shots land short but close shots are fine
    public static double RPM_PER_INCH = 0;
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
    public static double AIM_TRIM_DEG = 0;

    // ---------------- Tuning: shooting while moving ----------------
    // Seconds from deciding to shoot until the ball leaves. The robot's position and speed are predicted this far ahead
    public static double RELEASE_DELAY = 0.15;
    // How much of the robot's speed is aimed against (1 = all of it, 0 = aim as if standing still).
    // Raise if moving shots land past the opening in the direction the robot moves, lower if they land behind it
    public static double VELOCITY_COMPENSATION = 1.0;
    // How much of the robot's acceleration is used for the RELEASE_DELAY prediction (0 to ignore it)
    public static double ACCELERATION_COMPENSATION = 1.0;

    // ---------------- Robot geometry ----------------
    // Shooter position from the robot center, in the direction the shooter points: forward and to the left, inches
    public static double SHOOTER_FORWARD = 8.32;
    public static double SHOOTER_LEFT = -3.01;
    // Pedro heading = shooter heading + this, the shooter points out the back of the robot
    public static double PEDRO_HEADING_OFFSET = Math.PI;

    private static final double GRAVITY = 386.09; // in/s^2

    public static class Shot {
        public boolean valid;
        // Pedro heading the robot should face, radians
        public double heading;
        public double rpm;
        // Flat distance from the shooter to the aim point, inches
        public double distance;
        public double aimX;
        public double aimY;
    }

    /**
     * @param x robot center, Pedro inches
     * @param heading current Pedro heading, radians
     * @param vx robot field velocity, in/s
     * @param ax robot field acceleration, in/s^2
     * @param red true to aim at the red HIVE
     */
    public static Shot solve(double x, double y, double heading,
                             double vx, double vy, double ax, double ay, boolean red) {
        Shot shot = new Shot();
        double delay = RELEASE_DELAY;
        double accelX = ax * ACCELERATION_COMPENSATION;
        double accelY = ay * ACCELERATION_COMPENSATION;
        // Where the robot is and how fast it goes when the ball leaves
        double releaseX = x + vx * delay + 0.5 * accelX * delay * delay;
        double releaseY = y + vy * delay + 0.5 * accelY * delay * delay;
        double releaseVx = (vx + accelX * delay) * VELOCITY_COMPENSATION;
        double releaseVy = (vy + accelY * delay) * VELOCITY_COMPENSATION;

        double targetX = 72 + (red ? -HIVE_X_FROM_CENTER : HIVE_X_FROM_CENTER);
        double side = releaseY >= 72 ? 1 : -1;
        double targetY = 72 + side * CELL_Y_FROM_CENTER;

        // Shooter position with the current heading, close enough for the side shift
        double shooterHeading = heading - PEDRO_HEADING_OFFSET;
        double shooterX = releaseX + SHOOTER_FORWARD * Math.cos(shooterHeading) - SHOOTER_LEFT * Math.sin(shooterHeading);
        double shooterY = releaseY + SHOOTER_FORWARD * Math.sin(shooterHeading) + SHOOTER_LEFT * Math.cos(shooterHeading);
        double tanOffAngle = (shooterX - targetX) / Math.max(1, Math.abs(shooterY - targetY));
        double shift = Math.max(-MAX_OPENING_SHIFT, Math.min(MAX_OPENING_SHIFT, OPENING_SHIFT * tanOffAngle));
        double openingX = targetX - shift;

        // The ball keeps the robot's speed, so aim at where the opening seems to be from the moving robot
        double aimX = openingX;
        double aimY = targetY;
        double distance = 0;
        for (int i = 0; i < 4; i++) {
            distance = Math.hypot(aimX - shooterX, aimY - shooterY);
            double flightTime = flightTime(distance);
            if (Double.isNaN(flightTime)) {
                shot.valid = false;
                return shot;
            }
            aimX = openingX - releaseVx * flightTime;
            aimY = targetY - releaseVy * flightTime;
        }

        // Turn so the shooter (off to the side of the center) lines up with the aim point
        double centerDx = aimX - releaseX;
        double centerDy = aimY - releaseY;
        double centerDistance = Math.hypot(centerDx, centerDy);
        if (centerDistance <= Math.abs(SHOOTER_LEFT) + 1) {
            shot.valid = false;
            return shot;
        }
        double aimShooterHeading = Math.atan2(centerDy, centerDx) - Math.asin(SHOOTER_LEFT / centerDistance);
        shot.heading = aimShooterHeading + PEDRO_HEADING_OFFSET + Math.toRadians(AIM_TRIM_DEG);

        double speed = exitSpeed(distance);
        double referenceSpeed = exitSpeed(REFERENCE_DISTANCE);
        if (Double.isNaN(speed) || Double.isNaN(referenceSpeed)) {
            shot.valid = false;
            return shot;
        }
        double rpm = REFERENCE_RPM * speed / referenceSpeed + RPM_OFFSET + RPM_PER_INCH * (distance - REFERENCE_DISTANCE);
        shot.rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
        shot.distance = distance;
        shot.aimX = aimX;
        shot.aimY = aimY;
        shot.valid = true;
        return shot;
    }

    // Ball speed needed to reach the opening height at this flat distance, in/s, NaN if the arc can't get there
    public static double exitSpeed(double distance) {
        double angle = Math.toRadians(LAUNCH_ANGLE_DEG);
        double rise = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double cos = Math.cos(angle);
        double drop = distance * Math.tan(angle) - rise;
        if (distance <= 0 || drop <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(GRAVITY * distance * distance / (2 * cos * cos * drop));
    }

    // Seconds the ball is in the air for this flat distance, NaN if the arc can't get there
    public static double flightTime(double distance) {
        double speed = exitSpeed(distance);
        if (Double.isNaN(speed)) {
            return Double.NaN;
        }
        return distance / (speed * Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG)));
    }
}
