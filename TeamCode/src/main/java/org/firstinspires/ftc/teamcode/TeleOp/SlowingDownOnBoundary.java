package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Limelight.calculatePosition.toPedroPose;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;



import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

public class SlowingDownOnBoundary extends Robot {

    private static final double FIELD_MIN = 0.0;
    private static final double FIELD_MAX = 141.5;

    private static final double SLOW_DISTANCE = 20.0;
    LimelightBlue calculatePosition = new LimelightBlue();

    private static final double MIN_RATE = 0.15;

    private final BasicBM basicBM;

    public SlowingDownOnBoundary(
            HardwareMap hmap,
            int robotId,
            BasicBM basicBM
    ) {
        super(hmap, robotId);
        this.basicBM = basicBM;
    }

    public void init() {
    }

    public void loop() {
        MotorPowers mp = basicBM.getPowers();
        double[] fieldPosition= calculatePosition.getPosition();
        Pose pedroPose = toPedroPose(fieldPosition);
        double x = pedroPose.getX();
        double y = pedroPose.getY();


        double distanceLeft = x - FIELD_MIN;
        double distanceRight = FIELD_MAX - x;

        double distanceBottom = y - FIELD_MIN;
        double distanceTop = FIELD_MAX - y;

        double closestDistance = Math.min(
                Math.min(distanceLeft, distanceRight),
                Math.min(distanceBottom, distanceTop)
        );

        double rate;

        if (closestDistance >= SLOW_DISTANCE) {
            rate = 1.0;
        } else if (closestDistance <= 0) {
            rate = MIN_RATE;
        } else {
            rate = closestDistance / SLOW_DISTANCE;
            rate = Math.max(rate, MIN_RATE);
        }

        mp.lf *= rate;
        mp.rf *= rate;
        mp.lb *= rate;
        mp.rb *= rate;

        setPowers(mp);
    }
}
