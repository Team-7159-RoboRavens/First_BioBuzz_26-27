package org.firstinspires.ftc.teamcode.Autonomous.Essentials;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftBack");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightBack");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        int id = 4;
        c.name.set("pinpoint");
        if (id == 4) {
            c.xPodOffset.set(-67.0);
            c.yPodOffset.set(-67.0);
        } else {
            c.xPodOffset.set(-155.0);
            c.yPodOffset.set(-60.0);
        }
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller forward = Controller.proportional(0.3);
        Controller forwardSecondary = Controller.proportional(0.1);
        Controller strafe = Controller.proportional(0.3);
        Controller strafeSecondary = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(forwardSecondary).put(2.5, forward));
        c.strafeTranslational.set(Controller.piecewise(strafeSecondary).put(2.5, strafe));
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
        c.maxAchievableForwardVelocity.set(65.4);
        c.maxAchievableStrafeVelocity.set(45.0);
        c.naturalForwardDeceleration.set(60.14);
        c.naturalStrafeDeceleration.set(61.79);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
