package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wraps the drivetrain motors, intake motor, and Pinpoint odometry computer,
// exposing simple methods used by the button maps and TeleOp op modes.
public class BarebonesRobot {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    private GoBildaPinpointDriver pinpoint;


    /**
     * @param hmap hardware map used to look up the drivetrain motors, intake motor,
     *             and Pinpoint odometry computer by configuration name
     */
    public BarebonesRobot(HardwareMap hmap) {
        leftFront = hmap.get(DcMotor.class, "leftFront");
        rightFront = hmap.get(DcMotor.class, "rightFront");
        leftBack = hmap.get(DcMotor.class, "leftBack");
        rightBack = hmap.get(DcMotor.class, "rightBack");
        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-155, -60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Left front/right back motors are mounted mirrored, so their direction
        // must be reversed for all wheels to drive the same way for a given power sign.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    /**
     * @param mp per-wheel motor powers to apply to the drivetrain
     */
    public void setPowers(MotorPowers mp) {
        leftFront.setPower(mp.lf);
        rightFront.setPower(mp.rf);
        leftBack.setPower(mp.lb);
        rightBack.setPower(mp.rb);
    }

    /**
     * @param mp per-wheel motor powers to apply to the drivetrain
     */
    public void setAcleratePowers(MotorPowers mp){
        leftFront.setPower(mp.lf);
        rightFront.setPower(mp.rf);
        leftBack.setPower(mp.lb);
        rightBack.setPower(mp.rb);
    }


    public void resetPinpoint(){
        pinpoint.resetPosAndIMU();
    }
    public void pinpointUpdate() {
        pinpoint.update();
    }

    public double pinpointAngle() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }
}
