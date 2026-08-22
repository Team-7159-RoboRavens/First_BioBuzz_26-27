package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor intakeMotor1;

    GoBildaPinpointDriver pinpoint;

    // x and y positions of pinpoint relative to robot center in mm
    private final int pinpointX = 0;
    private final int pinpointY = 0;

    public Robot(HardwareMap hmap) {
        leftFront = hmap.get(DcMotor.class, "leftFront");
        rightFront = hmap.get(DcMotor.class, "rightFront");
        leftBack = hmap.get(DcMotor.class, "leftBack");
        rightBack = hmap.get(DcMotor.class, "rightBack");
        intakeMotor1 = hmap.get(DcMotor.class, "intakeMotor1");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        pinpoint.setOffsets(pinpointX, pinpointY, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

    }

    public void setIntakePower(double power) {
        intakeMotor1.setPower(power);
    }
    public void setPowers(MotorPowers mp) {
        leftFront.setPower(mp.lf);
        rightFront.setPower(mp.rf);
        leftBack.setPower(mp.lb);
        rightBack.setPower(mp.rb);
    }

    public void setAcleratePowers(MotorPowers mp){
        leftFront.setPower(mp.lf);
        rightFront.setPower(mp.rf);
        leftBack.setPower(mp.lb);
        rightBack.setPower(mp.rb);
    }
}
