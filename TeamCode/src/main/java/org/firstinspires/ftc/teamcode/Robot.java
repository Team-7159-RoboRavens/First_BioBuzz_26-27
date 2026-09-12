package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBase;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseNewBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

// Wraps the drivetrain motors, intake motor, and Pinpoint odometry computer,
// exposing simple methods used by the button maps and TeleOp op modes.
public class Robot {
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotor intakeMotor1;

    GoBildaPinpointDriver pinpoint;

    VoltageSensor voltageSensor;

    InputHandling inputHandler = null;

    public List<LynxModule> hubs;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    private final int startPinpointX = -155;
    private final int startPinpointY = -60;


    /**
     * @param hmap hardware map used to look up the drivetrain motors, intake motor,
     *             and Pinpoint odometry computer by configuration name
     */
    public Robot(HardwareMap hmap, TeleOpBase teleOpBase) {
        hardwareMap = hmap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        if (teleOpBase != null){
            inputHandler = new InputHandling(this, teleOpBase);
        }


        leftFront = hmap.get(DcMotorEx.class, "leftFront");
        rightFront = hmap.get(DcMotorEx.class, "rightFront");
        leftBack = hmap.get(DcMotorEx.class, "leftBack");
        rightBack = hmap.get(DcMotorEx.class, "rightBack");
        intakeMotor1 = hmap.get(DcMotor.class, "intakeMotor");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Left front/right back motors are mounted mirrored, so their direction
        // must be reversed for all wheels to drive the same way for a given power sign.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configure the Pinpoint with its physical offset from the robot's center
        // and the odometry pod type/direction, then zero out position and heading.
        pinpoint.setOffsets(startPinpointX, startPinpointY, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

    }

    public Robot(TeleOpBaseNewBot teleOpBase, HardwareMap hmap) {
        hardwareMap = hmap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        if (teleOpBase != null){
            inputHandler = new InputHandling(this, teleOpBase);
        }


        leftFront = hmap.get(DcMotorEx.class, "leftFront");
        rightFront = hmap.get(DcMotorEx.class, "rightFront");
        leftBack = hmap.get(DcMotorEx.class, "leftBack");
        rightBack = hmap.get(DcMotorEx.class, "rightBack");


        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Left front/right back motors are mounted mirrored, so their direction
        // must be reversed for all wheels to drive the same way for a given power sign.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configure the Pinpoint with its physical offset from the robot's center
        // and the odometry pod type/direction, then zero out position and heading.

    }

    /**
     * @return hardware map this Robot was constructed with, for classes (e.g. Limelight)
     *         that need to look up their own hardware devices
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /**
     * @param power intake motor power, [-1, 1]
     */
    public void setIntakePower(double power) {
        if (intakeMotor1 != null) {
            intakeMotor1.setPower(power);
        }
    }

    /**
     * @param mp per-wheel motor powers to apply to the drivetrain
     */
    public void setPowers(MotorPowers mp) {
//        if (inputHandler == null) {
//            if (Math.max(Math.max(Math.abs(mp.lf), Math.abs(mp.rf)), Math.max(Math.abs(mp.lb), Math.abs(mp.rb))) > 1e-5) {
//                mp = inputHandler.getMotorPowers(mp);
//            }
//        }
        leftFront.setPower(mp.lf);
        rightFront.setPower(mp.rf);
        leftBack.setPower(mp.lb);
        rightBack.setPower(mp.rb);
    }

    /**
     * @return current Pinpoint heading, in radians
     */
    public double pinpointAngle() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
     * @return current Pinpoint X position, in mm
     */
    public double getPinpointX() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }

    /**
     * @return current Pinpoint Y position, in mm
     */
    public double getPinpointY(){
        return pinpoint.getPosY(DistanceUnit.MM);
    }

    // Must be called once per loop before reading any Pinpoint position/heading data.
    public void pinpointUpdate(){
        pinpoint.update();
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
}
