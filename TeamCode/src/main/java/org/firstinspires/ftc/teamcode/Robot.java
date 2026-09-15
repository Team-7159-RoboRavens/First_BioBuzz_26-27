package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wraps the drivetrain motors, intake motor, and Pinpoint odometry computer,
// exposing simple methods used by the button maps and TeleOp op modes.
public class Robot {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor intakeMotor1;
    public DcMotorEx flywheelMotor;
    public double F = 0;
    public double P = 0;

    GoBildaPinpointDriver pinpoint;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    //TODO: try to use the values defined from Constants.java, instead of creating two sources of the same values
    private final int startPinpointX = -155;
    private final int startPinpointY = -60;


    /**
     * @param hmap hardware map used to look up the drivetrain motors, intake motor,
     *             and Pinpoint odometry computer by configuration name
     */
    public Robot(HardwareMap hmap) {
        hardwareMap = hmap;

        leftFront = hmap.get(DcMotor.class, "leftFront");
        rightFront = hmap.get(DcMotor.class, "rightFront");
        leftBack = hmap.get(DcMotor.class, "leftBack");
        rightBack = hmap.get(DcMotor.class, "rightBack");
        intakeMotor1 = hmap.get(DcMotor.class, "intakeMotor");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: why do we need brake for the intake wheel instead of float?
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        flywheelMotor = hmap.get(DcMotorEx.class,"motor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
        intakeMotor1.setPower(power);
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

    public double limelightAngle() { return pinpointAngle();}

    // TODO: how is this function different from setPowers above
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
