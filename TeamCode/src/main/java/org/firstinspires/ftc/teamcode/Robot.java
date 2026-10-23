package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

// Wraps the drivetrain motors, intake motor, and Pinpoint odometry computer,
// exposing simple methods used by the button maps and TeleOp op modes.
public class Robot {
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    public DcMotorEx shootMotor;
    DcMotor intakeMotor1;
    public CRServo shootFeedServo;
    public Servo flowerServo;
    public DcMotorEx flywheelMotor;
    public double F = 12;
    public double P = 36;
    public double I = 4;
    public double D = 0;
    GoBildaPinpointDriver pinpoint;

    VoltageSensor voltageSensor;

    InputHandling inputHandler = null;

    CRServo intakeServoLeft;
    CRServo intakeServoRight;


    public List<LynxModule> hubs;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    //TODO: try to use the values defined from Constants.java, instead of creating two sources of the same values
    private final int startPinpointX = -155;
    private final int startPinpointY = -60;

    public boolean inputHandlingEnabled = false;


    /**
     * @param hmap hardware map used to look up the drivetrain motors, intake motor,
     *             and Pinpoint odometry computer by configuration name
     */
    public Robot(HardwareMap hmap, int robotId) {
        hardwareMap = hmap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if (robotId == 7){
            return;
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        leftFront = hmap.get(DcMotorEx.class, "leftFront");
        rightFront = hmap.get(DcMotorEx.class, "rightFront");
        leftBack = hmap.get(DcMotorEx.class, "leftBack");
        rightBack = hmap.get(DcMotorEx.class, "rightBack");
        shootFeedServo = hmap.get(CRServo.class, "shootFeedServo");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor = hmap.get(DcMotorEx.class,"shootMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,D,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Left front/right back motors are mounted mirrored, so their direction
        // must be reversed for all wheels to drive the same way for a given power sign.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configure the Pinpoint with its physical offset from the robot's center
        // and the odometry pod type/direction, then zero out position and heading.
        if (robotId == 0 || robotId == 4) {
            intakeMotor1 = hmap.get(DcMotor.class, "intakeMotor");
            intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (robotId == 0 || robotId == 3 || robotId == 4) {
            pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(startPinpointX, startPinpointY, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pinpoint.resetPosAndIMU();
        }

        if (robotId != 3){
            inputHandler = new InputHandling(this);
        }

        if (robotId == 4){
            shootFeedServo = hardwareMap.get(CRServo.class, "shootFeedServo");
            intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
            intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
            shootMotor = hmap.get(DcMotorEx.class, "shootMotor");
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

        }
        if (robotId == 2){
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        }
        if (robotId == 10){
            shootFeedServo = hardwareMap.get(CRServo.class, "transferServo");
            flowerServo = hardwareMap.get(Servo.class, "flowerServo");
            shootMotor = hmap.get(DcMotorEx.class, "shootMotor");
            intakeMotor1 = hmap.get(DcMotor.class, "intakeMotor");
            shootFeedServo.setDirection(CRServo.Direction.REVERSE);
            intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shootMotor.setDirection(DcMotor.Direction.REVERSE);

        }

    }

    public Robot(HardwareMap hmap) {
        this(hmap, 4);
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
        if (intakeServoRight != null && intakeServoLeft != null){
            intakeServoRight.setPower(-power);
            intakeServoLeft.setPower(power);
        }
    }

    public void setShootFeedPower(double power){
        if (shootFeedServo != null) {
            shootFeedServo.setPower(power);
        }
    }
    public void setShootPower(double power){
        if (shootMotor != null) {
            shootMotor.setPower(power);
        }
    }
    public void setShootRPM(double RPM){
        if (shootMotor != null) {
            double targetTicksPerSec = (RPM / 60.0) * 383.6;
            shootMotor.setVelocity(targetTicksPerSec);
        }
    }

    public void tuneMotorPowers(MotorPowers mp){
        if (inputHandler != null && inputHandlingEnabled) {
            if (Math.max(Math.max(Math.abs(mp.lf), Math.abs(mp.rf)), Math.max(Math.abs(mp.lb), Math.abs(mp.rb))) > 1e-2) {
                mp = inputHandler.getMotorPowers(mp);
            }
        }
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


    public void setHeadingHold(boolean headingHold){
        inputHandler.setLockHeading(headingHold);
    }

    public void setFlowerState(double val) {
        if (val == 2){
            flowerServo.setPosition(0.8);
        }
        else if (val == 1){
            flowerServo.setPosition(0.35);
        }
    }
}
