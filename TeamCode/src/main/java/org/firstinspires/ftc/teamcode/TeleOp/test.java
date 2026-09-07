package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TestData")
public class test extends OpMode {
    public MotorPowers moveMotors;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    private OpMode opMode;
    private Gamepad g2;

    //creates a string TAG that will be added to with the data collected
    private static final String TAG = "MyActivity";



    Robot robot;
    public test(OpMode op, Robot r) {
        //opMode = op;
        //robot = r;
        //g2 = opMode.gamepad2;
    }
    public void moveMotors() {
        if(g2.a) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);}
        
        if(g2.b) {
            leftFront.setPower(0.6);
            rightFront.setPower(0.6);
            leftBack.setPower(0.6);
            rightBack.setPower(0.6);
        }

        if(g2.x) {
            leftFront.setPower(0.7);
            rightFront.setPower(0.7);
            leftBack.setPower(0.7);
            rightBack.setPower(0.7);
        }

        if(g2.y) {
            leftFront.setPower(0.8);
            rightFront.setPower(0.8);
            leftBack.setPower(0.8);
            rightBack.setPower(0.8);

        }

        if(g2.left_bumper) {
            leftFront.setPower(0.9);
            rightFront.setPower(0.9);
            leftBack.setPower(0.9);
            rightBack.setPower(0.9);
        }



        
        
    }

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        //Log.i("hello", "world");
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        //moveMotors();
    }
}
    
