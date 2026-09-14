package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.IOException;

@TeleOp(name="TestData2")
public class test2 extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DataLogger dataLogger;
    Robot robot;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        try {
            dataLogger = new DataLogger("test_Data.txt");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot = new Robot(hardwareMap);
        Log.i("motorPowers", "logMessage");
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        String logMessage = String.format("leftFront: (%.2f)", leftFront, "rightFront: (%.2f)", rightFront);
        Log.i("motorPowers", logMessage);
        //moveMotors();
    }
}
    
