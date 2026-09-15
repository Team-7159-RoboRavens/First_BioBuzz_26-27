package org.firstinspires.ftc.teamcode.TeleOp;

import java.lang.String;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.AnalogCommands;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.IOException;

@TeleOp(name="TestData2")
public class test2 extends OpMode {

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DataLogger dataLogger;
    Robot robot;
    BasicBM bm;
    VoltageSensor voltageSensor;




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
        robot = new Robot(hardwareMap, null);
        bm = new BasicBM(this, robot);
        voltageSensor = new VoltageSensor() {
            @Override
            public double getVoltage() {
                return 0;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };

        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);

        String logMessage;
        logMessage = String.format("leftFront: (%.2f)", mp.lf, "rightFront: (%.2f)", mp.rf);
        String logBack;
        logBack = String.format("leftBack: (%.2f)", mp.lb, "rightBack: (%.2f)", mp.rb);
        Log.i("motorPowers", logMessage);
        Log.i("motor powers", logBack);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        robot = new Robot(hardwareMap, null);
        bm = new BasicBM(this, robot);
        robot.getPositions();
        robot.getVelocities();


        MotorPowers mp = new MotorPowers(1, 1, 1, 1);
        robot.setPowers(mp);



        telemetry.addData("motor powers", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("motor powers", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);

        String logMessage;
        logMessage = String.format("leftFront: (%.2f)", robot.leftFrontCurrentPos);
        String logRight;
        logRight = String.format("rightFront: (%.2f)", robot.rightFrontCurrentPos);
        String logBack;
        logBack = String.format("leftBack: (%.2f)", robot.leftBackCurrentPos);
        String RightBack;
        RightBack = String.format("rightBack: (%.2f)", robot.rightBackCurrentPos);
        Log.i("motorPowers", logMessage);
        Log.i("motorPowers", logBack);
        Log.i("motorPowers", logRight);
        Log.i("motorPowers", RightBack);

        String lfVelocity;
        lfVelocity = String.format("leftFront: (%.2f)", robot.leftFrontVelocity);
        String rfVelocity;
        rfVelocity = String.format("rightFront: (%.2f)", robot.rightFrontVelocity);
        String lbVelocity;
        lbVelocity = String.format("leftBack: (%.2f)", robot.leftBackVelocity);
        String rbVelocity;
        rbVelocity = String.format("rightBack: (%.2f)", robot.rightBackVelocity);
        Log.i("velocity", lfVelocity);
        Log.i("velocity", rfVelocity);
        Log.i("velocity", lbVelocity);
        Log.i("velocity", rbVelocity);

        String voltage;
        voltage = String.format("voltage: (%.2f)", voltageSensor.getVoltage());
        Log.i("voltage", voltage);




        String volt;
        volt = String.format("voltage: (%.2f)", voltageSensor.getVoltage());
        Log.i("voltage", volt);






    }
}
    
