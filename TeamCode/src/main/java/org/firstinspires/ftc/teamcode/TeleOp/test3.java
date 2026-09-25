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

@TeleOp(name="TestData3")
public class test3 extends OpMode {

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
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        //robot = new Robot(hardwareMap, null);
        bm = new BasicBM(this, robot);

        robot.getPositions();
        robot.getVelocities();
        if (robot.rightFrontVelocity != 0) {
            Log.i("rf end motor", "stop");

        } else {

            Log.i("rf motor increments", String.valueOf(robot.RFsmp));
            robot.RFsmp += 0.001;

            Log.i("rf motor increments after", String.valueOf(robot.RFsmp));
            robot.setStartingPowers();
        }
        if (robot.rightBackVelocity != 0) {
            Log.i("rb end motor", "stop");


        } else {
            Log.i("rb motor increments", String.valueOf(robot.RBsmp));
            robot.RBsmp += 0.001;
            Log.i("rb motor increments after", String.valueOf(robot.RBsmp));
            robot.setStartingPowers();
        }
        if (robot.leftBackVelocity != 0) {
            Log.i("lb end motor", "stop");



        } else {
            Log.i("lb motor increments", String.valueOf(robot.LBsmp));
            robot.LBsmp += 0.001;
            Log.i("lb motor increments after", String.valueOf(robot.LBsmp));
            robot.setStartingPowers();

        }
        if (robot.leftFrontVelocity != 0) {
            Log.i("lf end motor", "stop");


        } else {
            Log.i("lf motor increments", String.valueOf(robot.LFsmp));
            robot.LFsmp += 0.001;
            Log.i("lf motor increments after", String.valueOf(robot.LFsmp));
            robot.setStartingPowers();
        }


    }
}