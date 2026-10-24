package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class WorkingBotButtonMap extends BasicBM{
    private double RPM = 0;

    public WorkingBotButtonMap(OpMode op, Robot r) {
        super(op, r);
    }


    //return a 3 value array where the first value is rpm of the shoot motor the
    // second is intake the third is feed motor and the 4th is the thing for getting stuff out of flowers
    public double[] getState(){
        double[] vals = new double[4];
        if (g2.dpadRightWasPressed()){
            RPM += 1;
        }
        if (g2.dpadUpWasPressed()){
            RPM += 10;
        }
        if (g2.dpadLeftWasPressed()){
            RPM -= 40;
            if (RPM < 0){
                RPM = 0;
            }
        }
        vals[0] = RPM;
        vals[1] = getIntakePower();
        if (g2.right_bumper){
            double feedPower = 0.7;
            vals[2] = feedPower;
        }
        if (g1.right_bumper){
            vals[3] = 2;
        }
        else if (g1.left_bumper){
            vals[3] = 1;
        }
        return vals;
    }
}
