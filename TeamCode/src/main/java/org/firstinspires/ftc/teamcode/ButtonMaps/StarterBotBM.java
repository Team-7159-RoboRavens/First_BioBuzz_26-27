package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonMaps.*;
import org.firstinspires.ftc.teamcode.Robot;


public class StarterBotBM extends BasicBM{

    private boolean isShooting;

    private double shootRampUpTime = 3;
    private double power = 1;

    private ElapsedTime shootingTime = new ElapsedTime();

    /**
     * @param op running op mode, used to read gamepad state
     * @param r  robot instance this button map drives
     */
    public StarterBotBM(OpMode op, Robot r) {
        super(op, r);
        shootingTime.reset();
    }
    public void updateBot(){
        if (g1.bWasPressed()){
            getRobot().setHeadingHold(true);
        }
        if (g1.xWasPressed()){
            getRobot().setHeadingHold(false);
        }


        //enables shooting mode
        if (g1.dpad_down){
            if (isShooting) {
                getRobot().setShootPower(power);
                if (shootingTime.milliseconds()/1000 > shootRampUpTime){
                    getRobot().setShootFeedPower(1);
                }
            }
            else {
                isShooting = true;
                shootingTime.reset();
                getRobot().setShootPower(power);
            }
        }
    }

}
