package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.MotorPowers;

public class WheelTestBM extends BasicBM{
    public WheelTestBM(OpMode op, Robot r) {
        super(op, r);
    }
    private boolean turnForwardWheels = true;
    public MotorPowers getTestPowers(){
        MotorPowers powers = new MotorPowers();
        if (g1.y){
            turnForwardWheels = true;
        }
        else if (g1.a){
            turnForwardWheels = false;
        }
        double pressedRight = g1.right_trigger;
        double pressedLeft = g1.left_trigger;
        if (pressedRight > 0.1){
            if (turnForwardWheels){
                powers.rf = pressedRight;
            }
            else {
                powers.rb = pressedRight;
            }
        }
        if (pressedLeft > 0.1){
            if (turnForwardWheels){
                powers.lf = pressedLeft;
            }
            else {
                powers.lb = pressedLeft;
            }
        }
        return powers;
    }
}
