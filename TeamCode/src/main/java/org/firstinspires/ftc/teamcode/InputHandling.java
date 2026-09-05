package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class InputHandling {
    private final double MAX_RPM = 1500;

    private final double INITIAL_VOLTAGE = 12;

    private final double MOTOR_TORQUE_CONSTANT = 1;

    private final double FRICTIONAL_TORQUE = 1;

    private final double GEAR_RATIO = 1;
    private final double TOTAL_ROTAIONAL_INIRTIA = 1;

    private final double RADIAN_TO_RPM = Math.PI / 30;

    //ticks per revolution of motor
    private final double TICKS = 383.6;

    private final int SECONDS_PER_MINUTE = 60;
    private double maxRPM = MAX_RPM;
    private Robot robot;

    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;

    private VoltageSensor voltageSensor;

    private double batteryVoltage;
    public InputHandling(Robot x){
        robot = x;
        voltageSensor = robot.voltageSensor;
//        lf = robot.leftFront;
//        rf = robot.rightFront;
//        lf = robot.leftBack;
//        rf = robot.rightBack;
    }

    public void updateVoltage(){
        batteryVoltage = voltageSensor.getVoltage();
        maxRPM = MAX_RPM * batteryVoltage / INITIAL_VOLTAGE;
    }


    public MotorPowers getMotorPowers(MotorPowers power){
        double[] targetRPM = new double[4];
        targetRPM[0] = power.lf * maxRPM;
        targetRPM[1] = power.rf * maxRPM;
        targetRPM[2] = power.lb * maxRPM;
        targetRPM[3] = power.rf * maxRPM;

        double[] RPM = new double[4];
        RPM[0] = lf.getVelocity() * SECONDS_PER_MINUTE / TICKS;
        RPM[1] = rf.getVelocity() * SECONDS_PER_MINUTE / TICKS;
        RPM[2] = lb.getVelocity() * SECONDS_PER_MINUTE / TICKS;
        RPM[3] = rf.getVelocity() * SECONDS_PER_MINUTE / TICKS;

        double[] diff = new double[4];
        for (int i = 0; i < 4; i++){
            diff[i] = RPM[i] - targetRPM[i];
        }

        int maxIndex = maxIndex(diff);

        boolean isAccelerating = (RPM[maxIndex] > 0 && targetRPM[maxIndex] > RPM[maxIndex]) || (RPM[maxIndex] < 0 && targetRPM[maxIndex] < RPM[maxIndex]);
        //calculates the acceleration of the motor that needs the greatest change in rpm
        double maxAcel = maxRPMAcel(RPM[maxIndex], maxIndex, isAccelerating);

        double[] inputVoltages = calculateInputVoltages(maxAcel, maxIndex, RPM, targetRPM, diff);

        MotorPowers setPower = calculateMotorPowers(inputVoltages);

        return setPower;
    }

    private MotorPowers calculateMotorPowers(double[] inputVoltages){
        MotorPowers powers = new MotorPowers();
        double max = inputVoltages[maxIndex(inputVoltages)];
        if (max > 1){
            for (int i = 0; i < 4; i++){
                inputVoltages[i] /= max;
            }
        }
        powers.lf = inputVoltages[0] / batteryVoltage;
        powers.rf = inputVoltages[1] / batteryVoltage;
        powers.lb = inputVoltages[2] / batteryVoltage;
        powers.rb = inputVoltages[3] / batteryVoltage;
        return powers;
    }
    private double[] calculateInputVoltages(double maxAcel, int motor, double[] RPM, double[] targetRPM, double[] diff){
        double[] voltages = new double[4];
        for (int i = 0; i < 4; i ++){
            if (i != motor) {
                double motorAcel = maxAcel * diff[i] / diff[motor];
                Double voltage = calculateMotorVoltage(motorAcel, RPM[i]);
                if (voltage == null){
                    boolean isAcelerating = (RPM[i] > 0 && targetRPM[i] > RPM[i]) || (RPM[i] < 0 && targetRPM[i] < RPM[i]);
                    maxAcel = maxRPMAcel(RPM[i], i, isAcelerating);
                    return calculateInputVoltages(maxAcel, i, RPM, targetRPM, diff);
                }
                voltages[i] = voltage;
            }
        }
        return voltages;
    }

    private Double calculateMotorVoltage(double acel, double RPM){

        return null;
    }
    private double maxRPMAcel(double RPM, int motor, boolean isAcelerating){
        int sign = -1;
        if (isAcelerating){
            sign = 1;
        }
        double numerator = batteryVoltage - (RPM) / MOTOR_TORQUE_CONSTANT - sign * FRICTIONAL_TORQUE / MOTOR_TORQUE_CONSTANT / GEAR_RATIO;
        double denominator = TOTAL_ROTAIONAL_INIRTIA * RADIAN_TO_RPM / MOTOR_TORQUE_CONSTANT / GEAR_RATIO;
        double acel = numerator/denominator;
        return acel;
    }
    private int maxIndex(double[] a){
        double max = Math.abs(a[0]);
        int index = 0;
        for(int i = 1; i < a.length; i++){
            double c = Math.abs(a[i]);
            if (c > max){
                index = i;
                max = c;
            }
        }
        return index;
    }
}
