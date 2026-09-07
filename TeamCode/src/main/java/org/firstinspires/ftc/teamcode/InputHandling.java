//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBase;
//public class InputHandling {
//    private final double MAX_RPM = 1500;
//
//    private final double INITIAL_VOLTAGE = 12;
//
//    private final double[] MOTOR_TORQUE_CONSTANT = {1, 1, 1, 1};
//
//    private final double[] FRICTIONAL_TORQUE = {1, 1, 1, 1};
//
//    private final double[] GEAR_RATIO = {1, 1, 1, 1};
//    private final double[] TOTAL_ROTAIONAL_INIRTIA = {1, 1, 1, 1};
//
//    private final double RADIAN_TO_RPS = Math.PI/30;
//
//    //ticks per revolution of motor
//    private final double TICKS = 383.6;
//
//    private final double[] ENERGY_TOTAL = {1, 1, 1, 1};
//
//    private final double[] MOTOR_VELOCITY_CONSTANT = {1, 1, 1, 1};
//
//    private final int SECONDS_PER_MINUTE = 60;
//
//    private final double FRACTION_UNDER_SHOOT = 0.7;
//
//    private final double TURN_AJUST_SCALE = 0.1;
//    private double maxRPM = MAX_RPM;
//
//    private double frameTime = 0.02;
//    private Robot robot;
//
//    private DcMotorEx lf;
//    private DcMotorEx rf;
//    private DcMotorEx lb;
//    private DcMotorEx rb;
//
//    private TeleOpBase teleOp;
//
//    private VoltageSensor voltageSensor;
//
//    private int lockHeading;
//    private double lockAngle;
//
//    private double[] angles = new double[5];
//
//    private boolean disableLockHeading;
//
//    private double batteryVoltage;
//    public InputHandling(Robot a, TeleOpBase b){
//        robot = a;
//        teleOp = b;
//        voltageSensor = robot.voltageSensor;
//        lf = robot.leftFront;
//        rf = robot.rightFront;
//        lb = robot.leftBack;
//        rb = robot.rightBack;
//    }
//
//    public void updateVoltage(){
//        batteryVoltage = voltageSensor.getVoltage();
//        maxRPM = MAX_RPM * batteryVoltage / INITIAL_VOLTAGE;
//    }
//
//
//    public MotorPowers getMotorPowers(MotorPowers power){
//        frameTime = teleOp.averageFrameTime;
//        double[] targetRPM = new double[4];
//        targetRPM[0] = power.lf * maxRPM;
//        targetRPM[1] = power.rf * maxRPM;
//        targetRPM[2] = power.lb * maxRPM;
//        targetRPM[3] = power.rb * maxRPM;
//
//        //if opposite wheels are doing the same thing the robot is not trying to rotate;
//        if (targetRPM[0] == targetRPM[4] && targetRPM[2] == targetRPM[3]){
//            if (lockHeading < 5) {
////                angles[lockHeading] = robot.pinpointAngle();
//                lockHeading++;
//                if (lockHeading == 5){
//                    double sum = 0;
//                    for (int i = 0; i < 5; i++){
//                        sum += angles[i];
//                    }
//                    lockAngle = sum / 5;
//                    while (lockAngle > Math.PI){
//                        lockAngle -= Math.PI;
//                    }
//                }
//            }
//        }
//        else {
//            lockHeading = 0;
//        }
//        if (!disableLockHeading) {
//            if (lockHeading == 5) {
//                double angleErrorRight = robot.pinpointAngle() - lockAngle;
//                double angleErrorLeft = 2 * Math.PI - angleErrorRight;
//
//                if (angleErrorLeft < 0) {
//                    double temp = angleErrorRight;
//                    angleErrorRight = angleErrorLeft;
//                    angleErrorLeft = temp;
//                }
//
//                double turnCorrection = 0;
//
//                if (angleErrorLeft < angleErrorRight) {
//                    turnCorrection = -TURN_AJUST_SCALE * angleErrorLeft;
//                } else {
//                    turnCorrection = TURN_AJUST_SCALE * angleErrorRight;
//                }
//                targetRPM[0] += turnCorrection * maxRPM;
//                targetRPM[2] += turnCorrection * maxRPM;
//                targetRPM[1] -= turnCorrection * maxRPM;
//                targetRPM[3] -= turnCorrection * maxRPM;
//                double max = targetRPM[maxIndex(targetRPM)];
//                if (max > maxRPM){
//                    for (int i = 0; i < 4; i ++){
//                        targetRPM[i] *= maxRPM / max;
//                    }
//                }
//            }
//        }
//
//        double[] RPM = new double[4];
//        RPM[0] = lf.getVelocity() * SECONDS_PER_MINUTE / TICKS;
//        RPM[1] = rf.getVelocity() * SECONDS_PER_MINUTE / TICKS;
//        RPM[2] = lb.getVelocity() * SECONDS_PER_MINUTE / TICKS;
//        RPM[3] = rb.getVelocity() * SECONDS_PER_MINUTE / TICKS;
//
//        double[] diff = new double[4];
//        for (int i = 0; i < 4; i++){
//            diff[i] = targetRPM[i] - RPM[i];
//        }
//
//        int maxIndex = maxIndex(diff);
//
//        boolean isAccelerating = (RPM[maxIndex] > 0 && targetRPM[maxIndex] > RPM[maxIndex]) || (RPM[maxIndex] < 0 && targetRPM[maxIndex] < RPM[maxIndex]);
//        //calculates the acceleration of the motor that needs the greatest change in rpm
//        double maxAcel = maxRPMAcel(RPM[maxIndex], targetRPM[maxIndex], maxIndex, isAccelerating);
//
//        double[] inputVoltages = calculateInputVoltages(maxAcel, maxIndex, RPM, targetRPM, diff, 0);
//
//        return calculateMotorPowers(inputVoltages);
//    }
//
//    private MotorPowers calculateMotorPowers(double[] inputVoltages){
//        MotorPowers powers = new MotorPowers();
//        double max = Math.abs(inputVoltages[maxIndex(inputVoltages)]);
//        if (max > batteryVoltage){
//            for (int i = 0; i < 4; i++){
//                inputVoltages[i] *= batteryVoltage / max;
//            }
//        }
//        powers.lf = inputVoltages[0] / batteryVoltage;
//        powers.rf = inputVoltages[1] / batteryVoltage;
//        powers.lb = inputVoltages[2] / batteryVoltage;
//        powers.rb = inputVoltages[3] / batteryVoltage;
//        return powers;
//    }
//    private double[] calculateInputVoltages(double maxAcel, int motor, double[] RPM, double[] targetRPM, double[] diff, int recursionCycles){
//        double[] voltages = new double[4];
//        double denominator = diff[motor];
//        denominator = Math.abs(denominator);
//        if (denominator < 1e-5){
//            denominator = 1e-5;
//        }
//        for (int i = 0; i < 4; i ++){
//            if (i != motor) {
//                double motorAcel = maxAcel * diff[i] / denominator;
//                double voltage = calculateMotorVoltage(motorAcel, RPM[i], i);
//                if (voltage > batteryVoltage + 1e-3 || voltage < -batteryVoltage - 1e-3){
//                    if (recursionCycles <= 2) {
//                        recursionCycles ++;
//                        boolean isAccelerating = (RPM[i] > 0 && targetRPM[i] > RPM[i]) || (RPM[i] < 0 && targetRPM[i] < RPM[i]);
//                        maxAcel = maxRPMAcel(RPM[i], targetRPM[i], i, isAccelerating);
//                        return calculateInputVoltages(maxAcel, i, RPM, targetRPM, diff, recursionCycles);
//                    }
//                }
//                voltages[i] = voltage;
//            }
//        }
//        if (diff[motor] < 0){
//            maxAcel = -maxAcel;
//        }
//        voltages[motor] = calculateMotorVoltage(maxAcel, RPM[motor], motor);
//        return voltages;
//    }
//
//    private double calculateMotorVoltage(double acel, double RPM, int motor){
//        int sign = 1;
//        if (acel < 0){
//            sign = -1;
//        }
//        double a = acel * (ENERGY_TOTAL[motor] * Math.PI / 30 / MOTOR_TORQUE_CONSTANT[motor] / GEAR_RATIO[motor]);
//        double b = RPM / MOTOR_VELOCITY_CONSTANT[motor];
//        double c = sign * FRICTIONAL_TORQUE[motor] / MOTOR_TORQUE_CONSTANT[motor] / GEAR_RATIO[motor];
//        double voltage = a + b + c;
//        return voltage;
//    }
//    private double maxRPMAcel(double RPM, double targetRPM, int motor, boolean isAcelerating){
//        int sign = 1;
//        if (!isAcelerating){
//            sign = -1;
//        }
//        RPM = Math.abs(RPM);
//        double numerator = batteryVoltage - (RPM) / MOTOR_TORQUE_CONSTANT[motor] - sign * FRICTIONAL_TORQUE[motor] / MOTOR_TORQUE_CONSTANT[motor] / GEAR_RATIO[motor];
//        double denominator = TOTAL_ROTAIONAL_INIRTIA[motor] * RADIAN_TO_RPS / MOTOR_TORQUE_CONSTANT[motor] / GEAR_RATIO[motor];
//        double acel = numerator/denominator;
//        double velocity;
//        if (motor == 0){
//            velocity = lf.getVelocity();
//        } else if (motor == 1){
//            velocity = rf.getVelocity();
//        } else if (motor == 2){
//            velocity = lb.getVelocity();
//        } else {
//            velocity = rb.getVelocity();
//        }
//        velocity = velocity / TICKS * SECONDS_PER_MINUTE;
//        if (acel > 0){
//            if (acel * frameTime / FRACTION_UNDER_SHOOT + velocity > targetRPM){
//                acel = (targetRPM - velocity) * FRACTION_UNDER_SHOOT / frameTime;
//            }
//        }
//        else if (acel < 0){
//            if (acel * frameTime / FRACTION_UNDER_SHOOT + velocity < targetRPM){
//                acel = (targetRPM - velocity) * FRACTION_UNDER_SHOOT / frameTime;
//            }
//        }
//        return acel;
//    }
//    private int maxIndex(double[] a){
//        double max = Math.abs(a[0]);
//        int index = 0;
//        for(int i = 1; i < a.length; i++){
//            double c = Math.abs(a[i]);
//            if (c > max){
//                index = i;
//                max = c;
//            }
//        }
//        return index;
//    }
//}
