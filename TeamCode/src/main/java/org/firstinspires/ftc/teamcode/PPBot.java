package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PPBot extends BarebonesRobot{
    VoltageSensor voltageSensor;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    //TODO: try to use the values defined from Constants.java, instead of creating two sources of the same values
    private static final int START_PINPOINT_X = -110;
    private static final int START_PINPOINT_Y = 105;

    public boolean inputHandlingEnabled = false;
    
    public PPBot(HardwareMap hmap) {
        // BarebonesRobot sets up and resets the Pinpoint once with these offsets
        super(hmap, START_PINPOINT_X, START_PINPOINT_Y);
        hardwareMap = hmap;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
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

}
