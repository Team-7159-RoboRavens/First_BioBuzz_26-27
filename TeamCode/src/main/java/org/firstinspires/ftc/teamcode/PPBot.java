package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PPBot extends BarebonesRobot{
    GoBildaPinpointDriver pinpoint;
    VoltageSensor voltageSensor;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    //TODO: try to use the values defined from Constants.java, instead of creating two sources of the same values
    private final int startPinpointX = -110;
    private final int startPinpointY = 105;

    public boolean inputHandlingEnabled = false;
    
    public PPBot(HardwareMap hmap) {
        super(hmap);
        hardwareMap = hmap;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(startPinpointX, startPinpointY, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    public double pinpointAngle() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
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

    // Must be called once per loop before reading any Pinpoint position/heading data.
    public void pinpointUpdate(){
        pinpoint.update();
    }
}
