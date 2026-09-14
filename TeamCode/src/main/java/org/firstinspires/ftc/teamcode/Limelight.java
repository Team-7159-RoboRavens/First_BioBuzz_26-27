package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

// Wraps a Limelight3A vision sensor, exposing pipeline switching and simple
// getters for AprilTag (fiducial) and color-blob results.
public class Limelight {
    public static final int PIPELINE_COLOR = 0;
    public static final int PIPELINE_APRILTAG = 1;

    private final Limelight3A limelight;

    private LLResult latestResult;
    private List<LLResultTypes.FiducialResult> fiducialResults;
    private List<LLResultTypes.ColorResult> colorResults;

    /**
     * @param robot robot used to look up the Limelight3A device from its hardware map
     *              (device must be configured with the name "limelight")
     */
    public Limelight(Robot robot) {
        limelight = robot.getHardwareMap().get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(PIPELINE_COLOR);
        limelight.start();
    }

    // Must be called once per loop before reading any AprilTag/color-blob data,
    // so the cached results reflect the latest poll from the Limelight.
    public void update() {
        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            fiducialResults = latestResult.getFiducialResults();
            colorResults = latestResult.getColorResults();
        } else {
            fiducialResults = null;
            colorResults = null;
        }
    }

    // Switches to the color-blob detection pipeline.
    public void useColorPipeline() {
        limelight.pipelineSwitch(PIPELINE_COLOR);
    }

    // Switches to the AprilTag detection pipeline.
    public void useAprilTagPipeline() {
        limelight.pipelineSwitch(PIPELINE_APRILTAG);
    }

    /**
     * @param index pipeline index to switch to
     */
    public void switchPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /**
     * @return index of the pipeline currently active on the Limelight
     */
    public int getPipelineIndex() {
        return limelight.getStatus().getPipelineIndex();
    }

    /**
     * @return true if the latest result exists and contains valid target data
     */
    public boolean hasValidResult() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * @return horizontal angle offset from the crosshair to the primary target, in degrees
     */
    public double getTargetXOffset() {
        return hasValidResult() ? latestResult.getTx() : 0.0;
    }

    /**
     * @return vertical angle offset from the crosshair to the primary target, in degrees
     */
    public double getTargetYOffset() {
        return hasValidResult() ? latestResult.getTy() : 0.0;
    }

    /**
     * @return id of the first detected AprilTag, or -1 if none is visible
     */
    public int getAprilTagId() {
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }
        return fiducialResults.get(0).getFiducialId();
    }

    /**
     * @return straight-line distance from the camera to the first detected AprilTag, in inches,
     *         or -1 if none is visible
     */
    public double getAprilTagDistance3D() {
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }
        Position pos = fiducialResults.get(0).getTargetPoseCameraSpace().getPosition();
        return Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
    }
    public double getAprilTagDistance2D() {
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }
        Position pos = fiducialResults.get(0).getTargetPoseCameraSpace().getPosition();
        return Math.sqrt(pos.x * pos.x + pos.y * pos.y);
    }


    /**
     * @return horizontal angle offset to the first detected AprilTag, in degrees,
     *         or 0 if none is visible
     */
    public double getAprilTagAngleXOffset() {
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return 0.0;
        }
        return fiducialResults.get(0).getTargetXDegrees();
    }
    public double getAprilTagAngleYOffset() {
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return 0.0;
        }
        return fiducialResults.get(0).getTargetYDegrees();
    }

    /**
     * @return true if at least one color blob is currently visible
     */
    public boolean hasColorBlob() {
        return colorResults != null && !colorResults.isEmpty();
    }

    /**
     * @return horizontal angle offset to the first detected color blob, in degrees,
     *         or 0 if none is visible
     */
    public double getColorBlobXOffset(int id) {
        if (colorResults == null || colorResults.isEmpty()) {
            return 0.0;
        }
        return colorResults.get(id).getTargetXDegrees();
    }

    /**
     * @return vertical angle offset to the first detected color blob, in degrees,
     *         or 0 if none is visible
     */
    public double getColorBlobYOffset(int id) {
        if (colorResults == null || colorResults.isEmpty()) {
            return 0.0;
        }
        return colorResults.get(id).getTargetYDegrees();
    }
    public double getColorBlobXSize(int id) {
        if (colorResults == null || colorResults.isEmpty()) {
            return 0.0;
        }
        return colorResults.get(id).getTargetXPixels();
    }
    public double getColorBlobYSize(int id) {
        if (colorResults == null || colorResults.isEmpty()) {
            return 0.0;
        }
        return colorResults.get(id).getTargetYPixels();
    }

    /**
     * @return number of color blobs currently detected
     */
    public int getColorBlobCount() {
        return colorResults == null ? 0 : colorResults.size();
    }

    // Stops polling the Limelight for new results. Call when the sensor is no longer needed.
    public void stop() {
        limelight.stop();
    }
}
