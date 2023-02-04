
package frc.lib.vision;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Handles receiving data from a LimeLight.
 * <p>
 * This class is used for receiving data from the LimeLight over Network Tables.
 * Additionally, it contains methods for setting various settings on the
 * LimeLight and getting the distance to the target.
 */
public class LimeLight {

    // LimeLight Network Table
    NetworkTable limelightTable;

    // LimeLight Network Table Entries (Get Values)
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry ts;
    NetworkTableEntry tl;
    NetworkTableEntry getpipe;
    NetworkTableEntry limelightJson;

    NetworkTableEntry botpose;
    NetworkTableEntry botpose_wpiblue;
    NetworkTableEntry botpose_wpired;
    NetworkTableEntry camerapose_targetspace;
    NetworkTableEntry targetpose_cameraspace;
    NetworkTableEntry targetpose_robotspace;
    NetworkTableEntry botpose_targetspace;
    NetworkTableEntry tID;

    // LimeLight Network Table Entries (Set Values)
    NetworkTableEntry ledMode;
    NetworkTableEntry camMode;
    NetworkTableEntry pipeline;
    NetworkTableEntry stream;
    NetworkTableEntry snapshot;

    MedianFilter xFilter = new MedianFilter(5);
    MedianFilter yFilter = new MedianFilter(5);

    double xMedian = 0;
    double yMedian = 0;

    public enum LedMode {
        /**
         * LEDs Off
         */
        Off,
        /**
         * Use the LED Mode set in the current pipeline.
         */
        Pipeline,
        /**
         * LEDs On
         */
        On,
        /**
         * Blink LEDs
         */
        Blink
    }

    public enum CameraMode {
        /**
         * Driver Camera (increases exposure and disables vision processing)
         */
        Driver,
        /**
         * Vision Processor
         */
        Vision
    }

    public enum StreamMode {
        /**
         * Side-by-side streams if a webcam is attached to LimeLight.
         */
        Standard,
        /**
         * The secondary camera stream is placed in the lower-right corner of the
         * primary camera stream.
         */
        PiPMain,
        /**
         * The primary camera stream is placed in the lower-right corner of the
         * secondary camera stream.
         */
        PiPSecondary
    }

    public enum PipelineMode {
        // April Tags
        AprilTag,
        // Retroreflective
        Retroreflective
    }

    public enum CoordinateSpace{
        Field,
        Blue,
        Red,
        Target,
        Camera
    }

    /**
     * Uses the LimeLight's default network table name ("limelight").
     */
        /*  ---only use if there is only 1 limelight
    public LimeLight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        getTableEntries();
    }
        */

        
    /**
     * @param networkTableName the name of the LimeLight's network table.
     */
    public LimeLight(String networkTableName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(networkTableName);
        getTableEntries();
    }

    private void getTableEntries() {
        // LimeLight Network Table Entries (Get Values)
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        tl = limelightTable.getEntry("tl");
        getpipe = limelightTable.getEntry("getpipe");
        limelightJson = limelightTable.getEntry("json");

        botpose = limelightTable.getEntry("botpose");
        botpose_wpiblue = limelightTable.getEntry("botpose_wpiblue");
        botpose_wpired = limelightTable.getEntry("botpose_wpired");
        camerapose_targetspace = limelightTable.getEntry("camerapose_targetspace");
        targetpose_cameraspace = limelightTable.getEntry("targetpose_cameraspace");
        targetpose_robotspace = limelightTable.getEntry("targetpose_robotspace");
        botpose_targetspace = limelightTable.getEntry("botpose_targetspace");
        
        tID = limelightTable.getEntry("tid");

        // LimeLight Network Table Entries (Set Values)
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        stream = limelightTable.getEntry("stream");
        snapshot = limelightTable.getEntry("snapshot");


    }

    /**
     * @return true if LimeLight has a target.
     */
    public boolean hasTarget() {
        if (tv.getDouble(0) < 1) {
            return false;
        } else {
            return true;
        }
    }

    public void resetMedianFilters() {
        xFilter.reset();
        yFilter.reset();
    }

    /**
     * @return the X offset value from the crosshair to the target in degrees.
     */
    public double getTargetXOffset() {
        xMedian = xFilter.calculate(tx.getDouble(0));
        return xMedian;
        // return tx.getDouble(0);
    }

    /**
     * @return the Y offset value from the crosshair to the target in degrees.
     */
    public double getTargetYOffset() {
        yMedian = yFilter.calculate(ty.getDouble(0));
        return yMedian;
    }

    /**
     * @return the area of the target from 0% of the image to 100% of the image.
     */
    public double getTargetArea() {
        return ta.getDouble(0);
    }

    /**
     * @return the skew or rotation of the target from -90 degrees to 0 degrees.
     */
    public double getTargetRotation() {
        return ts.getDouble(0);
    }

    /**
     * You should add at least 11ms for image capture latency.
     * 
     * @return the pipeline's latency contribution in milliseconds.
     */
    public double getPipelineLatency() {
        return tl.getDouble(0);
    }

    /**
     * @return the active pipeline index of the camera.
     */
    public double getActivePipeline() {
        return getpipe.getDouble(0);
    }

    public String getLimelightJson(){
        return limelightJson.getString("No Data");
    }

    public long getTargetID() {
        if (this.hasTarget()){
            return tID.getInteger(-1L);
        } else{
            return -1L;
        }       
    }

    public double[] getBotPose(CoordinateSpace space){
         switch (space) {
            case Field:
                return botpose.getDoubleArray(new double[6]);
            case Blue:
                return botpose_wpiblue.getDoubleArray(new double[6]);
            case Red:
                return botpose_wpired.getDoubleArray(new double[6]);
            case Target:
                return botpose_targetspace.getDoubleArray(new double[6]);
            case Camera:
            //TODO: should return robot center offset from camera lens center
                return new double[6];
            default:
                return null;
        }

    }

    /**
     * @param mode the desired set LED mode.
     */
    public void setLedMode(LedMode mode) {
        switch (mode) {
            case Off:
                ledMode.setDouble(1);
                break;
            case Pipeline:
                ledMode.setDouble(0);
                break;
            case On:
                ledMode.setDouble(3);
                break;
            case Blink:
                ledMode.setDouble(2);
                break;
        }
    }

    /**
     * @param mode the desired set camera mode.
     */
    public void setCameraMode(CameraMode mode) {
        switch (mode) {
            case Driver:
                camMode.setDouble(1);
                break;
            case Vision:
                camMode.setDouble(0);
                break;
        }
    }

    /**
     * @param pipelineNumber select pipeline number from 0 to 9.
     */
    public void setActivePipline(int pipelineNumber) {
        if (pipelineNumber >= 0 && pipelineNumber <= 9) {
            pipeline.setDouble(pipelineNumber);
        }
    }

    /**
     * @param mode the desired set stream mode.
     */
    public void setStreamMode(StreamMode mode) {
        switch (mode) {
            case Standard:
                stream.setDouble(0);
                break;
            case PiPMain:
                stream.setDouble(1);
                break;
            case PiPSecondary:
                stream.setDouble(2);
                break;
        }
    }

    /**
     * @param toggleSnapshots take two snapshots per second.
     */
    public void setTakeSnapshots(boolean toggleSnapshots) {
        if (toggleSnapshots) {
            snapshot.setDouble(1);
        } else {
            snapshot.setDouble(0);
        }
    }

    /**
     * @param cameraAngle  the mounting angle of the camera.
     * @param cameraHeight the height of the camera from the floor.
     * @param targetHeight the height of the target from the floor.
     * @return the distance from the camera to the target.
     */
    public double getDistanceToTarget(double cameraAngle, double cameraHeight, double targetHeight) {
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + getTargetYOffset()));
    }

    public double getCameraAngle(double distance, double targetHeight, double cameraHeight) {
        return (Math.toDegrees(Math.atan((targetHeight - cameraHeight) / distance)) + getTargetYOffset());
    }

    public void setPipelineMode(PipelineMode pipeline) {
        switch (pipeline) {
            case AprilTag:
                setActivePipline(1);
                break;
            case Retroreflective:
                setActivePipline(0);
                break;
        }
    }
}