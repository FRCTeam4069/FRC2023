// uncomment when used 

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.drivebaseConstants;

public class cameraHelper extends SubsystemBase {
    private String name;
    private NetworkTable Cam;
    private NetworkTableEntry ty, tx, ta, tv, pn, led, mode;
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public cameraHelper(String CameraName, String CameraStreamURL, String networkTableKey) {
        name = CameraName;

        Cam = NetworkTableInstance.getDefault().getTable(networkTableKey);
        try {
            autoTab.addCamera(CameraName, networkTableKey, CameraStreamURL);
        } catch (Exception e) {
            System.out.println("Error in camera Streaming, or already streaming");
        }

        pn = Cam.getEntry("pipeline");
        led = Cam.getEntry("ledMode");
        mode = Cam.getEntry("camMode");
        tv = Cam.getEntry("tv");
        tx = Cam.getEntry("tx");
        ty = Cam.getEntry("ty");
        ta = Cam.getEntry("ta");

    }

    /**
     * @return X value of the target
     */
    public double tx() {
        return tx.getDouble(0);
    }

    /**
     * @return Y value of the target
     */
    public double ty() {
        return ty.getDouble(0);
    }

    /**
     * @return The area of the target
     */
    public double ta() {
        return ta.getDouble(0);
    }

    /**
     * @returns 0 if no target is found
     * @returns 1 if target is found
     */
    public double tv() {
        return tv.getDouble(0);
    }

    public double getPipeline() {
        return pn.getDouble(0);
    }

    public double getMode() {
        return mode.getDouble(0);
    }

    public double getLed() {
        return led.getDouble(0);
    }

    public void setMode(double mode) {
        this.mode.setDouble(mode);
    }

    public void setLED(double mode) {
        this.led.setDouble(mode);
    }

    public void setPipeline(double mode) {
        this.pn.setDouble(mode);
    }

    public void printerNumbers(){
        SmartDashboard.putNumber(name+"tv", tv());
        SmartDashboard.putNumber(name+"tx", tx());
        SmartDashboard.putNumber(name+"ty", ty());
        SmartDashboard.putNumber(name+"ta", ta());
    }

    
}
             