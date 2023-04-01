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

public class LimeLight1 extends SubsystemBase {

    public boolean F_targetFound, B_targetFound;
    public double F_x, F_y, F_a, F_v, B_x, B_y, B_a, B_v;
    public SwerveDrivePoseEstimator est;
    public NetworkTable Front_Cam, Back_Cam;
    public NetworkTableEntry F_ty, F_tx, F_ta, F_pn, F_led, F_tv, F_mode;
    public NetworkTableEntry B_ty, B_tx, B_ta, B_pn, B_led, B_tv, B_mode;
    public double side;
    public ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public LimeLight1() {

        Front_Cam = NetworkTableInstance.getDefault().getTable("limelight-fcam");
        Back_Cam = NetworkTableInstance.getDefault().getTable("limelight-bcam");
        try {
            autoTab.addCamera("Front LimeLight", "limelight-fcam", "http://10.40.69.39:5800");
            autoTab.addCamera("Back LimeLight", "limelight-bcam", "http://10.40.69.70:5801");
        } catch (Exception e) {
            System.out.println("Error in camera Streaming, or already streaming");
        }

    }

    @Override
    public void periodic() {
        side = RobotContainer.swerveSubsystem.getSide();
        F_mode.setNumber(0);
        B_mode.setNumber(0);
        updateCam();
        printCameras();
    }

    /**
     * Using PhotonVision to get the estimated pose of the Robot
     * 
     * @return Pose2d of the robot
     */

    public void updateCam() {

        F_pn = Front_Cam.getEntry("pipeline");
        F_led = Front_Cam.getEntry("ledMode");
        F_mode = Front_Cam.getEntry("camMode");
        F_tv = Front_Cam.getEntry("tv");
        F_tx = Front_Cam.getEntry("tx");
        F_ty = Front_Cam.getEntry("ty");
        F_ta = Front_Cam.getEntry("ta");

        F_x = F_tx.getDouble(0.0);
        F_y = F_ty.getDouble(0.0);
        F_a = F_ta.getDouble(0.0);
        F_v = F_tv.getDouble(0.0);

        B_pn = Back_Cam.getEntry("pipeline");
        B_led = Back_Cam.getEntry("ledMode");
        B_mode = Back_Cam.getEntry("camMode");
        B_tv = Back_Cam.getEntry("tv");
        B_tx = Back_Cam.getEntry("tx");
        B_ty = Back_Cam.getEntry("ty");
        B_ta = Back_Cam.getEntry("ta");

        B_x = B_tx.getDouble(0.0);
        B_y = B_ty.getDouble(0.0);
        B_a = B_ta.getDouble(0.0);
        B_v = B_tv.getDouble(0.0);

    }

    public void odometryUpdater() {
        if (side == -1 && (F_pn.getNumber(0).intValue() == 1)) {
            est.addVisionMeasurement(new Pose2d( ), Timer.getFPGATimestamp());

        }
        else if (side == 1 && (B_pn.getNumber(0).intValue() == 1)) {
            est.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp() );
        }

    }

    public void printCameras() {
        SmartDashboard.putNumber("Front LimelightX", F_x);
        SmartDashboard.putNumber("Front LimelightY", F_y);
        SmartDashboard.putNumber("Front LimelightArea", F_a);

        SmartDashboard.putNumber("Back LimelightX", B_x);
        SmartDashboard.putNumber("Back LimelightY", B_y);
        SmartDashboard.putNumber("Back LimelightArea", B_a);
    }

}
