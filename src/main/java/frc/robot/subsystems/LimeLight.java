// uncomment when used 

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private boolean targetFound;

    public LimeLight() {
        camera = new PhotonCamera("LimeLight");
        targetFound = false;
    }

    @Override
    public void periodic() {
        runCamera();
    }

    /**
     * Using PhotonVision to get the estimated pose of the Robot
     * 
     * @return Pose2d of the robot
     */

    public void runCamera() {
        var results = camera.getLatestResult();
        targetFound = results.hasTargets();

        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();
            int targetID = target.getFiducialId();
            Transform3d targetPose =  target.getBestCameraToTarget();
            double poseAmbiguity = target.getPoseAmbiguity();
            SmartDashboard.putNumber("tagetID", targetID);
            SmartDashboard.putNumber("taget Y", targetPose.getY());
            SmartDashboard.putNumber("taget X", targetPose.getX());
            SmartDashboard.putNumber("taget Z", targetPose.getZ());
            SmartDashboard.putNumber("taget Pitch", targetPose.getRotation().getAngle());

            PhotonUtils.calculateDistanceToTargetMeters(6, targetID, poseAmbiguity, targetID);
            
        }
        SmartDashboard.putBoolean("targetFound", targetFound);
    }

    public void getAprilTags() {

    }

    public void getReflectiveTape() {
        camera.setLED(VisionLEDMode.kOn);
        camera.setPipelineIndex(0);
    }

    /**
     * Get the Distance from the Limelight to the Refelctive tape on the Poles
     * 
     * @return Double of the distance from the Limelight to the Refelctive tape on
     *         the Pole
     */
    public double refecltiveTapeDist() {
        return 0;
    }

    /**
     * Get the Angle from the Limelight to the Refelctive tape on the Poles
     * 
     * @return Duoble of the angle from the Limelight to the Refelctive tape on the
     *         P
     */
    public double refecltiveTapeAngle() {
        return 0;
    }

    /**
     * Get the Distance from the Limelight to the April tags
     * 
     * @return DOuble of the distance from the Limelight to the April Tags
     */
    public double apriltagDist() {
        return 0;
    }

    /**
     * Get the Angle from the Limelight to the April tags
     * 
     * @return double of the angle from the Limelight to the April Tags
     */
    public double apriltagAngle() {
        return 0;
    }

}
