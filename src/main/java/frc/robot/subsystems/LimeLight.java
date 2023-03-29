// uncomment when used 

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class 
LimeLight extends SubsystemBase {

    public PhotonCamera F_camera, B_camera;
    private PhotonPoseEstimator B_estimator, F_estimator;
    public boolean F_targetFound, B_targetFound;
    public double F_pitch, B_pitch, F_area, B_area;
    public  SwerveDrivePoseEstimator est;

    public LimeLight() {

        
        F_camera = new PhotonCamera("LimeLightONE");
        B_camera = new PhotonCamera("LimeLightTWO");
        
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            B_estimator = new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, B_camera,
                    new Transform3d(new Translation3d(Units.inchesToMeters(-6), Units.inchesToMeters(-7.1),
                            Units.inchesToMeters(15)), new Rotation3d(0, 0, Units.degreesToRadians(-80))));
            B_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            B_estimator = null;
        }

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            F_estimator = new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, F_camera,
                    new Transform3d(new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(7.1),
                            Units.inchesToMeters(15)), new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(80)))

            );
            B_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            F_estimator = null;
        }
        
        F_camera.setPipelineIndex(2);
        F_camera.setLED(VisionLEDMode.kOff);

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
        if (F_camera.getLatestResult().hasTargets()) {
            PhotonTrackedTarget F_target = F_camera.getLatestResult().getBestTarget();
            if(F_target != null){
            F_pitch = F_target.getPitch(); 
            F_area = F_target.getArea(); 

            F_estimator.update(F_camera.getLatestResult());
            F_estimator.setReferencePose(RobotContainer.swerveSubsystem.getPose());
            F_estimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP);

        }}
        
        if (B_camera.getLatestResult().hasTargets()) {
            PhotonTrackedTarget B_target = B_camera.getLatestResult().getBestTarget();
            if(B_target != null){
            B_pitch = B_target.getPitch(); 
            B_area = B_target.getArea(); 
        }}
        
        
    }

    public void setToAprilTags() {

    }

    public void setToReflectiveTape() {
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


}
