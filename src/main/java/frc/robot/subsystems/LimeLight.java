package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    
    public PhotonCamera camera;
    public PhotonPoseEstimator estimator;
  
    
    public LimeLight(){
        //camera = new PhotonCamera("LimeLight");
    }

    public Pose2d poseEstimation(){
        return new Pose2d(0,0, new Rotation2d(0));

    }

    /**
     * Get the Distance from the Limelight to the Refelctive tape on the Poles
     * @return
     */
    public double refecltiveTapeDist(){
        return 0;
    }

    /**
     * Get the Angle from the Limelight to the Refelctive tape on the Poles
     * @return
     */
    public double refecltiveTapeAngle(){
        return 0;
    }

    /**
     * Get the Distance from the Limelight to the Refelctive tape on the Poles
     * @return
     */
    public double apriltagDist(){
        return 0;
    }
    
    /**
     * Get the Angle from the Limelight to the Refelctive tape on the Poles
     * @return
     */
    public double apriltagAngle(){
        return 0;
    }

   
}