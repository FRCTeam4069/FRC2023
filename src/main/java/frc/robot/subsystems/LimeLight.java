package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private boolean targetFound;
  
    
    public LimeLight(){
        camera = new PhotonCamera("LimeLight");
        targetFound = false;
        
    }
    /**
     * Using PhotonVision to get the estimated pose of the Robot
     * @return Pose2d of the robot
    */

    public void runCamera(){
        
    }

    public Pose2d poseEstimation(){
        return new Pose2d(0,0, new Rotation2d(0));

    }

    /**
     * Get the Distance from the Limelight to the Refelctive tape on the Poles
     * @return Double of the distance from the Limelight to the Refelctive tape on the Pole
     */
    public double refecltiveTapeDist(){
        return 0;
    }

    /**
     * Get the Angle from the Limelight to the Refelctive tape on the Poles
     * @return Duoble of the angle from the Limelight to the Refelctive tape on the P
     */
    public double refecltiveTapeAngle(){
        return 0;
    }

    /**
     * Get the Distance from the Limelight to the April tags
     * @return DOuble of the distance from the Limelight to the April Tags
     */
    public double apriltagDist(){
        return 0;
    }
    
    /**
     * Get the Angle from the Limelight to the April tags
     * @return double of the angle from the Limelight to the April Tags
     */
    public double apriltagAngle(){
        return 0;
    }

   
}