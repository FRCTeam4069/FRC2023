// uncomment when used 


package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private boolean targetFound;
    
    

    
    public LimeLight(){
        //camera = new PhotonCamera("LimeLight");
        //targetFound = false;
    }
    /**
     * Using PhotonVision to get the estimated pose of the Robot
     * @return Pose2d of the robot
    */

    public void runCamera(){
        var results = camera.getLatestResult();
        targetFound = results.hasTargets();
        List<PhotonTrackedTarget> targets = results.getTargets();

        PhotonTrackedTarget target = results.getBestTarget();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        
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
