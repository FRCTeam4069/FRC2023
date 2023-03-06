package frc.robot.Auto.Commands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.armSubsystem;

public class FieldOrientedMTP extends CommandBase {
    private Supplier<Double> side;
    private static final armSubsystem arm = RobotContainer.arm;
    private static final Intake intake = RobotContainer.intake;
    private double[] articulatePositions = {125
        , 45, 46, 31};
    private double[] wristPoses = {1.05, 0, 0, -0.45};
    
    private final Supplier<Integer> POV;
    private int AorB;
    private int poseNum;

    public FieldOrientedMTP(int side, Supplier<Double> robotSide, Supplier<Integer> POV) {
        this.side = robotSide;
        this.POV = POV;
        this.AorB = side;
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute() {
        switch(POV.get()){
            case 90: 
            poseNum = 3;
            break; 
            case 180: 
            poseNum = 0;
            break;
            case 270: 
            poseNum = 1;
            break;
            case 0:
            poseNum = 2; 
            break; 
            default:
            poseNum = -1;
            
        }
        if(poseNum != -1){
        arm.setArmPose(articulatePositions[poseNum] * AorB * side.get());
        //arm.extendToPose(extendToPose[poseNum], 1);
        intake.wristToPose(AorB * wristPoses[poseNum]);

        
        SmartDashboard.putNumber("SIDE", side.get());
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
       return false;
    }

}

