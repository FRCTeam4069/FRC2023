package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    
    public Pigeon2 gyro;
    
    public Gyro(int id){
        gyro = new Pigeon2(id);
    }

    public void resetGyro(){
        gyro.setYaw(0);

    }

    /**
     * 
     * @return Accumalted Yaw
     */
    public double getYaw(){
        return gyro.getYaw();
    }

    /** 
     * @return Current heading 
     */
    public double getHeading(){
        return gyro.getYaw() % 360;
    }

    public void setYaw(double Yaw){
        gyro.setYaw(Yaw);
    }

    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll(){
        return gyro.getRoll();
    }
    //Refer to CTRE docs for Piegon 2 to understand Pitch, and Roll

    @Override
    public void periodic(){
        SmartDashboard.putNumber("GYRO PITCH", getPitch());
        SmartDashboard.putNumber("GYRO YAW", getYaw());
        SmartDashboard.putNumber("GYRO ROLL", getRoll());
        SmartDashboard.putNumber("GYRO HEADING", getHeading());
    }
}