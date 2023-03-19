package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

    public Pigeon2 gyro;

    public Gyro(int id) {
        gyro = new Pigeon2(id);
    }

    public void resetGyro() {
        gyro.setYaw(0);

    }

    /**
     * 
     * @return Accumalted Yaw
     */
    public double getYaw() {
        return gyro.getYaw();
    }

    /** 
     * @return Current heading 
     */
    public double getHeading(double heading){
        if(gyro.getYaw() % 360+heading < -180){
            return (360-(gyro.getYaw()+heading % 360));
        }else{
            return ((gyro.getYaw()+heading % 360));
        }
 
    }

    public void setYaw(double Yaw) {
        gyro.setYaw(Yaw);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void periodic() {
    }
}