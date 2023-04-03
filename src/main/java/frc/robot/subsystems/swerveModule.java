// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.drivebaseConstants;
import frc.robot.Constants.drivebaseConstants.CharacterizationData;
import frc.robot.Constants.drivebaseConstants.ModuleConstants;

public class swerveModule extends SubsystemBase {
   private final TalonFX driveMotor;
   private final CANSparkMax turnMotor;
   private final PIDController tPidController;
   private final CANCoder tCanCoder;
   private final double absoluteEncoderOffsetRad;
   private final boolean absoluteEncoderReversed;
   private double output;

   /***
    * 
    * @param driveMotorId             Drive motor ID
    * @param turnMotorId              Turn motor ID
    * @param driveMotorReversed       Is drive motor reversed?
    * @param turnMotorReversed        Is turn motor reversed?
    * @param absoluteEncoderID        Absolute encoder ID
    * @param absoluteEncoderOffsetRad Absolute encoder offset in Rad
    * @param absoluteEncoderReversed  Is absolute encoder reversed?
    */
   public swerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
         int absoluteEncoderID, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {

      this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      tCanCoder = new CANCoder(absoluteEncoderID);

      driveMotor = new TalonFX(driveMotorId); // 2048 unit / rev
      turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      driveMotor.setNeutralMode(NeutralMode.Brake);

      tCanCoder.configSensorDirection(absoluteEncoderReversed);
      tCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, absoluteEncoderID);
      turnMotor.setInverted(turnMotorReversed);
      tCanCoder.configMagnetOffset(absoluteEncoderOffsetRad);

      tPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
      tPidController.enableContinuousInput(-Math.PI, Math.PI);
      resetEncoders();
   }

   public double getDrivePosition() {
      return (driveMotor.getSelectedSensorPosition() / (2048 * 6.75)) * Math.PI * drivebaseConstants.ModuleConstants.kWheelDiameterMeters;
   }

   public double getTurnignPosition() {
      return Math.toRadians(tCanCoder.getAbsolutePosition());
   }

   public double getDriveVelocity() {
      return (driveMotor.getSelectedSensorVelocity() / (2048 * 6.75)) * 10 * Math.PI * drivebaseConstants.ModuleConstants.kWheelDiameterMeters;
   }

   public double getTurningVelocity() {
      return Math.toRadians(tCanCoder.getVelocity());
   }

   public void resetEncoders() {
      driveMotor.setSelectedSensorPosition(0);
   }

   public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnignPosition()));
   }

   public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnignPosition()));

   }

   public void setDesiredState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.005) {
         stop();
         return;
      }
      state = SwerveModuleState.optimize(state, getState().angle);
      this.output = CharacterizationData.CurrentFFController.calculate(state.speedMetersPerSecond) / 12;
      driveMotor.set(TalonFXControlMode.PercentOutput,
            (CharacterizationData.CurrentFFController.calculate(state.speedMetersPerSecond) / 12));
      turnMotor.set(tPidController.calculate(getTurnignPosition(), state.angle.getRadians()));
   }

   public void setCurrentFFController(int controllerIndex){
      if (controllerIndex == 0){
         CharacterizationData.CurrentFFController = CharacterizationData.AutofeedForwardController;
         System.out.println("Set To Auto FF");
      } else if (controllerIndex == 1){
         CharacterizationData.CurrentFFController = CharacterizationData.TelefeedForwardController;
         System.out.println("Set To Tele FF");

      }
   }

   public double getDriveVoltage() {
      return this.output;
   }

   public void stop() {
      driveMotor.set(ControlMode.PercentOutput, 0);
      turnMotor.set(0);
   }

   public TalonFX getDriveMotor() {
      return driveMotor;
   }
   public double getTurningAngle(){
      return tCanCoder.getAbsolutePosition();
  }


   public void printNumbers(){
      SmartDashboard.putNumber("Turning Angle", tCanCoder.getAbsolutePosition());
   }
}
