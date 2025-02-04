// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstans;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

  private DutyCycleEncoder m_elevatorEncoder;
  private final SparkMax m_ElevatorSparkMax = new SparkMax(12, MotorType.kBrushless);
  private double m_lastVoltage = 0;

  public void RunElevatorUp() {
    m_lastVoltage = ElevatorConstants.m_ElevatorMotorspeed;
    m_ElevatorSparkMax.setVoltage(m_lastVoltage);
   
   
  }

  public void RunElevatorDown() {
    m_lastVoltage = ElevatorConstants.m_ElevatorMotorspeed;
    m_ElevatorSparkMax.setVoltage(m_lastVoltage);
  }

  public void RunElevatorStop() {
    m_lastVoltage = 0;
    m_ElevatorSparkMax.setVoltage(m_lastVoltage);
    
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
   

  }

  @Override
  public void periodic() {

    double outputCurrent = m_ElevatorSparkMax.getOutputCurrent();
   // System.out.println("Motor tarafından çekilen akım: " + outputCurrent + " amper");
   // System.out.println(" Encoder aaaaa" + m_elevatorEncoder.get() + " ");

  }

  public Command RunElevatorUpCommand() {
    return run(() -> RunElevatorUp());
  }

  public Command RunElevatorUpOnceCommand() {
    return run(() -> RunElevatorUp());
  }

  public Command RunElevatorDownCommand() {
    return run(() -> RunElevatorDown());
  }

  public Command RunElevatorDowntOnceCommand() {
    return run(() -> RunElevatorDown());
  }

  public Command RunElevatorStopCommand() {
    return run(() -> RunElevatorStop());
  }

  public Command RunElevatorStopOnceCommand() {
    return run(() -> RunElevatorStop());
  }
}
