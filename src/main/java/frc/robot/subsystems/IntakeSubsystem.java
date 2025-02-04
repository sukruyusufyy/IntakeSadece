// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstans;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax m_IntakeSparkMax1 = new SparkMax(IntakeConstans.m_IntakeMotor1Id, MotorType.kBrushless);
  private final SparkMax m_IntakeSparkMax2 = new SparkMax(IntakeConstans.m_IntakeMotor2Id, MotorType.kBrushless);
private final SparkClosedLoopController  closedLoopController = m_IntakeSparkMax1.getClosedLoopController();
 private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final AbsoluteEncoder m_boreencoder;
  private double m_lastVoltage = 0;
private final PIDController pid = new PIDController(1, 0, 0);

  public void RunIntakeIn(  ) { // ıntake motor gücü (+ yönlü)
    if(m_IntakeSparkMax1.getOutputCurrent()<5)
    {
      m_lastVoltage = IntakeConstans.m_IntakeMotor1speed;
      m_IntakeSparkMax1.setVoltage(m_lastVoltage);
    }
    else{
     m_lastVoltage =0;
    }
  
    // System.out.println("Deneme");
  }

  public void RunIntakeOut() { // ıntake motor gücü (-yön)
    
    m_lastVoltage = IntakeConstans.m_IntakeMotor1speed;
    m_IntakeSparkMax1.setVoltage(-m_lastVoltage);
  }

  public void RunIntakeStop() {
    m_lastVoltage = 0;
    m_IntakeSparkMax1.setVoltage(m_lastVoltage);

  }
public void runIntakefifteenPID(){
 
   m_IntakeSparkMax1.setVoltage( MathUtil.clamp(-pid.calculate(m_boreencoder.getPosition(), 42), -5, 5));
  System.out.println("PID");
  //closedLoopController.setReference(50, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
public void runIntaketenPID(){
 
  m_IntakeSparkMax1.setVoltage( MathUtil.clamp(-pid.calculate(m_boreencoder.getPosition(), 55), -5, 5));
 System.out.println("PID");
 //closedLoopController.setReference(50, ControlType.kPosition, ClosedLoopSlot.kSlot0);
}
  /*-----------------2.SparkMax------------------- */

  public void TakeObject() {

    m_lastVoltage = IntakeConstans.m_IntakeMotor2speed;
    m_IntakeSparkMax2.setVoltage(m_lastVoltage);
  }

  public void ReverseObject() {
    m_lastVoltage = IntakeConstans.m_IntakeMotor2speed;
    m_IntakeSparkMax2.setVoltage(-m_lastVoltage);
  }

  public void ObjectStop() {
    m_lastVoltage = 0;
    m_IntakeSparkMax2.setVoltage(m_lastVoltage);
    
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
   
     motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
      
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_boreencoder = m_IntakeSparkMax1.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
    m_IntakeSparkMax1.setVoltage(0);
    SmartDashboard.putNumber("Intake Amper Value  ", m_IntakeSparkMax1.getOutputCurrent());
   SmartDashboard.putNumber("Intake Open Bore", m_boreencoder.getPosition());
   SmartDashboard.putNumber("Velocity",  MathUtil.clamp(pid.calculate(m_boreencoder.getPosition(), 16), -5, 5));
    double outputCurrent = m_IntakeSparkMax1.getOutputCurrent();
   
    // System.out.println("Motor tarafından çekilen akım: " + outputCurrent + "
    // amper");
    // Syst50.out.println("Motor tarafından çekilen akım: " + outputCurrent + "
    // amper");
    // m_ımtakeencoder neonun ıcındekı encoder
    // mborenecoder bore encoder
    // System.out.println("pozisyon : "+ m_IntakeEncoder.getPosition());
    // return;

  }

  public Command runIntakeInCommand() {
    return run(() -> RunIntakeIn());
  }

  public Command runIntakeInOnceCommand() {
    return run(() -> RunIntakeIn());
  }

  public Command runIntakeOutCommand() {
    return run(() -> RunIntakeOut());
  }

  public Command runIntakeOutOnceCommand() {
    return run(() -> RunIntakeOut());
  }

  public Command runIntakeStopCommand() {
    return run(() -> RunIntakeStop());
  }

  public Command runIntakeStopOnceCommand() {
    return run(() -> RunIntakeStop());
  }
  public Command runIntakePIDfifteenCommand() {
    return run(() -> runIntakefifteenPID());
  }
  public Command runIntakePIDtenCommand() {
    return run(() -> runIntaketenPID());
  }

  /*--------------2.SparkMax----------- */

  public Command TakeObjectCommand() {
    return run(() -> TakeObject());
  }

  public Command TakeObjectOnceCommand() {
    return run(() -> TakeObjectOnceCommand());
  }

  public Command ReverseObjectCommand() {
    return run(() -> ReverseObjectCommand());
  }

  public Command ReverseObjectOnceCommand() {
    return run(() -> ReverseObjectOnceCommand());
  }

  public Command ObjectStopCommand() {
    return run(() -> ObjectStopCommand());
  }

  public Command ObjectStopOnceCommand() {
    return run(() -> ObjectStopOnceCommand());
  }

}
