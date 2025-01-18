// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstans;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax m_IntakeMotor = new SparkMax(5, MotorType.kBrushless);

  public void RunIntakeIn() { // ıntake motor gücü (+ yönlü)
    m_IntakeMotor.setVoltage(3);
  }

  public void RunIntakeOut() { // ıntake motor gücü (-yön)
    m_IntakeMotor.setVoltage(-3);
  }

  public void RunIntakeStop() {
    m_IntakeMotor.setVoltage(0);
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command runIntakeInCommand() {return run(() -> RunIntakeIn());}
  public Command runIntakeInOnceCommand() {return run(() -> RunIntakeIn());}
  public Command runIntakeOutCommand() {return run(() -> RunIntakeIn());}
  public Command runIntakeOutOnceCommand() {return run(() -> RunIntakeIn());}
  public Command runIntakeStopCommand() {return run(() -> RunIntakeIn());}
  public Command runIntakeStopOnceCommand() {return run(() -> RunIntakeIn());}
}
