// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final CommandPS5Controller testPS5 = new CommandPS5Controller(1);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * s
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    testPS5.povUp().toggleOnTrue(m_intake.runIntakeInCommand());
    testPS5.povUp().toggleOnFalse(m_intake.runIntakeStopCommand());
    testPS5.povDown().toggleOnTrue(m_intake.runIntakeOutCommand());
    testPS5.povDown().toggleOnFalse(m_intake.runIntakeStopCommand());

    testPS5.povLeft().toggleOnTrue(m_elevator.RunElevatorUpCommand());
    testPS5.povLeft().toggleOnFalse(m_elevator.RunElevatorStopCommand());
    testPS5.povRight().toggleOnTrue(m_elevator.RunElevatorDownCommand());
    testPS5.povRight().toggleOnFalse(m_elevator.RunElevatorStopCommand());

     testPS5.L1().toggleOnTrue(m_intake.TakeObjectCommand());
    testPS5.L1().toggleOnFalse(m_intake.ObjectStopCommand());
    testPS5.L2().toggleOnTrue(m_intake.ReverseObjectCommand());
    testPS5.L2().toggleOnFalse(m_intake.ObjectStopCommand());

    testPS5.R1().whileTrue(m_intake.runIntakePIDfifteenCommand());
    testPS5.R1().whileFalse(m_intake.runIntakeStopCommand());
    testPS5.R2().whileTrue(m_intake.runIntakePIDtenCommand());
    testPS5.R2().whileFalse(m_intake.runIntakeStopCommand());
    
    
    /* 
      testPS5.povLeft().onTrue(
      new InstantCommand(() -> {
      m_intake.RunIntakeIn();
      }));
      testPS5.povLeft().onFalse(
      new InstantCommand(() -> {
      m_intake.RunIntakeStop();
      }));
      
      testPS5.povRight().onTrue(
      new InstantCommand(() -> {
      m_intake.RunIntakeOut();
      
      }));
      testPS5.povRight().onFalse(
      new InstantCommand(() -> {
      m_intake.RunIntakeStop();
      }));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
