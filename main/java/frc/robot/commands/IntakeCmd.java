// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends CommandBase {
  private final Intake m_intake;
  XboxController m_xboxController = new XboxController(0);
  /** Creates a new IntakeCmd. */
  public IntakeCmd(Intake m_intake, XboxController m_xboxController) {

    this.m_intake = m_intake;
    this.m_xboxController = m_xboxController;
    addRequirements(m_intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.control(m_xboxController.getAButton(), m_xboxController.getBButton(), 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
