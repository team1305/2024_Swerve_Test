// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class PathfindToPose extends Command {
  /** Creates a new PathfindToPose. */
  private final DriveSubsystem m_drive;

  private final double m_x;
  private final double m_y;
  private final double m_rot;
  private final double m_endvel;
  private final double m_rotdelay;

  public PathfindToPose(DriveSubsystem drive, double Xpoint, 
  double Ypoint, double rotation, double endvelocity, double rotationdelay) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = Xpoint;
    m_y = Ypoint;
    m_rot = rotation;
    m_endvel = endvelocity;
    m_rotdelay = rotationdelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.PathFindToPose(
    m_x, 
    m_y, 
    m_rot, 
    m_endvel, 
    m_rotdelay);
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
