// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TurnToAprilTag extends Command {
  
  LimelightSubsystem m_limelight;
  DriveSubsystem m_drivebase;

  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTag(DriveSubsystem drivebase, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
    m_limelight= limelight;
    m_drivebase = drivebase;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = m_drivebase.getHeading() + m_limelight.get_Tx();
    m_drivebase.turnOnLocationLock(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.turnOffLocationLock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
