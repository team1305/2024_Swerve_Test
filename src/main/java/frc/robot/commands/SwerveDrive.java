// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends Command {

  private final DriveSubsystem m_drive;

  private final DoubleSupplier m_X;
  private final DoubleSupplier m_Y;
  private final DoubleSupplier m_Z;


  /** Creates a new SwerveDrive. */
  public SwerveDrive(DriveSubsystem drive, DoubleSupplier X, DoubleSupplier Y, DoubleSupplier Z) {
   addRequirements(drive);
   m_drive = drive;
   m_X = X;
   m_Y = Y;
   m_Z = Z;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(
      -MathUtil.applyDeadband(m_Y.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_X.getAsDouble(), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_Z.getAsDouble(), OIConstants.kDriveDeadband)
    );
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
