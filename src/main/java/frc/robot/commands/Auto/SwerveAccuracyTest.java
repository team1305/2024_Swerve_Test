// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TrajectoryResolver;

public class SwerveAccuracyTest extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public SwerveAccuracyTest(
    DriveSubsystem drivebase
  ) {
    super();
    addRequirements(drivebase);
    String name = "Swerve_Accuracy_Test";
    PathPlannerPath path1 = TrajectoryResolver.getTrajectoryFromPath(name);
  
    addCommands(
      Commands.runOnce(() -> drivebase.resetOdometry(path1.getPreviewStartingHolonomicPose())),
      AutoBuilder.followPathWithEvents(path1)
    );
  }

}
