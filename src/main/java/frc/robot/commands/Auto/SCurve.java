// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TrajectoryResolver;

public class SCurve extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public SCurve(
    DriveSubsystem drivebase
  ) {
    super();
    addRequirements(drivebase);
    String name = "S_Curve";
    PathPlannerPath path1 = TrajectoryResolver.getTrajectoryFromPath(name);
  
    addCommands(
      AutoBuilder.followPathWithEvents(path1)
    );
  }

}
