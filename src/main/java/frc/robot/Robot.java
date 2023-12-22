// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// Start - this can be deleted once we get autos working properly
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
// End - This can be deleted once we get autos working properly

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ToggleDriveCentricity;


import frc.robot.Constants.DriverControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.DriveSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private final DriveSubsystem m_drive = new DriveSubsystem(true);

  public static final XboxController m_controller = new XboxController(0);
  

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureButtonBindings();
    setDefaultCommands();
    setupAutoChoosers();
  }

  private void configureButtonBindings(){
    new JoystickButton(m_controller, DriverControllerConstants.RIGHT_BUMPER).onTrue(new ToggleDriveCentricity(m_drive));
    new JoystickButton(m_controller, DriverControllerConstants.LEFT_BUMPER).whileTrue(new RunCommand(() -> m_drive.setX(),m_drive));
    new JoystickButton(m_controller, DriverControllerConstants.A_BUTTON).whileTrue(new RunCommand(() -> m_drive.PathFindToPose(7.0, 1.0, 0.0, 0.0, 0.0),m_drive));
    //new JoystickButton(m_controller, DriverControllerConstants.A_BUTTON).whileTrue(new PathfindToPose(m_drive, 0.0, 0.0, 0.0, 0.0, 0.0));
  }

  private void setDefaultCommands(){
    m_drive.setDefaultCommand(
      new SwerveDrive(
        m_drive,
        () -> m_controller.getLeftX(),
        () -> m_controller.getLeftY(),
        () -> m_controller.getRightX()
      )
    );
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link Robot} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    Command autoCommand = getAutonomousCommand();

    autoCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  private void setupAutoChoosers(){ 
    new PathPlannerAuto("Example Auto1");
    new PathPlannerAuto("Straight Auto");
    new PathPlannerAuto("S Curve");
    new PathPlannerAuto("Swerve Accuracy Test");
    new PathPlannerAuto("Swerve Wait Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
