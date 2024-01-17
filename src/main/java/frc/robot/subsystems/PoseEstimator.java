// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PoseEstimator extends SubsystemBase {
  private final DriveSubsystem m_drive;
  private final LimelightSubsystem m_limelight;

  //THIS NEED TO BE TUNED
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)); 
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(1000, 1000, Units.degreesToRadians(3600)); //0,.5,0.5,30
  private static SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(DriveSubsystem m_drive, LimelightSubsystem m_limelight) {
    this.m_drive = m_drive;
    this.m_limelight = m_limelight;

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      m_drive.getRotation2d(), 
      m_drive.getModulePositions(), 
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)), 
      stateStdDevs, 
      visionMeasurementStdDevs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    addvisionmeasurement();
    updatePose();
  }

  public void updatePose(){
   poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getRotation2d(), m_drive.getModulePositions());
  }

  public static Pose2d getcurrentpose() {
    return poseEstimator.getEstimatedPosition();
  
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(m_drive.getRotation2d(), m_drive.getModulePositions(), pose);
  }

  public void addvisionmeasurement(){
    poseEstimator.addVisionMeasurement(visionPose(), m_limelight.get_Latency_From_Bot_Pose());
  }


  public Pose2d visionPose(){
    Rotation2d visionrot = Rotation2d.fromDegrees(m_limelight.get_botpose()[5]);
    return new Pose2d(m_limelight.get_botpose()[0], m_limelight.get_botpose()[1], visionrot);
  }
}
