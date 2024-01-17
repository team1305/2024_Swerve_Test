// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  public NetworkTable table;

  Optional<Alliance> alliance_colour;

  double height_of_limelight = 9.5; //Inches
  double height_of_target = 47.5; //Inches

  double angle_of_limelight_degrees = 26.5; // Degrees
  //double angle_of_limelight_radians = 17 * (Math.PI/180); //Radians

  /** Creates a new LimlightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    alliance_colour = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Targetted", is_Target());
    SmartDashboard.putNumber("tx", get_Tx());
    SmartDashboard.putNumber("ty", get_Ty());
    SmartDashboard.putNumber("Tag ID", get_tag_id());
    SmartDashboard.putNumber("distance", getDistance());
    SmartDashboard.putNumberArray("botpose", get_botpose());
  }

  public double get_Tx(){
    return table.getEntry("tx").getDouble(0.0);
  }

  public double get_Ty(){
    return table.getEntry("ty").getDouble(0.0);
  }

  public double get_Tv(){
    return table.getEntry("tv").getDouble(0);
  }

  public double getDistance(){
    double distance = 0;

    if (is_Target()){

    double angleToGoalDegrees = angle_of_limelight_degrees + get_Ty();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    distance = (height_of_target - height_of_limelight) / Math.tan(angleToGoalRadians);
    }
    return distance;
  }

  public boolean is_Target(){
    if (get_Tv() == 1) {  
      return true;
   } else {
    return false;
   }
  }

  public void setStream(int ivalue){ // 0 - standard (double stream if availibe), 1 - PiP secondary small, 2 - PiP secondary big
    table.getEntry("stream").setNumber(ivalue);
  }

  public double get_tag_id(){
    return table.getEntry("tid").getDouble(0);
  }

  public double[] get_botpose(){
    /*
    if (alliance_colour.get() == Alliance.Red) {
      return table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }
    if (alliance_colour.get() == Alliance.Blue) {
      return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);     
    }
    return null;*/
    return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); 
    //return table.getEntry("botpose").getDoubleArray(new double[6]);
  }

  public double get_Latency_From_Bot_Pose(){
    return Timer.getFPGATimestamp() - (get_botpose()[6]/1000.0);
  }
  
}
