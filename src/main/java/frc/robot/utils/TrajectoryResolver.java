package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;


public class TrajectoryResolver {
    public static PathPlannerPath path;

    public static PathPlannerPath getTrajectoryFromPath(String pathName){
        DriverStation.reportWarning(pathName, false);
        System.out.println("running get trajectory from path");
        try{
            return path = PathPlannerPath.fromPathFile(pathName);
        }
        catch (Exception e){
            DriverStation.reportError("Trajectory file not found. Returning blank trajectory " + e.getMessage(), false);
            return null;
        }
    }
}
