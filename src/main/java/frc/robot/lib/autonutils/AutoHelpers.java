// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.autonutils;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.statecommands.SwerveStateMachine;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoHelpers {


  /* PATH HELP */

  public static Command commandPathFrom(String pathName){

    PathPlannerPath path;
    try { //SHOULD return here
      path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);

    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

     return generateBlankCommand();
  }


  /* PATHFINDING */
    public static Pose2d getLastPoseOf(PathPlannerPath path){
    
    List<Pose2d> poses = path.getPathPoses();

    Pose2d lastPose = poses.get(poses.size()-1);

    return new Pose2d(lastPose.getX(), lastPose.getY(), path.getGoalEndState().rotation());

  }
  public static Command pathfindToEndOfPath(String name){
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(name);
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();

      return generateBlankCommand();
      
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();

      return generateBlankCommand();

    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();

      return generateBlankCommand();

    }


    return AutoBuilder.pathfindToPose(getLastPoseOf(path), path.getGlobalConstraints());
  }

  public static Command goalCentricShoot(){
    return new ParallelDeadlineGroup(new ShootGamePiece(), new SwerveStateMachine(SwerveSubsystem.getInstance(), () -> 0, () -> 0, () -> 0));
  }


  private static Command generateBlankCommand(){
    return new Command() {
      public boolean isFinished() {
        return true;
      };
    };
  }



}
