// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.SourceSubwooferRanged;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.functions.AutoNotefindCommand;
import frc.robot.commands.auton.functions.AutoNotefindCommand.SearchDirection;
import frc.robot.commands.intake.BabyBirdCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.lib.autonutils.AutoHelpers;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceSubwooferRanged5 extends SequentialCommandGroup {
  /** Creates a new CenterSource5. */
  public SourceSubwooferRanged5() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new PathPlannerAuto("StartSourceSubwooferRanged5"),
            new AutoNotefindCommand(SearchDirection.TOWARDS_AMP)
          ),
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            new DeployIntakeCommand()
          )
          
      ),
      new ParallelDeadlineGroup(
        new DeployIntakeCommand()
      ),
      AutoHelpers.pathfindToEndOfPath("5ToRanged"),
      AutoHelpers.goalCentricShoot()

      );

  }

  
}
