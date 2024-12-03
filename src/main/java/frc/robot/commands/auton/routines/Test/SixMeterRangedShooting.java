// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.Test;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.ScoringType;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.lib.autonutils.AutoHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixMeterRangedShooting extends SequentialCommandGroup {
  /** Creates a new SixMeterRangedShooting. */
  public SixMeterRangedShooting() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.scoringMode(ScoringType.SUBWOOFER),
      new ShootGamePiece(),
      RobotContainer.scoringMode(ScoringType.RANGED),

      new ParallelDeadlineGroup( //Picks Up Note
        new PathPlannerAuto("StartSubwooferTo2Auto"),
        new DeployIntakeCommand()
      ), 
      AutoHelpers.commandPathFrom("2ToRangedShoot"), //Shoots from ranged
      AutoHelpers.goalCentricShoot()
    );
  }
}
