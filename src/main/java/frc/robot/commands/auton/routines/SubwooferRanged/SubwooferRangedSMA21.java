// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.SubwooferRanged;

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
public class SubwooferRangedSMA21 extends SequentialCommandGroup {
  /** Creates a new SubwooferRangedSMA12. */
  public SubwooferRangedSMA21() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Preload
      RobotContainer.scoringMode(ScoringType.SUBWOOFER),
      new ShootGamePiece(),
      RobotContainer.scoringMode(ScoringType.RANGED),
      // Stage note
      new ParallelDeadlineGroup(
        new PathPlannerAuto("StartSubwooferRanged"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.goalCentricShoot(),
      // Middle note
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("StageToMiddle"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.goalCentricShoot(),
      // Amp note
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MiddleToAmp"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.goalCentricShoot(),
      // Note 2
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("AmpTo2"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.commandPathFrom("2ToMidRanged"),
      AutoHelpers.goalCentricShoot(),
      // Note 1
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidRangedTo1"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.commandPathFrom("1ToCloseRanged"),
      AutoHelpers.goalCentricShoot()
    );
  }
}
