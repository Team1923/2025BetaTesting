// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.AmpSubwooferRanged;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.ScoringType;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.lib.autonutils.AutoHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpSubwooferRanged123 extends SequentialCommandGroup {
  /** Creates a new AmpRanged123. */
  public AmpSubwooferRanged123() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      RobotContainer.scoringMode(ScoringType.SUBWOOFER),
      new ShootGamePiece(),
      RobotContainer.scoringMode(ScoringType.RANGED),


      /* Note 1 */
      new ParallelDeadlineGroup(
        new PathPlannerAuto("StartAmpSubwooferRanged1"),
        new SequentialCommandGroup(
            new WaitCommand(2.0),
          new DeployIntakeCommand()
        )

      ),
      AutoHelpers.commandPathFrom("1ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot(),
      
      /* Note 2 */
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidFieldRangedTo2"),
        new SequentialCommandGroup(
            new WaitCommand(2.0),
          new DeployIntakeCommand()
          )
      ),
      AutoHelpers.commandPathFrom("2ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot(),

      /* Note 3 */
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidFieldRangedTo3"),
         new SequentialCommandGroup(
            new WaitCommand(1.3),
          new DeployIntakeCommand()
          )
      ),
      AutoHelpers.commandPathFrom("3ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot()
    );
  }
}
