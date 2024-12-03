// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.AmpRanged;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.lib.autonutils.AutoHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpRanged213 extends SequentialCommandGroup {
  /** Creates a new AmpRanged213. */
  public AmpRanged213() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      /*Preload*/
      new PathPlannerAuto("StartAmpRanged"),
      AutoHelpers.goalCentricShoot(),

      /*Note 2*/
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidFieldRangedTo2"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.commandPathFrom("2ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot(),

       /*Note 1*/
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidFieldRangedTo1"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.commandPathFrom("1ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot(),

      /*Note 3*/
      new ParallelDeadlineGroup(
        AutoHelpers.commandPathFrom("MidFieldRangedTo3"),
        new DeployIntakeCommand()
      ),
      AutoHelpers.commandPathFrom("3ToMidFieldRanged"),
      AutoHelpers.goalCentricShoot()
    );
  }
}
