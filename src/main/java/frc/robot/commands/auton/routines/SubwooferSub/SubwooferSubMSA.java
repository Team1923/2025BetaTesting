package frc.robot.commands.auton.routines.SubwooferSub;

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
public class SubwooferSubMSA extends SequentialCommandGroup {
        /** Creates a new SubwooferRangedSMA12. */
        public SubwooferSubMSA () {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(
                                // Preload
                                RobotContainer.scoringMode(ScoringType.SUBWOOFER),
                                new ShootGamePiece(),
                                // Middle
                                new ParallelDeadlineGroup(
                                                new PathPlannerAuto("StartSubwooferSubM"),
                                                new DeployIntakeCommand()),

                                AutoHelpers.commandPathFrom("MiddleToSubwooferStart"),
                                new ShootGamePiece(),

                                // Stage
                                new ParallelDeadlineGroup(
                                                AutoHelpers.commandPathFrom("SubwooferStartToStage"),
                                                new DeployIntakeCommand()),

                                AutoHelpers.commandPathFrom("StageToSubwooferStart"),
                                new ShootGamePiece(),

                                // Amp
                                new ParallelDeadlineGroup(
                                                AutoHelpers.commandPathFrom("SubwooferStartToAmp"),
                                                new DeployIntakeCommand()),

                                AutoHelpers.commandPathFrom("AmpToSubwooferStart"),
                                new ShootGamePiece()

                );
        }
}

