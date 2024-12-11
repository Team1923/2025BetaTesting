// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.StateHandler.ScoringType;
import frc.robot.commands.auton.routines.AmpRanged.AmpRanged123;
import frc.robot.commands.auton.routines.AmpRanged.AmpRanged213;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged5;
import frc.robot.commands.climb.ClimbingCommandGroup;
import frc.robot.commands.defense.ArmToDefense;
import frc.robot.commands.intake.BabyBirdCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.FullEjectCommand;
import frc.robot.commands.intake.IntakeEjectCommand;
import frc.robot.commands.misc.ManualArmControl;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.commands.swerve.AlignHeadingCommand;
import frc.robot.commands.swerve.SetGoalCentric;
import frc.robot.lib.autonutils.AutoInstatiateSelector;
import frc.robot.lib.misc.ControllerQuadraticLimiter;
import frc.robot.lib.swerve.Telemetry;
import frc.robot.lib.swerve.TunerConstants;
import frc.robot.statecommands.ArmStateMachine;
import frc.robot.statecommands.FeederStateMachine;
import frc.robot.statecommands.IntakeStateMachine;
import frc.robot.statecommands.ShooterStateMachine;
import frc.robot.statecommands.SwerveStateMachine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.InfoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

public class RobotContainer {

  StateHandler stateHandler = StateHandler.getInstance();

  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS5Controller operatorPS5Controller = new CommandPS5Controller(1);

  /* Subsystem Instantiations */
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
  private final ArmSubsystem armSubsystem = new ArmSubsystem(); 
  private final FeederSubsystem feederSubsystem = new FeederSubsystem(); 
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  /* Helper class Instantiation */
  private final InfoSubsystem infoSubsystem = new InfoSubsystem(driverXboxController, operatorPS5Controller);
  private final Telemetry swerveLogger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private static InterpolationConstants interpConsts = new InterpolationConstants(); //just needs to be constructed for the sake of constructor
  private final AutoInstatiateSelector autoInstatiateSelector = new AutoInstatiateSelector();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default commands */
    shooterSubsystem.setDefaultCommand(new ShooterStateMachine(shooterSubsystem)); 
    intakeSubsystem.setDefaultCommand(new IntakeStateMachine(intakeSubsystem)); 
    armSubsystem.setDefaultCommand(new ArmStateMachine(armSubsystem)); 
    feederSubsystem.setDefaultCommand(new FeederStateMachine(feederSubsystem)); 
    swerveSubsystem.setDefaultCommand(new SwerveStateMachine(swerveSubsystem, 
                                      () -> -driverXboxController.getLeftY(),
                                      () -> -driverXboxController.getLeftX(), 
                                      () -> -driverXboxController.getRightX()));

    /*They don't have this in the new example */
    // if (Utils.isSimulation()) {
    //   swerveSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    // }

    swerveSubsystem.registerTelemetry(swerveLogger::telemeterize);

    /* Driver Button Bindings */
    driverXboxController.rightTrigger().whileTrue(new ShootGamePiece());
    driverXboxController.rightStick().whileTrue(new AlignHeadingCommand());
    driverXboxController.y().onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.zeroGyro()));

    /* Operator Button Bindings */

    /* Scoring selection */
    operatorPS5Controller.triangle().onTrue(scoringMode(ScoringType.RANGED));
    operatorPS5Controller.square().onTrue(scoringMode(ScoringType.AMP));
    operatorPS5Controller.cross().onTrue(scoringMode(ScoringType.SUBWOOFER));
    operatorPS5Controller.circle().onTrue(scoringMode(ScoringType.REVERSE_SUBWOOFER) );
    operatorPS5Controller.povUp().onTrue(scoringMode(ScoringType.TRAP));
    operatorPS5Controller.L2().onTrue(scoringMode(ScoringType.LOW_PUNT));
    operatorPS5Controller.R2().onTrue(scoringMode(ScoringType.HIGH_PUNT));

    /* Intaking/Ejecting */
    operatorPS5Controller.povLeft().whileTrue(new BabyBirdCommand());
    operatorPS5Controller.create().whileTrue(new FullEjectCommand());
    operatorPS5Controller.R1().whileTrue(new DeployIntakeCommand());
    operatorPS5Controller.L1().whileTrue(new IntakeEjectCommand());


    /* Misc */
    operatorPS5Controller.options().toggleOnTrue(new ClimbingCommandGroup(armSubsystem, () -> /*ControllerQuadraticLimiter.quadraticLimit(*/operatorPS5Controller.getRightY())); //#7



    // operatorPS5Controller.povDown().whileTrue(new ArmToDefense());


    if (stateHandler.isAngleRPMTuning){
      operatorPS5Controller.touchpad().onTrue(scoringMode(ScoringType.TUNING));
    }

    // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverXboxController.a().and(driverXboxController.y()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
        driverXboxController.a().and(driverXboxController.x()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
        driverXboxController.b().and(driverXboxController.y()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
        driverXboxController.b().and(driverXboxController.x()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));









  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    // return new PathPlannerAuto("CenterSource5");
    return autoInstatiateSelector.startMode();
  }


  public static Command scoringMode(ScoringType scoringType){
    return new InstantCommand( () -> StateHandler.getInstance().scoringType = scoringType); 
  }
}
