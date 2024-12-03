// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.fasterxml.jackson.core.StreamWriteCapability;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

public class AlignHeadingCommand extends Command {

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new AlignHeadingCommand. */
  public AlignHeadingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(stateHandler.scoringType){
      case CLIMB:
        stateHandler.swerveState = SwerveStates.FACING_CLIMB;
        break;
      case TRAP:
        stateHandler.swerveState = SwerveStates.FACING_TRAP;
        break;
      case AMP:
        stateHandler.swerveState = SwerveStates.FACING_AMP;
        break;
      case RANGED:
        stateHandler.swerveState = SwerveStates.GOAL_CENTRIC;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.swerveState = SwerveStates.FIELD_CENTRIC;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
