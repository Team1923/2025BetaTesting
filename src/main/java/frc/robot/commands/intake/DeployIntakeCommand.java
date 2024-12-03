// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerStates;

public class DeployIntakeCommand extends Command {

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.desiredIntakeArmState = IntakeArmStates.DEPLOYED;
    stateHandler.desiredIntakeRollerState = IntakeRollerStates.INTAKE;
    stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.desiredIntakeArmState = IntakeArmStates.STOWED;
    stateHandler.desiredFeederState = FeederStates.OFF;
    stateHandler.desiredIntakeRollerState = IntakeRollerStates.OFF;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.bb3Covered;
  }
}
