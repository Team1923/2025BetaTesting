// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class BabyBirdCommand extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new BabyBirdCommand. */
  public BabyBirdCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.desiredShooterState = ShooterStates.BABY_BIRD_VELO;
    stateHandler.desiredArmState = ArmStates.BABY_BIRD;
    stateHandler.desiredFeederState = FeederStates.FEED_TO_INTAKE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.desiredShooterState = ShooterStates.IDLE_VELO;
     stateHandler.desiredArmState = ArmStates.STOWED;
    stateHandler.desiredFeederState = FeederStates.OFF;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.bb4Covered && stateHandler.bb3Covered; //TODO: this is slightly different logic, is it fine?
  }
}
