// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;

public class SimpleIntakeArmStateSwapCommand extends Command {

  private IntakeArmStates state1;
  private IntakeArmStates state2;
  
  /** Creates a new SimpleStateSwapCommand. */
  public SimpleIntakeArmStateSwapCommand(IntakeArmStates state1, IntakeArmStates state2) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.state1 = state1;
    this.state2 = state2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StateHandler.getInstance().desiredIntakeArmState = state1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    StateHandler.getInstance().desiredIntakeArmState = state2;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
