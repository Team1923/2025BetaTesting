// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.StateHandler.ScoringType;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;

public class ArmUpToClimb extends Command {

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmUpToClimb. */
  public ArmUpToClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      stateHandler.desiredArmState = ArmStates.CLIMB;
      stateHandler.desiredIntakeArmState = IntakeArmStates.DEPLOYED;
      stateHandler.scoringType = ScoringType.CLIMB;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stateHandler.currentArmState != ArmStates.CLIMB){
      stateHandler.scoringType = ScoringType.RANGED;
    }
    else{
      System.out.println("MANUALLY CLIMBING");

    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.currentArmState == ArmStates.CLIMB;
  }
}
