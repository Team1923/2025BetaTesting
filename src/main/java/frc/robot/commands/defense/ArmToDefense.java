// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defense;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.ArmSubsystem.ArmStates;

public class ArmToDefense extends Command {
  
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmToDefense. */
  public ArmToDefense() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.desiredArmState = ArmStates.DEFENSE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // "hi have fun tomorrow peace out" - Aalind T, Night before playoffs, Worlds 2024
    stateHandler.desiredArmState = ArmStates.STOWED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
