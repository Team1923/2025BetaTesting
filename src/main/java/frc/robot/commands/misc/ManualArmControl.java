// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends Command {


  private ArmSubsystem arm;
  private DoubleSupplier control;
  private StatusSignal<Double> voltage;

  /** Creates a new ManualArmControl. */
  public ManualArmControl(ArmSubsystem arm, DoubleSupplier control) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm = arm;
    this.control = control;

    this.voltage = arm.getArmVoltage();

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPercentOut(control.getAsDouble());
    SmartDashboard.putNumber("VOLTAGE", voltage.getValueAsDouble());
    SmartDashboard.putNumber("PERCENT", arm.getPercentOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
