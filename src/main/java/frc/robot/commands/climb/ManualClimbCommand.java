// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.StateHandler.ScoringType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;

public class ManualClimbCommand extends Command {

  private ArmSubsystem armSubsystem;
  private DoubleSupplier input;

  private MotionMagicVoltage holdingRequest;

  /** Creates a new ManualClimbCommand. */
  public ManualClimbCommand(ArmSubsystem armSubsystem, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.input = input;

    if (ArmStates.CLIMB.REQUEST instanceof MotionMagicVoltage){
        holdingRequest = (MotionMagicVoltage)ArmStates.CLIMB.REQUEST;
    }
    else{
      for (int i = 0; i < 100; i++){
        System.out.println("FATAL ERROR ON CLIMB");
      }
      holdingRequest = new MotionMagicVoltage(0);
    }

    addRequirements(armSubsystem);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ArmStates.CLIMB.REQUEST instanceof MotionMagicVoltage){
        holdingRequest.Position = ((MotionMagicVoltage)ArmStates.CLIMB.REQUEST).Position;
    }
    else{
      for (int i = 0; i < 100; i++){
        System.out.println("FATAL ERROR ON CLIMB");
      }
      holdingRequest = new MotionMagicVoltage(0);
    }


  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(input.getAsDouble()) > ControllerConstants.Operator.deadband){

      // SmartDashboard.putNumber("INPUT", input.getAsDouble());

      armSubsystem.setPercentOut(input.getAsDouble() * 0.5);
      holdingRequest.Position = armSubsystem.getArmPositionRots();
      
    }
    else{
      armSubsystem.setArmTo(holdingRequest);
    }

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    StateHandler.getInstance().scoringType = ScoringType.RANGED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
