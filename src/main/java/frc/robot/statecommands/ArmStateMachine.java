// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;


import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;

public class ArmStateMachine extends Command {

  private ArmSubsystem armSubsystem;
  private Timer settleTimer;
  private Timer zeroTimer;
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmStateMachine. */

  private double startZeroingTime = 0.75;
  private double againstHardstopTime = 1.5;
  private double relaxedTime = 2;
  private boolean armZeroed;

  private boolean rangedSet;

  public ArmStateMachine(ArmSubsystem armSubsystem) {

    this.armSubsystem = armSubsystem;
    settleTimer = new Timer();
    zeroTimer = new Timer();
    armZeroed = false;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    settleTimer.stop();
    settleTimer.reset();

    zeroTimer.stop();
    zeroTimer.reset();

    armZeroed = false;
    rangedSet = false;

  }

  @Override
  public void execute() {

    //add arm self zeroing

    ArmStates desiredArmState = stateHandler.desiredArmState;

    switch(desiredArmState){
    

      case RANGED: //Update the ranged shot's motion magic value if we want ot shoot ranged
        if (stateHandler.speakerDistance() != -1 && stateHandler.isCenteredToSpeakerTag() && !rangedSet){ //if we can update the angle, do it, otherwise stay at the last ranged angle
            // ((MotionMagicVoltage)(ArmStates.RANGED.REQUEST)).Position = InterpolationConstants.distanceToAngle.get(stateHandler.speakerDistance()) * ArmConstants.armDegreesToRots;\
            ((MotionMagicVoltage)(ArmStates.RANGED.REQUEST)).Position = Units.degreesToRotations(InterpolationConstants.distanceToAngle.get(stateHandler.speakerDistance()));
            rangedSet = true;

            //TOdo only set when centered
        }
        break;
      default:
        rangedSet = false;
    }


    
    if (desiredArmState != ArmStates.STOWED){
      zeroTimer.stop();
      zeroTimer.reset();
      armZeroed = false;
    }
    else if (!armZeroed && armSubsystem.isAtState(ArmStates.STOWED) && desiredArmState == ArmStates.STOWED){
      zeroTimer.start();
    }
    else if (armZeroed == true){
      zeroTimer.stop();
    }

    if (!armZeroed){
      if (zeroTimer.hasElapsed(relaxedTime)){
      armSubsystem.zeroArm();
      armZeroed = true;
      }
      else if (zeroTimer.hasElapsed(againstHardstopTime)){
        desiredArmState = ArmStates.OFF;
      }
      else if (zeroTimer.hasElapsed(startZeroingTime)){
        desiredArmState = ArmStates.ZEROING;
      }
    }
    
  


   
    

    if (desiredArmState != stateHandler.currentArmState && armSubsystem.isAtState(desiredArmState) && settleTimer.get() == 0){ //Start the timer for the arm to settle when the arm motors are at the desired state (settle timer starts AFTER the motors are in the right place)
      settleTimer.restart();
    }


    //Check for current position; arm motors must read that they are at position AND settle timer must have elapsed
    if (settleTimer.hasElapsed(desiredArmState.settleTime) && armSubsystem.isAtState(desiredArmState)) {
      stateHandler.currentArmState = desiredArmState;
      settleTimer.stop();
      settleTimer.reset();
    }


    armSubsystem.setArmTo(desiredArmState.REQUEST);

    SmartDashboard.putBoolean("ISZEROED", armZeroed);

    SmartDashboard.putNumber("ZERO TIMER", zeroTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}
