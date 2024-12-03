// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateHandler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class ShooterStateMachine extends Command {

  private ShooterSubsystem shooterSubsystem;

  private StateHandler stateHandler = StateHandler.getInstance();

  

  private Timer puntTimer = new Timer();

  private boolean rangedSet;


  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rangedSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ShooterStates desiredState = stateHandler.desiredShooterState;

    switch(desiredState){
      
      case PUNT_LOW_DUTY: //If doing a low punt, set current position via a timer
        if (puntTimer.get() == 0){
          puntTimer.start();
        }
        else if (puntTimer.hasElapsed(0.5)){
          stateHandler.currentShooterState = ShooterStates.PUNT_LOW_DUTY;
        }
        break;

      case RANGED_VELO: //If in ranged mode, set update current velocity
        if (stateHandler.speakerDistance() != -1 && stateHandler .isCenteredToSpeakerTag() && !rangedSet){ //if we can update the angle, do it, otherwise stay at the last ranged angle
            ((MotionMagicVelocityVoltage)(ShooterStates.RANGED_VELO.REQUEST_TOP)).Velocity = InterpolationConstants.distanceToRPM.get(stateHandler.speakerDistance()) * ShooterConstants.RPMToRPS;
            ((MotionMagicVelocityVoltage)(ShooterStates.RANGED_VELO.REQUEST_BOTTOM)).Velocity = InterpolationConstants.distanceToRPM.get(stateHandler.speakerDistance()) * ShooterConstants.RPMToRPS;
          rangedSet = true;
        }
      default: //fall through to this in most cases
        rangedSet = false;
        puntTimer.stop(); //resetting punt timer once we are no longer doing a punt shot
        puntTimer.reset();
        if (shooterSubsystem.isAtState(desiredState)){ // current state check
            stateHandler.currentShooterState = desiredState;
        }
        break;
    }
  

    
    shooterSubsystem.setShooterMotorsTo(desiredState.REQUEST_TOP, desiredState.REQUEST_BOTTOM);
    shooterSubsystem.setBlowerTo(stateHandler.blowerState.PERCENT);
    

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
