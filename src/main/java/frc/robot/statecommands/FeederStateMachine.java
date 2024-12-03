// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class FeederStateMachine extends Command {

  private FeederSubsystem feederSubsystem;
  
  private StateHandler stateHandler = StateHandler.getInstance();

  private Timer stallTimer;

  /** Creates a new FeederStateMachine. */
  public FeederStateMachine(FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.feederSubsystem = feederSubsystem;

    stallTimer = new Timer();

    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * MAIN FUNCTIONALITY REQUIRED:
     * - BACKING + FORWARD MOTION --> should be done in a switch statement's default case
     * - SCORING --> INWARD (look at old code, see how many cases in which the feeder runs inward)
     * - EJECT --> same idea as scoring, but only validate the intake position
     */


     

    FeederStates desiredState = stateHandler.desiredFeederState;


    if (desiredState != FeederStates.FEED_TO_SHOOTER && stallTimer.get() != 0){ //Timer that forces feeder to feed if desired is FEED_TO_SHOOTER and current hasn't been allowed to change (prevents shooter stalling)
      stallTimer.stop();
      stallTimer.reset();
    }


    if (desiredState == FeederStates.FULL_EJECT){ //if full eject, fall through to the end and let the motors be set (we want to prioritize this)

    }
    else if (desiredState == FeederStates.FEED_TO_INTAKE
          && stateHandler.desiredIntakeArmState == IntakeArmStates.DEPLOYED
          && stateHandler.currentIntakeArmState != IntakeArmStates.DEPLOYED){ //wait to eject note until IntakeArm is fully deployed

            desiredState = FeederStates.OFF; 
    }
    else if (stateHandler.latchingBB){ //continue feeding if there is a note actively in the intake (latching boolean)
      desiredState = FeederStates.FEED_TO_SHOOTER;
    }
    /* I think there may be merit to putting this here */
    // else if (stateHandler.hasGamePiece() && stateHandler.bb4Covered){
    //   desiredState = FeederStates.BACKING;
    // }
    /* SCORING */
    else if (stateHandler.hasGamePiece() && desiredState == FeederStates.FEED_TO_SHOOTER){ //this will technically activate while intaking, but shouldn't matter since no shooting command will run.  

      if (stallTimer.get() == 0){ //if the feeder was just set to FEED_TO_SHOOTER, reset the stall timer
        stallTimer.start();
      }

      if (!stallTimer.hasElapsed(FeederConstants.timeout)){ //if we haven't stalled out, check the normal shooting logic checks to make sure that the other subsystems are at the correct states to shoot.
        switch(stateHandler.scoringType){                   // (If we have stalled out, this statement will end, and we will fall through to the setting statements at the bottom of execute)

        case AMP: 
          if (stateHandler.currentArmState == ArmStates.FRONT_AMP
            && stateHandler.currentShooterState == ShooterStates.FRONT_AMP_VELO){
              break;
          }

        case SUBWOOFER:
          if (stateHandler.currentArmState == ArmStates.SUBWOOFER
            && stateHandler.currentShooterState == ShooterStates.SUBWOOFER_VELO){
              break;
          }

        case REVERSE_SUBWOOFER:
          if (stateHandler.currentArmState == ArmStates.REVERSE_SUBWOOFER
            && stateHandler.currentShooterState == ShooterStates.REVERSE_SUBWOOFER_VELO){
              break;
          }
        case TUNING: //fall through to ranged condition
          if (stateHandler.currentArmState == ArmStates.ANGLE_TUNING
            && stateHandler.currentShooterState == ShooterStates.RPM_TUNING
            && ( (stateHandler.isCenteredToSpeakerTag() && stateHandler.isInSpeakerRange()) )){
              break;
            } 
        case RANGED: 
          if (stateHandler.currentArmState == ArmStates.RANGED
            && stateHandler.currentShooterState == ShooterStates.RANGED_VELO
            && ( (stateHandler.isCenteredToSpeakerTag() && stateHandler.isInSpeakerRange()) )){
              break;
            } 

        case TRAP:
          if (stateHandler.currentArmState == ArmStates.TRAP
            && stateHandler.currentShooterState == ShooterStates.TRAP_VELO){
              break;
            }

        case HIGH_PUNT:
          if (stateHandler.currentArmState == ArmStates.PUNT_HIGH
            && stateHandler.currentShooterState == ShooterStates.PUNT_HIGH_VELO){
              break;
            }

        case LOW_PUNT:
          if (stateHandler.currentArmState == ArmStates.PUNT_LOW
            && stateHandler.currentShooterState == ShooterStates.PUNT_LOW_DUTY){
              break;
            }

        case UNGUARDABLE:
          if (stateHandler.currentArmState == ArmStates.UNGUARDABLE
            && stateHandler.currentShooterState == ShooterStates.UNGUARDABLE_VELO){
              break;
            }

        case CLIMB: //falls through, shouldn't be feeding in climb
                  
        default: //if one of these checks failed, make sure the feeder stays off until it is fufilled
          desiredState = FeederStates.OFF;
          }
      }
      
      
    }
    else if (desiredState == FeederStates.OFF
            && stateHandler.bb4Covered){ //backing condition when we aren't doing anything else

            desiredState = FeederStates.BACKING;
    } else if(desiredState == FeederStates.OFF
            && stateHandler.bb2Covered
            && !stateHandler.bb3Covered) {
              desiredState = FeederStates.FRONTING;
            }


    if (feederSubsystem.isAtState(desiredState)){
      stateHandler.currentFeederState = desiredState;
    }

    feederSubsystem.setFeederTo(desiredState.REQUEST);


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
