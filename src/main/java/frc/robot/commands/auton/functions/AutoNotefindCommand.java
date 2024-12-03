// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.functions;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

public class AutoNotefindCommand extends Command {

  private StateHandler stateHandler = StateHandler.getInstance();

  public enum SearchDirection{
    
    TOWARDS_AMP(1),
    TOWARDS_SOURCE(-1);

    private int dir;

    private SearchDirection(int dir){
      this.dir = dir;
    }

    public int direction(){
      return dir;
    }
  }

  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private int direction;

  /** Creates a new AutoNotefindCommand. */
  public AutoNotefindCommand(SearchDirection searchDirection) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve);

    this.direction = searchDirection.direction();
              

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isFinished()){
      swerve.setControl(((SwerveRequest.FieldCentricFacingAngle)SwerveStates.NOTEFIND.REQUEST)
                        .withVelocityY(AutonConstants.notefindingSpeed*direction)
                        .withVelocityX(0)
                        .withTargetDirection(Rotation2d.fromDegrees(90)));
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(SwerveStates.FIELD_CENTRIC.REQUEST);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOutOfBounds() || stateHandler.bb1Covered || stateHandler.bb2Covered || stateHandler.bb3Covered;
  }

  private boolean isOutOfBounds(){

    Pose2d currentPose = swerve.getState().Pose;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      return currentPose.getX() >= AutonConstants.whiteLineBoundX + AutonConstants.whiteLineTolerance
          || currentPose.getY() <= AutonConstants.sourceBoundY
          || currentPose.getY() >= AutonConstants.ampBoundY;
    }
    else{
      return currentPose.getX() <= AutonConstants.whiteLineBoundX - AutonConstants.whiteLineTolerance
          || currentPose.getY() <= AutonConstants.sourceBoundY
          || currentPose.getY() >= AutonConstants.ampBoundY;
    }

    
  }
}
