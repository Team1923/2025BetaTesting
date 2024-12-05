// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.lib.swerve.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;
import pabeles.concurrency.IntOperatorTask.Max;

public class SwerveStateMachine extends Command {

  private SwerveSubsystem swerve;


  private StateHandler stateHandler = StateHandler.getInstance();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = DegreesPerSecond.of(540).in(RadiansPerSecond);

  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;

  /** Creates a new ManageRequests. */
  /**
   * 
   * @param swerve
   * @param translation Expected to be + away from driver station, - towards driver station
   * @param strafe Expected to be + rightward relative to driver station, - leftward relative to driver station
   * @param rotation Expected to be CCW +
   */
  public SwerveStateMachine(SwerveSubsystem swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.


    this.swerve = swerve;
    this.translationSupplier = translation;
    this.strafeSupplier = strafe;
    this.rotationSupplier = rotation;

    addRequirements(this.swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveStates currentState = stateHandler.swerveState;


    double translation = MathUtil.applyDeadband(translationSupplier.getAsDouble() * sideInversions()[0], ControllerConstants.Driver.deadband);
    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble() * sideInversions()[1], ControllerConstants.Driver.deadband);
    double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble() * sideInversions()[2], ControllerConstants.Driver.deadband);

    // if (currentRequest == SwerveRequests.AUTO && !DriverStation.isAutonomous()){
    //   System.out.println("SWERVE STILL THINKS IN AUTO");
    //   currentRequest = SwerveRequests.FIELD_CENTRIC;
    // }


    SwerveRequest request;

    switch(currentState){ //set based on the states using the controller input
      case NOTEFIND:
        stateHandler.swerveState = SwerveStates.FIELD_CENTRIC;
      case FIELD_CENTRIC: 
          request = ((SwerveRequest.FieldCentric)SwerveStates.FIELD_CENTRIC.REQUEST)
            .withVelocityX(translation * MaxSpeed)
            .withVelocityY(strafe * MaxSpeed)
            .withRotationalRate(rotation * MaxAngularRate);
            break;

      case ROBOT_CENTRIC:
        request = ((SwerveRequest.RobotCentric)SwerveStates.ROBOT_CENTRIC.REQUEST)
        .withVelocityX(translation * MaxSpeed)
        .withVelocityY(strafe * MaxSpeed)
        .withRotationalRate(rotation * MaxAngularRate);
        break;

      case FACING_AMP:
        request = ((SwerveRequest.FieldCentricFacingAngle)SwerveStates.FACING_AMP.REQUEST)
        .withVelocityX(translation * MaxSpeed)
        .withVelocityY(strafe * MaxSpeed)
        .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90)); //TODO: check headings
        break;
      case FACING_TRAP:
        request = ((SwerveRequest.FieldCentricFacingAngle)SwerveStates.FACING_AMP.REQUEST)
        .withVelocityX(translation * MaxSpeed)
        .withVelocityY(strafe * MaxSpeed)
        .withTargetDirection(Rotation2d.fromDegrees(roundToClosestTrapHeading(swerve.getGyroYaw())));
        break;
      case FACING_CLIMB:
         request = ((SwerveRequest.FieldCentricFacingAngle)SwerveStates.FACING_AMP.REQUEST)
        .withVelocityX(translation * MaxSpeed)
        .withVelocityY(strafe * MaxSpeed)
        .withTargetDirection(Rotation2d.fromDegrees(roundToClosestClimbHeading(swerve.getGyroYaw())));
        break;
      case GOAL_CENTRIC:
        if (stateHandler.hasSpeakerTag() && Math.abs(rotation) < 0.5){
          request = ((SwerveRequest.FieldCentricFacingAngle)SwerveSubsystem.SwerveStates.GOAL_CENTRIC.REQUEST)
          .withVelocityX(0.4 * translation * MaxSpeed)
          .withVelocityY(0.4 * strafe * MaxSpeed)
          .withTargetDirection(Rotation2d.fromDegrees(Math.IEEEremainder(swerve.getGyroYaw()+stateHandler.llTx(), 360)));

          // System.out.println(Rotation2d.fromDegrees(swerve.getGyroYaw()+stateHandler.llTx()).rotateBy(parameters.operatorForwardDirection););

          break;
        }
      default://Equivilent to field centric
        request = ((SwerveRequest.FieldCentric)SwerveStates.FIELD_CENTRIC.REQUEST)
                  .withVelocityX(translation * MaxSpeed)
                  .withVelocityY(strafe * MaxSpeed)
                  .withRotationalRate(rotation * MaxAngularRate);

    }
    
    // else if (currentRequest == SwerveRequests.NOTE_SEARCHING){
    //    requestObj =((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.NOTE_SEARCHING.request)
    //     .withVelocityX(0.5 * sideInversions()[0] * MaxSpeed)
    //     .withVelocityY(0.5 * sideInversions()[1] * MaxSpeed)
    //     .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90));

    //     System.out.println("HERER TOO");
    // }

    
    

    swerve.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public static int[] sideInversions(){
    return (DriverStation.getAlliance().get() == Alliance.Blue) ? new int[]{1,1,1}
                                                                : new int[]{-1,-1, 1};
  }

  public static double roundToClosestTrapHeading(double ang){
    double remainder = ang % 360;

    if(remainder > 180){
      remainder -= 360;
    }
    else if(remainder < -180){
      remainder += 360;
    }
    
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      if(remainder >= -60 && remainder < 60){
        return 0;
      }
      else if(remainder >= 60 && remainder < 180){
        return 120;
      }
      else{
        return -120;
      }
    }
    else{ //TODO: implement red
      return 60;
    }
    
  }

  public static double roundToClosestClimbHeading(double ang){
    double remainder = ang % 360;
    if(remainder > 180){
      remainder -= 360;
    }
    else if(remainder < -180){
      remainder += 360;
    }

     if(remainder >= 0 && remainder < 120){
      return 60;
    }
    else if(remainder >= -120 && remainder < 0){
      return -60;
    }
    else{
      return 180;
    }
  }
}