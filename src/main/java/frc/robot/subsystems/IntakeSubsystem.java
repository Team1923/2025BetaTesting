// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.misc.SimpleIntakeArmStateSwapCommand;
import frc.robot.lib.tuningwidgets.MotorPIDFVAJWidget;
import frc.robot.StateHandler;

public class IntakeSubsystem extends SubsystemBase {
  /* Arm States Enum */
  public static enum IntakeArmStates {
    DEPLOYED(MMVoltage(120)), // original: 2.00 radians
    STOWED(MMVoltage(0)),
    TUNING(MMVoltage(0));

    public ControlRequest REQUEST;

    private IntakeArmStates(ControlRequest request) {
      this.REQUEST = request;
    }

    private static MotionMagicVoltage MMVoltage(double degrees) {

      // return new MotionMagicVoltage(degrees * IntakeConstants.intakeDegreesToRotations);
      return new MotionMagicVoltage(Units.degreesToRotations(degrees)).withEnableFOC(true);

    }
  }

  /* Roller States Enum */
  public static enum IntakeRollerStates {
    OFF(new DutyCycleOut(0)),
    EJECT(new DutyCycleOut(0.75)),
    INTAKE(new DutyCycleOut(-0.85));

    public ControlRequest REQUEST;

    private IntakeRollerStates(ControlRequest request) {
      this.REQUEST = request;
    }
  }

  private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreak1ID);

  private TalonFX intakeArmPrimary = new TalonFX(Constants.IntakeConstants.intakeArmPrimaryID, "rio");
  private TalonFX intakeArmFollower = new TalonFX(Constants.IntakeConstants.intakeArmFollowerID, "rio");


  private TalonFX intakeWheelTop = new TalonFX(Constants.IntakeConstants.intakeWheelTopID, "rio");
  private TalonFX intakeWheelBottom = new TalonFX(Constants.IntakeConstants.intakeWheelBottomID, "rio");

  /* Constructor */
  public IntakeSubsystem() {
    intakeArmPrimary.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);
    intakeArmFollower.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);

    intakeWheelBottom.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);
    intakeWheelTop.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);

    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));
    intakeWheelBottom.setControl(new Follower(IntakeConstants.intakeWheelTopID, false));

    // intakeArmRoot.append(intakeArmLigament);
    
    if (StateHandler.getInstance().isAngleRPMTuning){

    MotorPIDFVAJWidget intakeTuning = new MotorPIDFVAJWidget("INTAKE", 
                                    IntakeConstants.ARM_CONFIGS, 
                                    0, 
                                    360.0, 
                                    0, 
                                    IntakeConstants.intakePositionAllowableOffset, 
                                    IntakeArmStates.TUNING.REQUEST,
                                    new SimpleIntakeArmStateSwapCommand(IntakeArmStates.TUNING, IntakeArmStates.STOWED),
                                    intakeArmPrimary, intakeArmFollower);
    }

    zeroIntakeArm();
  }

  /**
   * Method to get the current intake arm's position (in degrees)
   * @return a double value representing the intake's position in degrees
   */
  public double getIntakeArmPositionDegrees() {
    // return intakeArmPrimary.getPosition().getValueAsDouble() * IntakeConstants.intakeRotsToDegrees;
    return Units.rotationsToDegrees(intakeArmPrimary.getPosition().getValueAsDouble());

  }

  /**
   * Method to determine if the intake has reached its desired position.
   * @param state the state specifying the desired position
   * @return a boolean representing if the intake arm has reached its desired position
   */
  public boolean isAtIntakeArmState(IntakeArmStates state) {

    if (Utils.isSimulation()) return true;


    if (state.REQUEST instanceof MotionMagicVoltage){
      // double desiredPosition = ((MotionMagicVoltage) state.REQUEST).Position
      //     * IntakeConstants.intakeRotsToDegrees;
       double desiredPositionDegrees = Units.rotationsToDegrees(((MotionMagicVoltage) state.REQUEST).Position);
          

      return Math.abs(getIntakeArmPositionDegrees() - desiredPositionDegrees) < IntakeConstants.intakePositionAllowableOffset;
    }

    return true;
      

  }

  /**
   * Method to determine if the intake roller wheels have reached their desired setpoint.
   * @param state the desired RollerState
   * @return true (always)
   */
  public boolean isAtRollerVelocity(IntakeRollerStates state) {
    return true;
  }


  /**
   * Method to set the intake arm to a specific ControlRequest
   * @param output the specified desired ControlRequest.
   */
  public void setIntakeArmTo(ControlRequest output) {
    intakeArmPrimary.setControl(output);
  }

  /**
   * Method to set the intake's rollers to a specific ControlRequest.
   * @param output the specified desired ControlRequest.
   */
  public void setRollerSpeedTo(ControlRequest output) {
    intakeWheelTop.setControl(output);
  }

  /**
   * Method to stop the intake arm's motors.
   */
  public void stopIntakeArmMotors() {
    intakeArmPrimary.stopMotor();
  }

  /**
   * Method to stop the intake wheels.
   */
  public void stopIntakeWheels() {
    intakeWheelTop.stopMotor();
  }

  /**
   * Method to zero the intake arm.
   */
  public void zeroIntakeArm() {
    intakeArmPrimary.setPosition(0);
    intakeArmFollower.setPosition(0);
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()){
      StateHandler.getInstance().bb1Covered = !beamBreakOne.get();
    }

   

    SmartDashboard.putNumber("Intake Primary Position", getIntakeArmPositionDegrees());
    // SmartDashboard.putNumber("Intake Follower Position", intakeArmFollower.getPosition().getValueAsDouble() * IntakeConstants.intakeRotsToDegrees);

  }

}
