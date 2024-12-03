// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.misc.SimpleArmStateSwapCommand;
import frc.robot.lib.tuningwidgets.MotorPIDFVAJWidget;

public class ArmSubsystem extends SubsystemBase {

  public static enum ArmStates {


    STOWED(MMVoltage(0)),
    REVERSE_SUBWOOFER(MMVoltage(-113)),
    UNGUARDABLE(MMVoltage(-108.5)),
    // AMP(MMVoltageWithDegrees(-108.5)),
    SUBWOOFER(MMVoltage(-46.1)), 
    RANGED(MMVoltage(0), 1),
    TRAP(MMVoltage(-51.6), 0.5),
    BABY_BIRD(MMVoltage(-40.1)), //-40.1
    PUNT_HIGH(MMVoltage(-38.4)),
    PUNT_LOW(MMVoltage(0)),
    FRONT_AMP(MMVoltage(-44.1)),
    DEFENSE(MMVoltage(-77.3)),
    CLIMB(MMVoltage(-77.3)),
    ZEROING(new DutyCycleOut(0.05).withEnableFOC(true)),
    ANGLE_TUNING(MMVoltage(0), 1), 
    OFF(new DutyCycleOut(0).withEnableFOC(true));


    public ControlRequest REQUEST;
    public double settleTime;

    private ArmStates(ControlRequest request) {
      this.REQUEST = request;
      this.settleTime = 0;
    }

    private ArmStates(ControlRequest request, double settleTime) {
      this.REQUEST = request;
      this.settleTime = settleTime;
    }

    private static MotionMagicVoltage MMVoltage(double degrees) {
      // return new MotionMagicVoltage(degrees * ArmConstants.armDegreesToRots);
      return new MotionMagicVoltage(Units.degreesToRotations(degrees)).withEnableFOC(true);
    }

  }

  /* Initialize arm motors */
  private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID, "rio");
  private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID, "rio");

  public final DCMotorSim armSimModel = new DCMotorSim(DCMotor.getKrakenX60Foc(2), ArmConstants.armGearRatio, ArmConstants.armMomentOfInertia);



  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPrimary.getConfigurator().apply(ArmConstants.CONFIGS);
    armFollower.getConfigurator().apply(ArmConstants.CONFIGS);


    armFollower.setControl(new Follower(ArmConstants.armMotorPrimaryID, true));

    if (StateHandler.getInstance().isAngleRPMTuning){
      MotorPIDFVAJWidget armTuning = new MotorPIDFVAJWidget("ARM", 
                                          ArmConstants.CONFIGS, 
                                          0, 
                                          360, 
                                          1, 
                                          ArmConstants.armPositionAllowableOffset, 
                                          ArmStates.ANGLE_TUNING.REQUEST,
                                          new SimpleArmStateSwapCommand(ArmStates.ANGLE_TUNING, ArmStates.STOWED),
                                          armPrimary, armFollower);
    }
    

    zeroArm();

  }

  /**
   * Method to set the arm position.
   * 
   * @param output the specified ControlRequest to output the motor at.
   */
  public void setArmTo(ControlRequest output) {
    armPrimary.setControl(output);
    // System.out.println("request :" + output.getControlInfo().toString());
  }

  /**
   * Move the arm using percent output. Primarily used for testing purposes.
   * 
   * @param out percent out speed to run the arm at
   */
  public void setPercentOut(double out) {
    armPrimary.setControl(new DutyCycleOut(out));
  }

  /**
   * Get the position of the arm, converted from the encoder reading.
   * 
   * @return The arm position in degrees.
   */
  public double getArmPositionDegrees() {
    // return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToDegrees;
    return Units.rotationsToDegrees(armPrimary.getPosition().getValueAsDouble());

  }


  /**
   * Gets the position of the arm directly from the encoder reading.
   * 
   * @return The arm position in rotations.
   */
  public double getArmPositionRots() {
    return armPrimary.getPosition().getValueAsDouble();
  }


  public double getPercentOutput() {
    return armPrimary.get();
  }
  // public StatusSignal<Double> getArmVoltage(){
  //   return armPrimary.getMotorVoltage().getValue();
  // }

  /**
   * Gets the supply current of the arm primary motor
   * @return arm supply current in Amps
   */
  public double getArmSupplyCurrent(){
    return armPrimary.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Method to determine if the intake has reached its desired position.
   * 
   * @param state the state specifying the desired positionMotionMagicVoltage
   * @return a boolean representing if the intake arm has reached its desired
   *         position
   */
  public boolean isAtState(ArmStates state) {

    //if (Utils.isSimulation()) return true;

    if (state.REQUEST instanceof MotionMagicVoltage){
      // double desiredPosition = ((MotionMagicVoltage) state.REQUEST).Position * ArmConstants.armRotsToDegrees;
      double desiredPositionDegrees = Units.rotationsToDegrees(((MotionMagicVoltage) state.REQUEST).Position);

      if (state == ArmStates.STOWED){
        return Math.abs(getArmPositionDegrees() - desiredPositionDegrees) < 1;
      }

      return Math.abs(getArmPositionDegrees() - desiredPositionDegrees) < ArmConstants.armPositionAllowableOffset;
    }
    else{
      return true;
    }
    

  }

  /**
   * Method to stop the arm's motors.
   */
  public void stopArmMotors() {
    armPrimary.stopMotor();
  }

  /**
   * Method to zero the arm.
   */
  public void zeroArm() {
    armPrimary.setPosition(0);
    armFollower.setPosition(0);
  }

//  public void setCoast() {
//   armPrimary.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
//  }

//  public void setBrake() {
//   armPrimary.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
//  }



  @Override
  public void periodic() {

    // SmartDashboard.putString("PRIMARY MODE", armPrimary.getAppliedControl().getName());
    //     SmartDashboard.putString("FOLLOWER MODE", armFollower.getAppliedControl().getName());
    SmartDashboard.putNumber("ArmPostion", getArmPositionDegrees());

    

    

  }


  @Override
  public void simulationPeriodic(){
    TalonFXSimState armPrimarySimState = armPrimary.getSimState();


    armPrimarySimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    

    armSimModel.setInputVoltage(addFriction(armPrimarySimState.getMotorVoltage(), ArmConstants.armSimFriction));
    armSimModel.update(0.020);

    armPrimarySimState.setRawRotorPosition(ArmConstants.armGearRatio * armSimModel.getAngularPositionRotations());
    armPrimarySimState.setRotorVelocity(ArmConstants.armGearRatio * Units.radiansToRotations(armSimModel.getAngularVelocityRadPerSec()));

  }

  //yoinked from swerve module sim
  private double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
        motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
        motorVoltage -= frictionVoltage;
    } else {
        motorVoltage += frictionVoltage;
    }
    return motorVoltage;
}
}
