// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  public enum FeederStates {
    OFF(new DutyCycleOut(0).withEnableFOC(true)),
    FEED_TO_SHOOTER(new DutyCycleOut(0.85).withEnableFOC(true)), // intake -> shooter
    FEED_TO_INTAKE(new DutyCycleOut(-0.8).withEnableFOC(true)), // shooter -> intake
    FRONTING(new DutyCycleOut(0.1).withEnableFOC(true)),
    BACKING(new DutyCycleOut(-0.1).withEnableFOC(true)),
    FULL_EJECT(new DutyCycleOut(0.85).withEnableFOC(true));

    public ControlRequest REQUEST;

    private FeederStates(ControlRequest request) {
      REQUEST = request;
    }
  }

  private TalonFX feederMotor = new TalonFX(FeederConstants.feederID, "rio");

  private final FlywheelSim feederSimModel = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1, FeederConstants.feederMomentOfInertia);

  /*
   * Beam Break initializations. These are DigitalInput objects that return
   * true/false.
   */
  private DigitalInput beamBreakTwo = new DigitalInput(FeederConstants.beamBreak2ID);
  private DigitalInput beamBreakThree = new DigitalInput(FeederConstants.beamBreak3ID);

  /* CONFIGURATION METHODS */
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {

    feederMotor.getConfigurator().apply(FeederConstants.CONFIGS);
  }

  /* FUNCTIONAL METHODS */

  public void setFeederTo(ControlRequest output) {
    feederMotor.setControl(output);
  }

  /* INFO METHODS */
  public double getFeederPercent() {
    return feederMotor.get();
  }

  public boolean isAtState(FeederStates state) {
    if (Utils.isSimulation()) return true;

    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!Utils.isSimulation()){
      StateHandler.getInstance().bb2Covered = !beamBreakTwo.get();
      StateHandler.getInstance().bb3Covered = !beamBreakThree.get();
    }
    
  }

  @Override
  public void simulationPeriodic(){
    TalonFXSimState feederSimState = feederMotor.getSimState();


    feederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    feederSimModel.setInputVoltage(feederSimState.getMotorVoltage());
    feederSimModel.update(0.020);

    feederSimState.setRotorVelocity(ArmConstants.armGearRatio * Units.radiansToRotations(feederSimModel.getAngularVelocityRadPerSec()));

  }
}
