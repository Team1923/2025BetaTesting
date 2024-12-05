// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.misc.SimpleShooterStateSwapCommand;
import frc.robot.lib.tuningwidgets.MotorPIDFVAJWidget;

public class ShooterSubsystem extends SubsystemBase {

  public static enum ShooterStates {
    IDLE_VELO(MMVeloVoltage(0)),
    BABY_BIRD_VELO(MMVeloVoltage(-1000)),
    FRONT_AMP_VELO(MMVeloVoltage(415)),
    UNGUARDABLE_VELO(MMVeloVoltage(1190), MMVeloVoltage(1905)),
    TRAP_VELO(MMVeloVoltage(650), MMVeloVoltage(1400)),
    PUNT_HIGH_VELO(MMVeloVoltage(2100)),
    PUNT_LOW_DUTY(new DutyCycleOut(1).withEnableFOC(true), new DutyCycleOut(0.7).withEnableFOC(true)),
    SUBWOOFER_VELO(MMVeloVoltage(2700)),
    REVERSE_SUBWOOFER_VELO(MMVeloVoltage(2500)),
    RANGED_VELO(MMVeloVoltage(1000)),
    FULL_EJECT_DUTY(new DutyCycleOut(1).withEnableFOC(true)),
    RPM_TUNING(new MotionMagicVelocityVoltage(0).withEnableFOC(true));

    public ControlRequest REQUEST_TOP;
    public ControlRequest REQUEST_BOTTOM;

    private ShooterStates(ControlRequest request) {
      this.REQUEST_TOP = request;
      this.REQUEST_BOTTOM = request;
    }

    private ShooterStates(ControlRequest REQUEST_TOP, ControlRequest REQUEST_BOTTOM) {
      this.REQUEST_TOP = REQUEST_TOP;
      this.REQUEST_BOTTOM = REQUEST_BOTTOM;
    }

    private static MotionMagicVelocityVoltage MMVeloVoltage(double RPM) {
      return new MotionMagicVelocityVoltage(RPM * ShooterConstants.RPMToRPS).withEnableFOC(true);
    }
  }

  public static enum BlowerStates{
    ON(1),
    OFF(0);

    public double PERCENT;
    
    private BlowerStates(double percent){
      PERCENT = percent;
    }
  }

  /* Initialize Shooter Motors */
  private TalonFX shooterTop = new TalonFX(ShooterConstants.shooterTopID, "rio");
  private TalonFX shooterBottom = new TalonFX(ShooterConstants.shooterBottomID, "rio");



  /* Initialize TalonSRX used for blower motor */
  private TalonSRX blower = new TalonSRX(ShooterConstants.blowerID);

  /* Initialize DigitalInput object for BB4 */
  private DigitalInput beamBreak4 = new DigitalInput(ShooterConstants.beamBreak4ID);


  public ShooterSubsystem() {
    /* Configure the Shooter Motors */
    shooterTop.getConfigurator().apply(ShooterConstants.CONFIGS);
    shooterBottom.getConfigurator().apply(ShooterConstants.CONFIGS);

    /* Apply a Default Configuration to the Blower Motor */
    blower.configFactoryDefault();

    if (StateHandler.getInstance().isAngleRPMTuning){

    MotorPIDFVAJWidget shooterTuner = new MotorPIDFVAJWidget(
                                                "SHOOTER", 
                                                ShooterConstants.CONFIGS,
                                                0, 
                                                0, 
                                                ShooterConstants.RPSToRPM, 
                                                ShooterConstants.shooterRPMThreshhold, 
                                                ShooterStates.RPM_TUNING.REQUEST_TOP, ShooterStates.RPM_TUNING.REQUEST_BOTTOM,
                                                new SimpleShooterStateSwapCommand(ShooterStates.RPM_TUNING, ShooterStates.IDLE_VELO),
                                                shooterTop, shooterBottom);
    }
  }

  /**
   * Method to set both shooter motors to a specified RPM.
   * @param output a Phoenix6 ControlRequest object.
   */
  public void setShooterMotorsTo(ControlRequest output) {
    shooterTop.setControl(output);
    shooterBottom.setControl(output);
  }

  /**
   * Method to set the shooter motor's velocities independently.
   * @param topOutput a Phoenix6 ControlRequest object for the top motor.
   * @param bottomOutput a Phoenix6 ControlRequest object for the bottom motor.
   */
  public void setShooterMotorsTo(ControlRequest topOutput, ControlRequest bottomOutput) {
    shooterTop.setControl(topOutput);
    shooterBottom.setControl(bottomOutput);

  }

  
  /**
   * Method to set the Blower Motor to a particular percent output.
   * @param percent the specified percent output for the blower.
   */
  public void setBlowerTo(double percent) {
    blower.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Method to get the top shooter's velocity.
   * @return a double value of the top motor's velocity.
   */
  public double getTopRPM() {
    return shooterTop.getVelocity().getValueAsDouble() * ShooterConstants.RPSToRPM;
  }

  /**
   * Method to get the bottom shooter's velocity.
   * @return a double value of the bottom motor's velocity.
   */
  public double getBottomRPM() {
    return shooterBottom.getVelocity().getValueAsDouble() * ShooterConstants.RPSToRPM;
  }

  /**
   * Method to determine if the shooter has reached the desired state specified.
   * @param state the desired state to evaluate.
   * @return a boolean representing whether or not the desired state has been reached.
   */
  public boolean isAtState(ShooterStates state) {

    // if (Utils.isSimulation()) return true;


    if (state.REQUEST_TOP instanceof MotionMagicVelocityVoltage) {

      /* Get the shooter's velocities (in RPM) */
      double desiredVelocityTop = ((MotionMagicVelocityVoltage) state.REQUEST_TOP).Velocity * ShooterConstants.RPSToRPM;
      double desiredVelocityBottom = ((MotionMagicVelocityVoltage) state.REQUEST_BOTTOM).Velocity * ShooterConstants.RPSToRPM;

      /* Check if the shooter's RPM is within the RPM tolerance (25 RPM). */
      return Math.abs(getTopRPM() - desiredVelocityTop) < ShooterConstants.shooterRPMThreshhold
          && Math.abs(getBottomRPM() - desiredVelocityBottom) < ShooterConstants.shooterRPMThreshhold;
    } else {
      /* DEFAULT: percent output is used. */
      return true;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* Update BB4's value. */

    if (!Utils.isSimulation()){
      StateHandler.getInstance().bb4Covered = !beamBreak4.get();

    }

    SmartDashboard.putNumber("Shooter Top", getTopRPM());
    SmartDashboard.putNumber("Shooter Bottom", getBottomRPM());

    // TODO: ((MotionMagicVelocityVoltage)(States.RANGED.OUTPUT)).Velocity = updated
    // value;
  }
  
}
