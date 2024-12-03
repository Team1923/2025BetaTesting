package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;


import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateHandler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateHandler.ScoringType;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class InfoSubsystem extends SubsystemBase {
  /** Creates a new ShuffleboardSubsystem. */

  public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
  public ShuffleboardTab stateDashboard = Shuffleboard.getTab("State Dashboard");
  private StateHandler stateHandler = StateHandler.getInstance();

  private CommandXboxController xboxController;
  private CommandPS5Controller ps4Controller;

  
  public InfoSubsystem(CommandXboxController x, CommandPS5Controller p){
    this.xboxController = x;
    this.ps4Controller = p;

    if (stateHandler.isAngleRPMTuning){

      driverDashboard.addBoolean("IN POSRPM MODE", () -> false)
      .withSize(3, 3)
      .withPosition(0, 0)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#57F542"));
    }
  }

	private GenericEntry ampPos = driverDashboard.add("AMP", false)
			.withSize(3, 1)
			.withPosition(0, 0)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#F59542"))
			.getEntry();

	private GenericEntry subwooferPos = driverDashboard.add("SUBWOOFER", false)
			.withSize(3, 1)
			.withPosition(0, 1)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#CE42F5"))
			.getEntry();
  
  private GenericEntry reverseSubwooferPos = driverDashboard.add("REVERSE_SUBWOOFER", false)
  		.withSize(3, 1)
			.withPosition(0, 2)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#000080"))
			.getEntry();
    private GenericEntry climbing = driverDashboard.add("Climbing", false)
  		.withSize(3, 1)
			.withPosition(2, 1)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#000080"))
			.getEntry();

  private GenericEntry unGuardablePos = driverDashboard.add("UNGUARDABLE", false)
      .withSize(3, 1)
      .withPosition(2, 2)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#FFFF00"))
      .getEntry();

	private GenericEntry seeSpeakerTag = driverDashboard.add("SPEAKER APRIL TAG", false)
			.withSize(3, 1)
			.withPosition(0, 3)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#57F542"))
			.getEntry();


  private SuppliedValueWidget<Double> distToSpeaker = driverDashboard.addNumber("DIST TO SPEAKER TAG", () -> stateHandler.speakerDistance()).withPosition(4, 3);


  private SuppliedValueWidget<String> currentArmPosition = stateDashboard.addString("Current Arm Position", () -> stateHandler.currentArmState.toString()).withPosition(0, 0);
  private SuppliedValueWidget<String> desiredArmPosition = stateDashboard.addString("Desired Arm Position", () -> stateHandler.desiredArmState.toString()).withPosition(0, 1);
  private SuppliedValueWidget<String> currentIntakeWheelSpeeds = stateDashboard.addString("Current Intake Wheel Speeds", () -> stateHandler.currentIntakeRollerState.toString()).withPosition(1, 0);
  private SuppliedValueWidget<String> desiredIntakeWheelSpeeds = stateDashboard.addString("Desired Intake Wheel Speeds", () -> stateHandler.desiredIntakeRollerState.toString()).withPosition(1, 1);
  private SuppliedValueWidget<String> currentIntakePosition = stateDashboard.addString("Current Intake Position", () -> stateHandler.currentIntakeArmState.toString()).withPosition(2, 0);
  private SuppliedValueWidget<String> desiredIntakePosition = stateDashboard.addString("Desired Intake Position", () -> stateHandler.desiredIntakeArmState.toString()).withPosition(2, 1);
  private SuppliedValueWidget<String> currentShooterSpeeds = stateDashboard.addString("Current Shooter Speed", () -> stateHandler.currentShooterState.toString()).withPosition(3, 0);
  private SuppliedValueWidget<String> desiredShooterSpeeds = stateDashboard.addString("Desired Shooter Speeds", () -> stateHandler.desiredShooterState.toString()).withPosition(3, 1);
  private SuppliedValueWidget<String> currentFeederSpeed = stateDashboard.addString("Current Feeder Speed", () -> stateHandler.currentFeederState.toString()).withPosition(4, 0);
  private SuppliedValueWidget<String> desiredFeederSpeed = stateDashboard.addString("Desired Feeder Speed", () -> stateHandler.desiredFeederState.toString()).withPosition(4, 1);
  private SuppliedValueWidget<String> swerveState = stateDashboard.addString("Swerve State", () -> stateHandler.swerveState.toString()).withPosition(4, 3);

  private SuppliedValueWidget<Boolean> bb1 = stateDashboard.addBoolean("BB ONE COVERED", () -> stateHandler.bb1Covered).withPosition(0, 2);
  private SuppliedValueWidget<Boolean> bb2 = stateDashboard.addBoolean("BB TWO COVERED", () -> stateHandler.bb2Covered).withPosition(1, 2);;
  private SuppliedValueWidget<Boolean> bb3 = stateDashboard.addBoolean("BB THREE COVERED", () -> stateHandler.bb3Covered).withPosition(2, 2);;
  private SuppliedValueWidget<Boolean> bb4 = stateDashboard.addBoolean("BB FOUR COVERED", () -> stateHandler.bb4Covered).withPosition(3, 2);;
  // private SuppliedValueWidget<Boolean> bb1Dead = stateDashboard.addBoolean("BB ONE DEAD", () -> stateHandler.getBB1Dead()).withPosition(0, 3);;

  private SuppliedValueWidget<Boolean> centeredToTag = stateDashboard.addBoolean("Centered To Speaker Tag", () -> stateHandler.isCenteredToSpeakerTag()).withPosition(4, 3);

  private SuppliedValueWidget<Double> positionData = stateDashboard.addDouble("Arm Data (Speaker)", () -> InterpolationConstants.distanceToAngle.get(stateHandler.speakerDistance())).withPosition(6, 3);
    private SuppliedValueWidget<Double> rpmData = stateDashboard.addDouble("RPM Data (Speaker)", () -> InterpolationConstants.distanceToRPM.get(stateHandler.speakerDistance())).withPosition(7, 3);




  
  

  @Override
  public void periodic() {
    /* Driver Dashboard Display */
    subwooferPos.setBoolean(stateHandler.scoringType == ScoringType.SUBWOOFER);
    ampPos.setBoolean(stateHandler.scoringType == ScoringType.AMP);
    reverseSubwooferPos.setBoolean(stateHandler.scoringType == ScoringType.REVERSE_SUBWOOFER);
    seeSpeakerTag.setBoolean(stateHandler.hasSpeakerTag());
    climbing.setBoolean(stateHandler.scoringType == ScoringType.CLIMB);
    unGuardablePos.setBoolean(stateHandler.scoringType == ScoringType.UNGUARDABLE);

    if(DriverStation.isTeleop() && (stateHandler.bb1Covered || (stateHandler.bb4Covered && stateHandler.currentArmState == ArmStates.BABY_BIRD)) /*&& !stateHandler.getBB1Dead()*/){ // add condition for non functional bb1 
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
      ps4Controller.getHID().setRumble(RumbleType.kBothRumble, 0.4);
    } else{
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
      ps4Controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    

    /* DEBUG PRINTOUTS - TODO: DISABLE WHEN IN MATCH! */
    /* BEAM BREAK VALUES */
    // SmartDashboard.putBoolean("BB ONE COVERED", stateHandler.getBBOneCovered());
    // SmartDashboard.putBoolean("BB TWO COVERED", stateHandler.getBBTwoCovered());
    // SmartDashboard.putBoolean("BB THREE COVERED", stateHandler.getBBThreeCovered());
    // SmartDashboard.putBoolean("BB FOUR COVERED", stateHandler.getBBFourCovered());

    // /* RELEVANT INTAKE STATES */
    // SmartDashboard.putString("CURRENT INTAKE ROLLER", stateHandler.getCurrentIntakeRollerSpeed().toString());
    // SmartDashboard.putString("DESIRED INTAKE POS", stateHandler.getDesiredIntakeState().toString());
    // SmartDashboard.putString("CURRENT INTAKE POS", stateHandler.getCurrentIntakeState().toString());

    /* RELEVANT ARM STATES */
    // SmartDashboard.putString("DESIRED ARM STATE", stateHandler.getDesiredArmState().toString());
    // SmartDashboard.putString("CURRENT ARM STATE", stateHandler.getCurrentArmState().toString());

    // // /* RELEVANT SHOOTER STATES */
    // SmartDashboard.putString("DESIRED SHOOTER STATE", stateHandler.getDesiredShootingSpeed().toString());
    // SmartDashboard.putString("CURRENT SHOOTER STATE", stateHandler.getCurrentShootingSpeed().toString());

    // /* RELEVANT FEEDER STATES */
    // SmartDashboard.putString("CURRENT FEEDER DIRECTION", stateHandler.getCurrentFeederSpeed().toString());



    // SmartDashboard.putBoolean("AUTO OVERRIDE", stateHandler.getAutoOverride());

    // SmartDashboard.putNumber("Tuning Angle", ((MotionMagicTorqueCurrentFOC)(ArmStates.ANGLE_TUNING.REQUEST)).Position * ArmConstants.armRotsToDegrees);
    // SmartDashboard.putNumber("Tuning RPM", ((MotionMagicVelocityTorqueCurrentFOC)(ShooterStates.RPM_TUNING.REQUEST_TOP)).Velocity * ShooterConstants.RPSToRPM);
    
// 

  }
}