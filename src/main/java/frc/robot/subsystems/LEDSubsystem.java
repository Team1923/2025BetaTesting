// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateHandler;
import frc.robot.StateHandler.ScoringType;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;


public class LEDSubsystem extends SubsystemBase {
  /** Creates a new CANdleSubsystem. */

  private StateHandler stateHandler = StateHandler.getInstance();

  private int LEDCount = Constants.LEDConstants.LEDCount;

  private Colors currentColor = Colors.OFF;

  private Animations currentAnimation = Animations.OFF;

  private CANdle candle = new CANdle(Constants.LEDConstants.CANdleID);

  public enum Colors {
    RED(new int[] { 255, 0, 0 }),
    GREEN(new int[] { 0, 255, 0 }),
    BLUE(new int[] { 0, 70, 255 }),
    PURPLE(new int[] { 90, 0, 255 }),
    PINK(new int[] { 255, 40, 200 }),
    ORANGE(new int[] { 255, 28, 0 }),
    YELLOW(new int[] { 255, 120, 0 }),
    WHITE(new int[] { 255, 255, 255 }),
    PUKE_GREEN(new int[] {160,176, 12}),
    RAINBOW(new int[] { 0, 0, 0 }),
    OFF(new int[] { 0, 0, 0 }),
    TEST(new int[] { 0, 0, 0 });

    public int[] RGB;

    private Colors(int[] c) {
      RGB = c;
    }
  }

  public enum Animations {
    HEARTBEAT,
    OFF,
    SOLID,
    FLASHING,
    FIRE,
    LARSON,
    RAINBOW;

    public Animation anim;

  }

  public LEDSubsystem() {

    CANdleConfiguration config = new CANdleConfiguration();

    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.brightnessScalar = 1;
    candle.configAllSettings(config);

    // waitTimer.start();
  }

  public void apply(Colors c, Animations a) {
    int r = c.RGB[0];
    int g = c.RGB[1];
    int b = c.RGB[2];

    endAnimation();
    candle.clearAnimation(0);

    // trying useing animation slot 0 alwasy, maybe overrunning?

    switch (a) {
      case SOLID:
        candle.setLEDs(r, g, b, 0, 0, LEDCount);
        // candle.animate(new SingleFadeAnimation(r,g , b,0,0, LEDCount),0);
        break;
      case RAINBOW:
        candle.animate(new RainbowAnimation(), 0);
        break;
      case HEARTBEAT:
        candle.animate(new SingleFadeAnimation(r, g, b, 0, 0.7, LEDCount), 0);
        break;
      case FLASHING:
        candle.animate(new StrobeAnimation(r, g, b, 0, 0.3, LEDCount, 0), 0);
        break;
      case FIRE:
        candle.animate(new FireAnimation(1, 0.3, -1, 1, 1, false, 0));
        break;
      case LARSON:
        candle.animate(new LarsonAnimation(r, g, b, 0, 0.5, LEDCount, BounceMode.Front, 4), 0);
        break;
      default:
        candle.animate(new SingleFadeAnimation(0, 0, 0), 0);
        break;
    }

  }

  public void endAnimation() {
    candle.animate(null);
  }

  public void off() {
    candle.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Colors desiredColor = null;
    Animations desiredAnimation = null;

    boolean hasNote = stateHandler.hasGamePiece();

    // No Code

    // Has Code
    // Button Coast
    // button Zero

    // No Code/Comms
    

    // regular
    if ((stateHandler.bb1Covered) || (stateHandler.desiredArmState == ArmStates.BABY_BIRD && stateHandler.bb4Covered)) { 

      desiredColor = Colors.WHITE;
      desiredAnimation = Animations.FLASHING;

    } else if (stateHandler.scoringType == ScoringType.CLIMB) {
      desiredColor = Colors.PINK;
      desiredAnimation = Animations.HEARTBEAT;
    } else if (stateHandler.currentIntakeArmState == IntakeArmStates.DEPLOYED) { // doesn't always go
      desiredColor = Colors.ORANGE;
      desiredAnimation = Animations.SOLID;
    }

    else if (hasNote) {

      
      switch (stateHandler.scoringType){
        case AMP:
           desiredColor = Colors.ORANGE;
          break;
        case CLIMB:
          desiredColor = Colors.PINK;
          break;
        case HIGH_PUNT:
          break;
        case LOW_PUNT:
          break;
        case RANGED:
          if (stateHandler.isInSpeakerRange()){
            desiredColor = Colors.RAINBOW;
            desiredAnimation = Animations.RAINBOW;
          }
          else{
            desiredColor = Colors.GREEN;
            desiredAnimation = Animations.HEARTBEAT;
          }
          break;
        case REVERSE_SUBWOOFER:
          desiredColor = Colors.PURPLE;
          break;
        case SUBWOOFER:
          desiredColor = Colors.BLUE;
          break;
        case TRAP:
          desiredColor = Colors.PUKE_GREEN;
          break;
        case UNGUARDABLE:
          desiredColor = Colors.YELLOW;
          break;
      }


        if (desiredColor == null){
          desiredColor = Colors.RED;
        }
        if (desiredAnimation == null){
          if (stateHandler.currentFeederState == FeederStates.FEED_TO_INTAKE){
          desiredAnimation = Animations.SOLID;
          }
          else{
              desiredAnimation = Animations.HEARTBEAT;

          }
        }
        
      } 
      else {
        desiredColor = Colors.RED;
        desiredAnimation = Animations.FIRE;
      }

    

    if ((desiredColor != null && desiredAnimation != null)
        && (currentAnimation != desiredAnimation || currentColor != desiredColor)) {
      apply(desiredColor, desiredAnimation);
      currentColor = desiredColor;
      currentAnimation = desiredAnimation;
    }
  }
}