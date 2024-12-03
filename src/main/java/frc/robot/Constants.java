package frc.robot;

import org.opencv.core.Point;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Constants {

        public static class ControllerConstants {
                public static class Driver {
                        public static final double deadband = 0.01;
                }

                public static class Operator {
                        public static final double deadband = 0.1;
                }
        }

        public static class ShooterConstants {
                /* Shooter Motor IDs */
                public static final int shooterTopID = 17;
                public static final int shooterBottomID = 18;

                /* Blower Motor ID */
                public static final int blowerID = 30;

                /* BB4 ID */
                public static final int beamBreak4ID = 4;

                /* Conversion Factors */
                public static final double RPSToRPM = 60;
                public static final double RPMToRPS = 1 / RPSToRPM;

                public static final double shooterMomentOfIntertia = 0.001206260649; //this still doesn't really function like the real shooter

                /* Parameters for the Shooter's Acceleration and Jerk */
                public static final double maxShooterAccel = 400; // Rotations/sec^2
                public static final double maxShooterJerk = 5000; // Rotations/sec^3

                /* TalonFX Motor Configuration for the Shooter */
                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                                .withSlot0(new Slot0Configs() // PID
                                                .withKP(0.39)
                                                .withKI(0)
                                                .withKD(0.000)
                                                .withKS(0.13)
                                                .withKA(0)
                                                .withKV(0.122))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicAcceleration(maxShooterAccel)
                                                .withMotionMagicJerk(maxShooterJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withTorqueCurrent(new TorqueCurrentConfigs()
                                                .withPeakForwardTorqueCurrent(80)
                                                .withPeakReverseTorqueCurrent(-80)
                                                .withTorqueNeutralDeadband(0.01));

                /* RPM Threshold for Current State Evaluation */
                public static final double shooterRPMThreshhold = 60
                ;


        }

        public static class IntakeConstants {
                /* Intake Arm Motor IDs */
                public static int intakeArmPrimaryID = 22;
                public static int intakeArmFollowerID = 21;

                /* Intake Wheels Motor IDs */
                public static int intakeWheelTopID = 25;
                public static int intakeWheelBottomID = 26;

                /* BB1 ID (port on RoboRIO) */
                public static int beamBreak1ID = 1;

                /* Parameters for the intake's motion */
                public static final double maxIntakeVel = 100;
                public static final double maxIntakeAccel = 50;
                public static final double maxIntakeJerk = 180;

                /* Gearbox ratios and unit conversions */
                public static final double intakeGearRatio = 60;
                public static final double intakeRotsToDegrees = 360 / intakeGearRatio;
                public static final double intakeDegreesToRotations = 1 / intakeRotsToDegrees;

                public static final double intakeMomentOfInertia = 0.0724166053;

                /* Define allowable offset for the intake arm's position */
                public static final double intakePositionAllowableOffset = 2.9;

                /* Arm Configuration - NOTE: clockwise positive is correct */
                public static final TalonFXConfiguration ARM_CONFIGS = new TalonFXConfiguration()
                                .withSlot0(new Slot0Configs() // PID
                                                .withKP(75)
                                                .withKI(0)
                                                .withKD(0)
                                                .withKS(0)
                                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                                                .withKV(0))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(maxIntakeVel)
                                                .withMotionMagicAcceleration(maxIntakeAccel)
                                                .withMotionMagicJerk(maxIntakeJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake) 
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withTorqueCurrent(new TorqueCurrentConfigs()
                                                .withPeakForwardTorqueCurrent(80)
                                                .withPeakReverseTorqueCurrent(-80))
                                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(intakeGearRatio));


                public static final TalonFXConfiguration WHEEL_CONFIGS = new TalonFXConfiguration();
        }

        public static class FeederConstants {
                public static final int feederID = 14;

                public static final int beamBreak2ID = 2;
                public static final int beamBreak3ID = 3;

                public static final double feederMomentOfInertia = 0.001255424111;

                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withTorqueCurrent(new TorqueCurrentConfigs()
                                                .withPeakForwardTorqueCurrent(80)
                                                .withPeakReverseTorqueCurrent(-80));
                
                public static final double timeout = 2; //seconds
        }

        public static class ArmConstants {
                /* Motor IDs */
                public static final int armMotorPrimaryID = 15; // right
                public static final int armMotorFollowerID = 16; // left

                /* Motion Magic Constants */
                // public static final double armKS = 0;
                // public static final double armkP = 1.2;
                // public static final double armkI = 0.005;
                // public static final double armkD = 0;

                //Faster motion magic 
                // public static final double maxArmVel = 300;
                // public static final double maxArmAccel = 175;
                // public static final double maxArmJerk = 1000;

                //Slower motion magic
                public static final double maxArmVel = 0.5;
                public static final double maxArmAccel = 0.34;
                public static final double maxArmJerk = 0;

                /* Gearbox Ratios & Unit Conversions */
                public static final double armGearRatio = 129.6;
                public static final double armRotsToDegrees = 360 / armGearRatio;
                public static final double armDegreesToRots = 1 / armRotsToDegrees;

                public static final double armMomentOfInertia = 0.5447340821;
                public static final double armSimFriction = 0.4;

                /* kG - gravity constant for motion of arm */
                //public static final double armMaxGravityConstant = 0.05 * 12; // 2 volts max ff

                public static final double armPositionAllowableOffset = 0.25;

                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()

                                .withSlot0(new Slot0Configs() // Fast PID
                                                .withKP(400)
                                                .withKI(0)
                                                .withKD(0)
                                                .withKS(0.4)
                                                .withKG(-0.4)
                                                .withGravityType(GravityTypeValue.Arm_Cosine))        
                                /* .withSlot1(new Slot1Configs() //Accurate PID
                                                .withKP(2)
                                                .withKI(0.1)
                                                .withKD(0)
                                                .withKS(0.09)
                                                // .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                                                .withKG(armMaxGravityConstant)
                                                .withGravityType(GravityTypeValue.Arm_Cosine))   */    
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(maxArmVel)
                                                .withMotionMagicAcceleration(maxArmAccel)
                                                .withMotionMagicJerk(maxArmJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withTorqueCurrent(new TorqueCurrentConfigs()
                                                .withPeakForwardTorqueCurrent(80)
                                                .withPeakReverseTorqueCurrent(-80))
                                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(armGearRatio));                



        }



        public static final class FieldConstants{
                public static final Pose2d blueSpeakerPos = new Pose2d(-0.038099999999999995, 5.547867999999999, Rotation2d.fromDegrees(180.0));
                public static final Pose2d redSpeakerPos = new Pose2d(16.579342, 5.547867999999999, Rotation2d.fromDegrees(0));

                public static final Pose2d blueSubwooferPos = new Pose2d(1.30, 5.57, Rotation2d.fromDegrees(180.0));
                public static final Pose2d redSubwooferPos = new Pose2d(15.20, 5.57, Rotation2d.fromDegrees(0));
                
                public static final Point blueSourceStart = new Point(14, 0);
                public static final Point blueSourceEnd = new Point(16.6, 1.7);
                
                public static final Point redSourceStart = new Point(0.135, 1.562);
                public static final Point redSourceEnd = new Point(1.732, 0.392);
        }

        public static final class LEDConstants{
                public static final int LEDCount = 41 + 8;
                public static final int CANdleID = 23;
        
        
        }

        public static final class LimelightConstants{
                public static final String limelightName = "limelight-shooter";

                public static final double centeredTolerance = 2;
        }

        public static final class InterpolationConstants {


                public static InterpolatingDoubleTreeMap tyToDistanceMap = new InterpolatingDoubleTreeMap();

                public static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
                public static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
                
                public InterpolationConstants(){
                        fillTyDistMap();
                        fillDistPosRPMMap();
                }
                
                public static void fillTyDistMap(){
                        // tyToDistanceMap.put(-14.97, 12.0);
                        // tyToDistanceMap.put(-11.99, 18.0);
                        // tyToDistanceMap.put(-9.36, 24.0);
                        // tyToDistanceMap.put(-7.03, 30.0);
                        // tyToDistanceMap.put(-4.64, 36.0);
                        // tyToDistanceMap.put(-2.72, 42.0);
                        // tyToDistanceMap.put(-0.80, 48.0);
                
                        // tyToDistanceMap.put(5.40, 72.0);
                        // tyToDistanceMap.put(6.53, 78.0);
                        // tyToDistanceMap.put(7.26, 84.0);
                        // tyToDistanceMap.put(7.93, 90.0);

                       //Lab Data
                        tyToDistanceMap.put(-7.26, 24.0);
                        tyToDistanceMap.put(-4.63,30.0);

                        //Havoc Data
                        tyToDistanceMap.put(-3.25, 36.0);
                        tyToDistanceMap.put(-1.45,42.0);
                        tyToDistanceMap.put(0.14, 48.0);
                        tyToDistanceMap.put(1.67, 54.0);
                        tyToDistanceMap.put(2.90, 60.0);
                        tyToDistanceMap.put(4.31, 66.0);
                        tyToDistanceMap.put(5.27, 72.0);
                        tyToDistanceMap.put(6.38, 78.0);
                        tyToDistanceMap.put(7.36, 84.0);
                        tyToDistanceMap.put(8.32, 90.0);
                        tyToDistanceMap.put(9.16, 96.0);
                        tyToDistanceMap.put(9.93, 102.0);
                        tyToDistanceMap.put(10.6, 108.0);
                        tyToDistanceMap.put(11.28, 114.0);


                        //Inconsistent Zone:
                        // tyToDistanceMap.put(12.31, 120.0);
                        // tyToDistanceMap.put(13.01, 126.0);
                        // tyToDistanceMap.put(13.59, 132.0);
                        // tyToDistanceMa.9p.put(15.03, 144.0);
                        // tyToDistanceMap.put(15.70 , 156.0); 
                        // tyToDistanceMap.put(16.41, 168.0);
                        // tyToDistanceMap.put(17.07, 180.0);
                        
                        



                }

                public static void fillDistPosRPMMap(){
                        distanceToAngle.put(24.0, -35.5); distanceToRPM.put(36.0,3800.0);                        
                        distanceToAngle.put(36.0, -31.0); distanceToRPM.put(36.0,3900.0);                        
                        distanceToAngle.put(48.0,-26.3); distanceToRPM.put(48.0, 4000.0);
                        distanceToAngle.put(54.0,-25.0); distanceToRPM.put(54.0, 4100.0);
                        distanceToAngle.put(60.0,-24.0); distanceToRPM.put(60.0, 4200.0);
                        distanceToAngle.put(66.0,-22.5); distanceToRPM.put(66.0, 4200.0);
                        distanceToAngle.put(72.0,-21.8); distanceToRPM.put(72.0, 4200.0);
                        distanceToAngle.put(78.0,-20.0); distanceToRPM.put(78.0, 4200.0);
                        distanceToAngle.put(84.0,-18.85); distanceToRPM.put(84.0, 4200.0);
                        distanceToAngle.put(90.0,-17.8); distanceToRPM.put(90.0, 4250.0);
                        distanceToAngle.put(96.0,-17.3); distanceToRPM.put(96.0, 4250.0);
                        distanceToAngle.put(102.0,-16.0); distanceToRPM.put(102.0, 4250.0);
                        distanceToAngle.put(108.0,-15.5); distanceToRPM.put(108.0, 4250.0);
                        distanceToAngle.put(114.0,-15.0); distanceToRPM.put(114.0, 4250.0);

                        // Inconsistent zone: 
                        // distanceToAngle.put(120.0,-14.05); distanceToRPM.put(120.0, 4200.0);
                        // distanceToAngle.put(126.0,-14.05); distanceToRPM.put(126.0,4300.0); // Better than others
                        // distanceToAngle.put(132.0,-13.35); distanceToRPM.put(132.0,4200.0);
                        // distanceToAngle.put(144.0, -11.8); distanceToRPM.put(144.0, 4330.0);
                        // distanceToAngle.put(156.0, -10.5); distanceToRPM.put(156.0, 4300.0);
                        // distanceToAngle.put(168.0, -10.3); distanceToRPM.put(168.0, 4500.0);
                        // distanceToAngle.put(180.0, -10.5); distanceToRPM.put(180.0, 4500.0);
                      

                }

        }


        public static class AutonConstants{
                public static final double ampBoundY = 7.75; //meters from wall
                public static final double sourceBoundY = 0.4; //meters from wall

                public static final double whiteLineBoundX = 8.28;
                public static final double whiteLineTolerance = 0.5; //meters over white line you are allowed to be, depending on alliance

                public static final double notefindingSpeed = 2; //m/s

        }

}
