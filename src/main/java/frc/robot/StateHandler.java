package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ControllerConstants.Driver;
import frc.robot.lib.simulation.SimulationUtils;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerStates;
import frc.robot.subsystems.ShooterSubsystem.BlowerStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class StateHandler {
    private static StateHandler stateHandler;

    private StateHandler(){
        System.out.println("StateHandler just got created");
    }

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }

        return stateHandler;
    }

    public static enum ScoringType {
        AMP,
        SUBWOOFER,
        REVERSE_SUBWOOFER,
        RANGED,
        TRAP,
        HIGH_PUNT,
        LOW_PUNT,
        UNGUARDABLE,
        CLIMB,
        TUNING
    }

    public ScoringType scoringType = ScoringType.RANGED;
    
    /* DESIRED STATES: These tell mechanisms to "go" to the state specified. */
    public ShooterStates desiredShooterState = ShooterStates.IDLE_VELO;
    public IntakeArmStates desiredIntakeArmState = IntakeArmStates.STOWED;
    public IntakeRollerStates desiredIntakeRollerState = IntakeRollerStates.OFF;
    public ArmStates desiredArmState = ArmStates.STOWED;
    public FeederStates desiredFeederState = FeederStates.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterStates currentShooterState = ShooterStates.IDLE_VELO;
    public IntakeArmStates currentIntakeArmState = IntakeArmStates.STOWED;
    public IntakeRollerStates currentIntakeRollerState = IntakeRollerStates.OFF;
    public ArmStates currentArmState = ArmStates.STOWED;
    public FeederStates currentFeederState = FeederStates.OFF;
    public BlowerStates blowerState = BlowerStates.OFF;


    /* SWERVE STATES - ONLY CURRENT STATE IS REQUIRED */
    public SwerveStates swerveState = SwerveStates.FIELD_CENTRIC;
    public Pose2d swervePose = new Pose2d();

    /* BEAM BREAK Values */
    public boolean bb1Covered = false;
    public boolean bb2Covered = false;
    public boolean bb3Covered = false;
    public boolean bb4Covered = false;

    public boolean latchingBB = false;

    public boolean hasGamePiece(){
        return bb3Covered;
    }


    /* Vision Values */

    public boolean hasSpeakerTag(){
        if (Utils.isSimulation()) return true;

        return (LimelightHelpers.getFiducialID(LimelightConstants.limelightName) == 7 || LimelightHelpers.getFiducialID(LimelightConstants.limelightName) == 4);
    }

    /**
     * Gets the horizontal angle offset from the tag
     * @return limelight tx or -1 if no tag
     */
    public double llTx(){
        if (Utils.isSimulation()) return -SimulationUtils.simLLAngleToSpeaker(swervePose);

        return (LimelightHelpers.getFiducialID(LimelightConstants.limelightName) == -1) ? 0 : LimelightHelpers.getTX(LimelightConstants.limelightName);
    }

    /**
     * Gets the vertical angle offset from the tag
     * @return limelight ty or -1 if no tag
     */
    public double llTy(){
        return (LimelightHelpers.getFiducialID(LimelightConstants.limelightName) == -1) ? 0 : LimelightHelpers.getTY(LimelightConstants.limelightName);
    }

    public boolean isCenteredToSpeakerTag(){
        return hasSpeakerTag() && Math.abs(llTx()) < LimelightConstants.centeredTolerance;
    }

    public boolean isInSpeakerRange(){
        if (Utils.isSimulation()) return true;

        return InterpolationConstants.tyToDistanceMap.get(Integer.MIN_VALUE * 1.0) <= speakerDistance() && speakerDistance() <= InterpolationConstants.tyToDistanceMap.get(Integer.MAX_VALUE * 1.0);
    }

    public double speakerDistance(){
        if (Utils.isSimulation()) return SimulationUtils.getDistFromRobotToPose(swervePose, (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? Constants.FieldConstants.blueSubwooferPos : FieldConstants.redSubwooferPos);
        return hasSpeakerTag() ? InterpolationConstants.tyToDistanceMap.get(llTy()) : -1;
    }



    

    /* MISC */
    public final boolean isAngleRPMTuning = false;

}
