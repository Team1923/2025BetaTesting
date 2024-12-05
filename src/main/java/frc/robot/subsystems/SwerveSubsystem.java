
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.StateHandler;
import frc.robot.lib.swerve.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {

    private static SwerveSubsystem swerve;

    public static synchronized SwerveSubsystem getInstance(){
        if (swerve == null){
            swerve = new SwerveSubsystem(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
            TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

            return swerve;
        }

        return swerve;
    }

    private StateHandler stateHandler = StateHandler.getInstance();

    public static enum SwerveStates{ 
        FIELD_CENTRIC(Default(new SwerveRequest.FieldCentric())),
        ROBOT_CENTRIC(Default(new SwerveRequest.RobotCentric())),
        GOAL_CENTRIC(withPID(Default(new SwerveRequest.FieldCentricFacingAngle()), new PhoenixPIDController(14, 0, 0.7))),
        FACING_AMP(withPID(Default(new SwerveRequest.FieldCentricFacingAngle()), new PhoenixPIDController(14, 0, 0.7))),
        FACING_TRAP(withPID(Default(new SwerveRequest.FieldCentricFacingAngle()), new PhoenixPIDController(14, 0, 0.7))),
        FACING_CLIMB(withPID(Default(new SwerveRequest.FieldCentricFacingAngle()), new PhoenixPIDController(14, 0, 0.7))),
        NOTEFIND(withPID(Default(new SwerveRequest.FieldCentricFacingAngle()), new PhoenixPIDController(14, 0, 0.7)));
        


        public SwerveRequest REQUEST;
        
        private SwerveStates(SwerveRequest request){
            REQUEST = request;
        }

        //TODO: TUNE DEADBANDS
        private static SwerveRequest.FieldCentric Default(SwerveRequest.FieldCentric FC){
            return FC.withDeadband(0).withRotationalDeadband(0);
        }
        private static SwerveRequest.RobotCentric Default(SwerveRequest.RobotCentric RC){
            return RC.withDeadband(0).withRotationalDeadband(0);
        }
        private static SwerveRequest.FieldCentricFacingAngle Default(SwerveRequest.FieldCentricFacingAngle FCA){ //this should probably have VERY low deadbands
            return FCA.withDeadband(0).withRotationalDeadband(0);
        }

        private static SwerveRequest.FieldCentricFacingAngle withPID(SwerveRequest.FieldCentricFacingAngle FCA, PhoenixPIDController PID){
            PID.enableContinuousInput(-Math.PI, Math.PI);
            FCA.HeadingController = PID;

            return FCA;
        }
    }


    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    


    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()){
            startSimThread();
        }

        zeroGyro();
        configurePathPlanner();
       
    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()){
            startSimThread();
        }
        zeroGyro();
        configurePathPlanner();
      
    }

     public SwerveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        zeroGyro();
        configurePathPlanner();
    }



    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        SmartDashboard.putString("Request is being run", requestSupplier.get().toString());
        return run(() -> this.setControl(requestSupplier.get()));
    }


    
    private void configurePathPlanner() {


         try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }  


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



    //getting from the pigeon used to generate CommandSwerveDriveTrain
    public void zeroGyro(){
        this.getPigeon2().setYaw(0);
    }

    public double getGyroYaw(){
   
        return this.getState().Pose.getRotation().getDegrees();

    }



    @Override
    public void periodic() {
        stateHandler.swervePose = this.getState().Pose;
        
        SmartDashboard.putNumber("Heading (Gyro)",getGyroYaw());
    }

    
    /*
     * 
     * SysID Stuff
     * 
     * 
     */



     /* Swerve requests to apply during SysId characterization */
     private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
     private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
     private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
 
     /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
     private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
         new SysIdRoutine.Config(
             null,        // Use default ramp rate (1 V/s)
             Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
             null,        // Use default timeout (10 s)
             // Log state with SignalLogger class
             state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
         ),
         new SysIdRoutine.Mechanism(
             output -> setControl(m_translationCharacterization.withVolts(output)),
             null,
             this
         )
     );
 
     /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
     private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
         new SysIdRoutine.Config(
             null,        // Use default ramp rate (1 V/s)
             Volts.of(7), // Use dynamic voltage of 7 V
             null,        // Use default timeout (10 s)
             // Log state with SignalLogger class
             state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
         ),
         new SysIdRoutine.Mechanism(
             volts -> setControl(m_steerCharacterization.withVolts(volts)),
             null,
             this
         )
     );
 
     /*
      * SysId routine for characterizing rotation.
      * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
      * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
      */
     private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
         new SysIdRoutine.Config(
             /* This is in radians per second², but SysId only supports "volts per second" */
             Volts.of(Math.PI / 6).per(Second),
             /* This is in radians per second, but SysId only supports "volts" */
             Volts.of(Math.PI),
             null, // Use default timeout (10 s)
             // Log state with SignalLogger class
             state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
         ),
         new SysIdRoutine.Mechanism(
             output -> {
                 /* output is actually radians per second, but SysId only supports "volts" */
                 setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                 /* also log the requested output for SysId */
                 SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
             },
             null,
             this
         )
     );
 
     /* The SysId routine to test */
     private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
 
     /**
      * Runs the SysId Quasistatic test in the given direction for the routine
      * specified by {@link #m_sysIdRoutineToApply}.
      *
      * @param direction Direction of the SysId Quasistatic test
      * @return Command to run
      */
     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
         return m_sysIdRoutineToApply.quasistatic(direction);
     }
 
     /**
      * Runs the SysId Dynamic test in the given direction for the routine
      * specified by {@link #m_sysIdRoutineToApply}.
      *
      * @param direction Direction of the SysId Dynamic test
      * @return Command to run
      */
     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
         return m_sysIdRoutineToApply.dynamic(direction);
     }
}
