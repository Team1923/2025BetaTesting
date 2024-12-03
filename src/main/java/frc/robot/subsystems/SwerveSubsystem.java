
package frc.robot.subsystems;

import java.util.function.Supplier;

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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    private boolean currentLimitsActivated = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (!Utils.isSimulation()){
            //setSwerveDriveCustomCurrentLimits();
            if(!currentLimitsActivated) {
                try {
                    throw new Exception("Swerve Current Limits Not Active!");
                } catch (Exception e) {
                    return;
                }
            }
        }
        else{
            startSimThread();
        }

        zeroGyro();
        //configurePathPlanner();
       
    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (!Utils.isSimulation()){
            //setSwerveDriveCustomCurrentLimits();
            if(!currentLimitsActivated) {
                try {
                    throw new Exception("Swerve Current Limits Not Active!");
                } catch (Exception e) {
                    return;
                }
            }
        }
        else{
            startSimThread();
        }
        zeroGyro();
        //configurePathPlanner();
      
    }

    // public void setSwerveDriveCustomCurrentLimits() {   
    //     //Create a current configuration to use for the drive motor of each swerve module.
    //     CurrentLimitsConfigs customCurrentLimitConfigs = new CurrentLimitsConfigs();
    //     TorqueCurrentConfigs torqueCurrentLimits = new TorqueCurrentConfigs();

    //     //Iterate through each module.
    //     for (var module : Modules) {
    //         //Get the Configurator for the current drive motor.
    //         var currentConfigurator = module.getDriveMotor().getConfigurator();

    //         //Refresh the current configuration, since the stator current limit has already been set.
    //         currentConfigurator.refresh(customCurrentLimitConfigs);
    //         currentConfigurator.refresh(torqueCurrentLimits);

    //         //Set all of the parameters related to the supply current.  The values should come from Constants.

    //         customCurrentLimitConfigs.StatorCurrentLimit = TunerConstants.kSwerveDriveStatorCurrentLimit;
    //         customCurrentLimitConfigs.StatorCurrentLimitEnable = TunerConstants.kSwerveStatorCurrentLimitEnable;
    //         torqueCurrentLimits.PeakForwardTorqueCurrent = TunerConstants.kSwervePeakForwardTorqueCurrent;
    //         torqueCurrentLimits.PeakReverseTorqueCurrent = -TunerConstants.kSwervePeakReverseTorqueCurrent;

    //         // customCurrentLimitConfigs.SupplyCurrentLimit = SwerveConstants.kSwerveDriveSupplyCurrentLimit;
    //         // customCurrentLimitConfigs.SupplyCurrentLimitEnable = SwerveConstants.kSwerveDriveSupplyCurrentLimitEnable;
    //         // customCurrentLimitConfigs.SupplyCurrentThreshold = SwerveConstants.kSwerveDriveSupplyCurrentThreshold;
    //         // customCurrentLimitConfigs.SupplyTimeThreshold = SwerveConstants.kSwerveDriveSupplyTimeThreshold;

  

    //         //Apply the new current limit configuration.
    //         currentConfigurator.apply(customCurrentLimitConfigs);
    //         currentConfigurator.apply(torqueCurrentLimits);

    //         // System.out.println(module.getDriveMotor().getConfigurator().);
    //         // System.out.println("did something");

    //      }

    //     currentLimitsActivated = true;

    // }


    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        SmartDashboard.putString("Request is being run", requestSupplier.get().toString());
        return run(() -> this.setControl(requestSupplier.get()));
    }

    // public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    //     return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    // }

    
    // private void configurePathPlanner() {

    //     // /*Have to do this here, because this needs to happen AFTER swerve subsystem is instantiated but BEFORE the autobuilder */
    //     // // NamedCommands.registerCommand("ScoreCommandGroup", new AutoScoreCommandGroup(this));
    //     // // NamedCommands.registerCommand("AlignHeading", new AlignHeading(this, () -> 0, () -> 0));
    //     // // NamedCommands.registerCommand("NonAutoScoreCommandGroup", new GCScoreCommandGroup(this, () -> 0, () -> 0, () -> 0));



    //     // double driveBaseRadius = 0;
    //     // for (var moduleLocation : m_moduleLocations) {
    //     //     driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    //     // }

    //     // PPHolonomicDriveController holoPathFollower = new PPHolonomicDriveController(
    //     //     new PIDConstants(5.0, 0, 0), //TODO: TUNE!
    //     //     new PIDConstants(5.0, 0, 0), //TODO: TUNE!
    //     //     TunerConstants.kSpeedAt12VoltsMps,
    //     //     driveBaseRadius,
    //     //     new ReplanningConfig());


    //     // AutoBuilder.configureHolonomic(
    //     //     () -> this.getState().Pose,
    //     //     this::seedFieldRelative,
    //     //     this::getCurrentRobotChassisSpeeds,
    //     //     (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
    //     //     holoPathFollower,
    //     //     () -> {
    //     //         var alliance = DriverStation.getAlliance();
    //     //         if (alliance.isPresent()) {
    //     //             return alliance.get() == DriverStation.Alliance.Red;
    //     //         }
    //     //         return false;
    //     //         },
    //     //     this);
    // }  


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
        // this.getPigeon2().setYaw(stateHandler.getAutoHeadingOffset());
        // this.getPigeon2().setYaw( (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? 0 : 180);

    }

    public double getGyroYaw(){
        // return this.m_yawGetter.getValueAsDouble();
        // return Rotation2d.fromDegrees(Math.IEEEremainder(this.getPigeon2().getYaw().getValueAsDouble(),360));
        // return Rotation2d.fromDegrees(Math.IEEEremainder(((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? 180 : 0) +this.m_yawGetter.getValueAsDouble(),360));
        // return Rotation2d.fromDegrees( (DriverStation.getAlliance().isPresent() == true && DriverStation.getAlliance().get() == Alliance.Blue) 
                            // ? Math.IEEEremainder(this.m_yawGetter.getValueAsDouble() + stateHandler.getAutoHeadingOffset(),360) 
                            // : Math.IEEEremainder(this.m_yawGetter.getValueAsDouble() - stateHandler.getAutoHeadingOffset(),360)
                            // );
        return this.getState().Pose.getRotation().getDegrees();

    }



    @Override
    public void periodic() {
        stateHandler.swervePose = this.getState().Pose;
        // stateHandler.setCurrentRobotHeading(getGyroYaw().getDegrees());

        
        SmartDashboard.putNumber("Heading (Gyro)",getGyroYaw());
    }
}
