// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.tuningwidgets;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

/** Add your docs here. */
public class MotorPIDFVAJWidget {

    private ShuffleboardTab tab;
    private TalonFX[] motors;

    private TalonFXConfiguration config;

    private double angleConversion;
    private double velocityConversion;
    private double tolerance;

    private GenericEntry PValue;
    private GenericEntry IValue;
    private GenericEntry DValue;
    private GenericEntry FFValue;
    private GenericEntry GValue;
    private GenericEntry VValue;
    private GenericEntry AValue;




    private GenericEntry VeloValue;
    private GenericEntry AccelValue;
    private GenericEntry JerkValue;

    private SuppliedValueWidget<Double> currentAngle;
    private SuppliedValueWidget<Double> currentVelocity;

    private GenericEntry desiredAngle;
    private GenericEntry desiredVelocity;


    private SendableChooser<Integer> slotChooser;


    private MotionMagicVoltage angleRequest;
    private MotionMagicVelocityVoltage veloRequestTop;
    private MotionMagicVelocityVoltage veloRequestBottom;







    /**
     * 
     * @param name
     * @param motors
     * @param defaultConfig ASSUMES ALL MOTORS HAVE SAME DEFAULT CONFIG
     */
    public MotorPIDFVAJWidget(String name, TalonFXConfiguration defaultConfig, int defaultSlot, double angleConversion, double velocityConversion, double tolerance, ControlRequest request, Command toRun, TalonFX... motors){
        tab = Shuffleboard.getTab(name);

        this.motors = motors;

        this.config = defaultConfig;


        this.angleConversion = angleConversion;
        this.velocityConversion = velocityConversion;
        
        this.tolerance = tolerance;

        slotChooser = new SendableChooser<>();

        slotChooser.addOption("Slot0", 0);
        slotChooser.addOption("Slot1", 1);
        slotChooser.addOption("Slot2", 2);

        SlotConfigs slot;

        switch(defaultSlot){
            case 0:
                slot = SlotConfigs.from(defaultConfig.Slot0);
                slotChooser.setDefaultOption("Slot0", 0);
                break;
            case 1:
                slot = SlotConfigs.from(defaultConfig.Slot1);
                slotChooser.setDefaultOption("Slot1", 1);
                break;
            case 2:
                slot = SlotConfigs.from(defaultConfig.Slot2);
                slotChooser.setDefaultOption("Slot2", 2);
                break;
            default:
                slot = SlotConfigs.from(defaultConfig.Slot0);
                System.out.println("invalid slot for " +name+" widget");
        }


        PValue = tab.add("P", slot.kP).withPosition(0, 0).getEntry();
        IValue = tab.add("I", slot.kI).withPosition(1, 0).getEntry();
        DValue = tab.add("D", slot.kD).withPosition(2,0).getEntry();
        FFValue= tab.add("Static FF (kS)", slot.kS).withPosition(3, 0).getEntry();
        GValue= tab.add("Gravity (kG)", slot.kG).withPosition(4, 0).getEntry();
        VValue= tab.add("Velocity FF (kV)",slot.kV).withPosition(5, 0).getEntry();
        AValue= tab.add("Accel FF (kA)", slot.kA).withPosition(6, 0).getEntry();
        
        
        VeloValue = tab.add("Cruise Velocity", defaultConfig.MotionMagic.MotionMagicCruiseVelocity).withPosition(0, 1).getEntry();
        AccelValue = tab.add("Acceleration", defaultConfig.MotionMagic.MotionMagicAcceleration).withPosition(1, 1).getEntry();
        JerkValue = tab.add("Jerk", defaultConfig.MotionMagic.MotionMagicJerk).withPosition(2, 1).getEntry();


        if (request instanceof MotionMagicVoltage){
            angleRequest = (MotionMagicVoltage)request;
        }
        else if (request instanceof MotionMagicVelocityVoltage){
            veloRequestTop = (MotionMagicVelocityVoltage)request;
        }
        else{
            System.out.println("invalid request in " + name + "PIDVAJ");
        }


        if (angleConversion != 0){
            System.out.println(name + "HERE");
            currentAngle = tab.addDouble("CURRENT ANGLE", () -> motors[0].getPosition().getValueAsDouble() * angleConversion).withPosition(0,2).withSize(2, 1);
            desiredAngle = tab.add("DESIRED TUNING ANGLE", ((MotionMagicVoltage)ArmStates.ANGLE_TUNING.REQUEST).Position * angleConversion).withPosition(0, 3).withSize(2, 1).getEntry();

        }

        if (velocityConversion != 0){
            currentVelocity= tab.addDouble("CURRENT VELOCITY", () -> motors[0].getVelocity().getValueAsDouble() * velocityConversion).withPosition(2,2).withSize(2, 1);
            desiredVelocity = tab.add("DESIRED TUNING VELOCITY", ((MotionMagicVelocityVoltage)ShooterStates.RPM_TUNING.REQUEST_TOP).Velocity * velocityConversion).withPosition(2, 3).withSize(2, 1).getEntry();
      
        }


        

        tab.add("SLOT", slotChooser).withPosition(4, 3);
        tab.add("UPDATE", new InstantCommand(() -> updateMotor()).ignoringDisable(true)).withPosition(4, 2).withSize(2, 1);
        tab.add("GOTO", toRun).withPosition(7, 2).withSize(2, 1);
        tab.addBoolean("AT POSITION", () -> atPosition()).withPosition(6, 2);
    }

    public MotorPIDFVAJWidget(String name, TalonFXConfiguration defaultConfig, int defaultSlot, double angleConversion, double velocityConversion, double tolerance, ControlRequest requestTop, ControlRequest requestBottom, Command toRun, TalonFX... motors){
        this(name, defaultConfig, defaultSlot, angleConversion, velocityConversion, tolerance, requestTop, toRun, motors);
        if (requestBottom instanceof MotionMagicVelocityVoltage){
            veloRequestBottom = (MotionMagicVelocityVoltage)requestBottom;
        }
        else{
            System.out.println("??? What are you doing in constructor 2 of PIDVAGWidget for " + name);
        }
    }

    public void updateMotor(){

        
        if (slotChooser.getSelected() == -1){
            System.out.println("NO THINGY SELECTED");

        }
        else{
            switch(slotChooser.getSelected()){
            case 0:
                config.Slot0.kP = PValue.get().getDouble();
                config.Slot0.kI = IValue.get().getDouble();
                config.Slot0.kD = DValue.get().getDouble();
                config.Slot0.kS = FFValue.get().getDouble();
                config.Slot0.kG = GValue.get().getDouble();
                config.Slot0.kV = VValue.get().getDouble();
                config.Slot0.kA = AValue.get().getDouble();
                break;
            case 1:
                config.Slot1.kP = PValue.get().getDouble();
                config.Slot1.kI = IValue.get().getDouble();
                config.Slot1.kD = DValue.get().getDouble();
                config.Slot1.kS = FFValue.get().getDouble();
                config.Slot1.kG = GValue.get().getDouble();
                config.Slot1.kV = VValue.get().getDouble();
                config.Slot1.kA = AValue.get().getDouble();
                break;
            case 2:
                config.Slot2.kP = PValue.get().getDouble();
                config.Slot2.kI = IValue.get().getDouble();
                config.Slot2.kD = DValue.get().getDouble();
                config.Slot2.kS = FFValue.get().getDouble();
                config.Slot2.kG = GValue.get().getDouble();
                config.Slot2.kV = VValue.get().getDouble();
                config.Slot2.kA = AValue.get().getDouble();
                break;
            default:
                System.out.println("WENT TO DEFAULT");
        }
        }
        

        config.MotionMagic.MotionMagicCruiseVelocity = VeloValue.get().getDouble();
        config.MotionMagic.MotionMagicAcceleration = AccelValue.get().getDouble();
        config.MotionMagic.MotionMagicJerk = JerkValue.get().getDouble();


        if (angleRequest != null){
            if (desiredAngle != null) {
                angleRequest.Position = desiredAngle.get().getDouble() * 1/angleConversion;
            }
        }
        
        else if (veloRequestTop != null){
            if (desiredVelocity != null){
                veloRequestTop.Velocity = desiredVelocity.get().getDouble() * 1/velocityConversion;
                veloRequestBottom.Velocity = desiredVelocity.get().getDouble() * 1/velocityConversion;
            } 
        }
        


        for (TalonFX motor : motors){
            motor.getConfigurator().apply(config);
        }


    }

    public boolean atPosition(){
       
        if (angleConversion != 0){
            return (Math.abs(motors[0].getPosition().getValueAsDouble() * angleConversion -((desiredAngle.get() != null) ? desiredAngle.get().getDouble() : 0)) < tolerance);

        }
        else if (velocityConversion != 0){
            return (Math.abs(motors[0].getVelocity().getValueAsDouble() * velocityConversion - ((desiredVelocity.get() != null) ? desiredVelocity.get().getDouble() : 0)) < tolerance);

        }
        else{
            return false;
        }
    }

}