// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.tuningwidgets;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

/** Add your docs here. */
public class SwerveRequestPIDWidget {


    
private PhoenixPIDController swerveHeadingPID;
private ComplexWidget headingPIDWidget;

  public SwerveRequestPIDWidget(SwerveStates swerveState){


    swerveHeadingPID = ((SwerveRequest.FieldCentricFacingAngle)swerveState.REQUEST).HeadingController;

   headingPIDWidget = Shuffleboard.getTab("PID Swerve").add(swerveState.toString()+" Heading Controller", swerveHeadingPID);
  }

  
}