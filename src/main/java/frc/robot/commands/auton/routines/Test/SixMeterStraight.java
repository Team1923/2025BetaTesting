// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.Test;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixMeterStraight extends SequentialCommandGroup {
  /** Creates a new SixMeterStraight. */
  public SixMeterStraight() {
    addCommands(
      new PathPlannerAuto("6MeterStraightAuto")
    );
  }
}
