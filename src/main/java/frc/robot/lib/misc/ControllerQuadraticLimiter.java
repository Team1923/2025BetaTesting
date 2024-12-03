// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.misc;

/** Add your docs here. */
public class ControllerQuadraticLimiter {

    public static double quadraticLimit(double d){

        int sign = 1;

        if (d < 0){
            sign = -1;
        }

        return d*d*sign;
    }
}
