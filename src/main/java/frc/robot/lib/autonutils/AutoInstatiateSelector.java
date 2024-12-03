// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.autonutils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.routines.AmpRanged.AmpRanged123;
import frc.robot.commands.auton.routines.AmpRanged.AmpRanged213;
import frc.robot.commands.auton.routines.AmpRanged.AmpRanged231;
import frc.robot.commands.auton.routines.AmpSubwooferRanged.AmpSubwooferRanged123;
import frc.robot.commands.auton.routines.AmpSubwooferRanged.AmpSubwooferRanged213;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged345;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged354;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged435;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged453;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged5;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged534;
import frc.robot.commands.auton.routines.SourceSubwooferRanged.SourceSubwooferRanged543;
import frc.robot.commands.auton.routines.SubwooferRanged.SubwooferRangedSMA12;
import frc.robot.commands.auton.routines.SubwooferRanged.SubwooferRangedSMA21;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubAMS;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubASM;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubM;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubMAS;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubMSA;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubSMA;
import frc.robot.commands.auton.routines.SubwooferSub.SubwooferSubSAM;
import frc.robot.commands.auton.routines.Test.SixMeterRangedShooting;
import frc.robot.commands.auton.routines.Test.SixMeterRotation;
import frc.robot.commands.auton.routines.Test.SixMeterStraight;
import frc.robot.commands.auton.routines.Test.SubwooferShootingTest;
import frc.robot.commands.auton.routines.Trap.SourceSubwooferTrap;
import frc.robot.commands.auton.routines.Test.SubwooferRangedShooting;

/** Add your docs here. */
public class AutoInstatiateSelector {
    public enum AutoMode {
        // SOURCE_SUBWOOFER_RANGED(new SourceSubwooferRanged5()),
        // AMP_RANGED_123(new AmpRanged123()),
        // AMP_RANGED_213(new AmpRanged213()),
        // AMP_RANGED_231(new AmpRanged231()),

        /* Autos starting from the side of the Subwoofer closest to the Amp, shooting from ranged */
        AMP_SUBWOOFER_RANGED_123(new AmpSubwooferRanged123()),
        AMP_SUBWOOFER_RANGED_213(new AmpSubwooferRanged213()),

        /* Autos starting from the center of the Subwoofer, shooting into the subwoofer */
        SUBWOOFER_SUB_SMA(new SubwooferSubSMA()),
        SUBWOOFER_SUB_SAM(new SubwooferSubSAM()),
        SUBWOOFER_SUB_MAS(new SubwooferSubMAS()),
        SUBWOOFER_SUB_MSA(new SubwooferSubMSA()),
        SUBWOOFER_SUB_AMS(new SubwooferSubAMS()),
        SUBWOOFER_SUB_ASM(new SubwooferSubASM()),
        // SUBWOOFER_SUB_M(new SubwooferSubM()),

        /* Autos starting from the center of the Subwoofer, shooting from ranged */
        SUBWOOBER_RANGED_SMA12(new SubwooferRangedSMA12()),
        SUBWOOBER_RANGED_SMA21(new SubwooferRangedSMA21()),

        /* Autos starting on the side of the Subwoofer closest to the Source, shooting from ranged */
        SOURCE_SUBWOOFER_RANGED_543(new SourceSubwooferRanged543()),
        SOURCE_SUBWOOFER_RANGED_534(new SourceSubwooferRanged534()),
        SOURCE_SUBWOOFER_RANGED_435(new SourceSubwooferRanged435()),
        SOURCE_SUBWOOFER_RANGED_453(new SourceSubwooferRanged453()),
        SOURCE_SUBWOOFER_RANGED_345(new SourceSubwooferRanged345()),
        SOURCE_SUBWOOFER_RANGED_354(new SourceSubwooferRanged354()),

        /* Autos that end with a trap shot */
        SOURCE_SUBWOOFER_TRAP(new SourceSubwooferTrap());
        
        //testing
        // SIX_METER_STRAIGHT(new SixMeterStraight()),
        // SIX_METER_ROTATION(new SixMeterRotation()),
        // SUBWOOFER_SHOOT_TEST(new SubwooferShootingTest()),
        // SUBWOOFER_RANGED_TEST(new SubwooferRangedShooting()),
        // SIX_METER_RANGED_TEST(new SixMeterRangedShooting());
    

        private Command routine;

        private AutoMode(Command routine) {
            this.routine = routine;
        }

        public Command getAutonRoutine() {
            return routine;
        }
    }

    private SendableChooser<AutoMode> chooser;

    ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
    ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
            .withPosition(3, 0)
            .withSize(2, 1);

    public AutoInstatiateSelector() {
        chooser = new SendableChooser<>();

        for (AutoMode m : AutoMode.values()) {
            chooser.addOption(m.name(), m);
        }

        auto.add(chooser);
    }

    public Command startMode() {
        AutoMode mode = (AutoMode) (chooser.getSelected());

        return mode.getAutonRoutine();
    }
}
