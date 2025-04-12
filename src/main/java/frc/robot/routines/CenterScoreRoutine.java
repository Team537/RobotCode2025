package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.upper_assembly.UpperAssembly;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.upper_assembly.ScoringHeight;

public class CenterScoreRoutine {
    public static Command getCommand(Alliance alliance, DriveSubsystem driveSubsystem, UpperAssembly upperAssembly) {
        return driveSubsystem
            .getScoringCommand(alliance, ReefScoringLocation.H)
            .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
            .andThen(upperAssembly.getLowerCommand());
    }
}
