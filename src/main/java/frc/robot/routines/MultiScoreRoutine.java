package frc.robot.routines;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.upper_assembly.UpperAssembly;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.StartingPosition;
import frc.robot.util.field.CoralStationSide;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.upper_assembly.ScoringHeight;

public class MultiScoreRoutine {
    private static ReefScoringLocation scoringLocation = ReefScoringLocation.E;
    private static CoralStationSide coralStationSide = CoralStationSide.RIGHT;
    private static boolean useClockwiseFillingScheme = false;

    public static Command getCommand(StartingPosition startingPosition, Alliance alliance, DriveSubsystem driveSubsystem, UpperAssembly upperAssembly) {
        driveSubsystem.setRobotPose(startingPosition.getPose(alliance));

        // From the left side, start at J (maybe this should be I?) then rotate counter clockwise around the reef
        if (startingPosition == StartingPosition.LEFT) {
            scoringLocation = ReefScoringLocation.J;
            coralStationSide = CoralStationSide.LEFT;
            useClockwiseFillingScheme = false;
        }
        // From the right side, start at E (maybe this should be F?) then rotate clockwise around the reef
        else {
            scoringLocation = ReefScoringLocation.E;
            coralStationSide = CoralStationSide.RIGHT;
            useClockwiseFillingScheme = true;
        }

        // Do first score directly
        return driveSubsystem
            .getScoringCommand(alliance, scoringLocation)
            .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
            .andThen(
                // Then loop intake -> score, shifting around the reef
                // Deferred so that the position is recalculated each loop
                new DeferredCommand(() ->
                    driveSubsystem.getIntakeCommand(alliance, coralStationSide, 2)
                        .alongWith(upperAssembly.getCoralIntakeCommand())
                        .andThen((driveSubsystem.getScoringCommand(alliance, getNextScoringLocation())
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))))
                , Set.of(driveSubsystem)).repeatedly()
            );
    }

    private static ReefScoringLocation getNextScoringLocation() {
        scoringLocation = ReefScoringLocation.getNextScoringLocation(scoringLocation, useClockwiseFillingScheme);

        System.out.println("Next scoring position: " + scoringLocation.toString());

        return scoringLocation;
    }
}
