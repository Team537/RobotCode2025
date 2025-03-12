package frc.robot.routine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.upper_assembly.UpperAssembly;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.StartingPosition;
import frc.robot.util.upper_assembly.ScoringHeight;

public class SimpleAutomatedScoringRoutine implements IRoutine {
    private final Alliance alliance;
    private final DriveSubsystem driveSubsystem;
    private final StartingPosition startingPosition;
    private final UpperAssembly upperAssembly;

    public SimpleAutomatedScoringRoutine(Alliance alliance, StartingPosition startingPosition, DriveSubsystem driveSubsystem, UpperAssembly upperAssembly) {
        this.alliance = alliance;
        this.driveSubsystem = driveSubsystem;
        this.startingPosition = startingPosition;
        this.upperAssembly = upperAssembly;
    }

    public Command getCommandSet() {
        Pose2d startingPoint = new Pose2d(); // todo this should be based on our alliance and starting position (left center right)
        Pose2d almostIinitalScoringPosition = new Pose2d(new Translation2d(11, 4), new Rotation2d(Math.PI)); // TODO determine this from starting position / etc
        Pose2d initalScoringPosition = new Pose2d(new Translation2d(11.845, 4.179), new Rotation2d(Math.PI)); // TODO determine this from starting position / etc
        Pose2d driversStationPosition = new Pose2d(); // TODO this will also vary based on where our human player is


        // Gross movement first - go quickly to "almost" position
        return new InstantCommand(() -> this.driveSubsystem.getPathfindingCommand(almostIinitalScoringPosition))
            .andThen(() ->
                // Fine movement - finish "travel" more slowly, while also scoring
                new InstantCommand(() -> this.driveSubsystem.getPathfindingCommand(initalScoringPosition))
                        .alongWith(() -> this.upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
            )
            .andThen(this.driveSubsystem.getPathfindingCommand(driversStationPosition))
            .andThen(() -> this.upperAssembly.getCoralIntakeCommand());
    }
}
