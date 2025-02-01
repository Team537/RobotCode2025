package frc.robot.util;

import frc.robot.Constants.SquidConstants.SquidClimberConstants;

public enum SquidClimberPosition {
    DOWN(SquidClimberConstants.DOWN_POSITION),
    CLIMBED(SquidClimberConstants.CLIMBED_POSITION);

    private final double position;

    SquidClimberPosition(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
