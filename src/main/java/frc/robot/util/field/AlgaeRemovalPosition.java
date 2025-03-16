package frc.robot.util.field;

public enum AlgaeRemovalPosition {
    AB(true),
    CD(false),
    EF(true),
    GH(false),
    IJ(true),
    KL(false);

    private boolean isTopRow;

    private AlgaeRemovalPosition(boolean isTopRow) {
        this.isTopRow = isTopRow;
    }

    public boolean isTopRow() {
        return isTopRow;
    }

    public boolean isBottomRow() {
        return !isTopRow;
    }
}
