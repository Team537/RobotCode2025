package frc.robot.util.field;

public enum AlgaeRemovalPosition {
    AB_TOP(AlgaeRemovalBasePosition.AB, AlgaeRemovalHeight.TOP),
    CD_TOP(AlgaeRemovalBasePosition.CD, AlgaeRemovalHeight.TOP),
    EF_TOP(AlgaeRemovalBasePosition.EF, AlgaeRemovalHeight.TOP),
    GH_TOP(AlgaeRemovalBasePosition.GH, AlgaeRemovalHeight.TOP),
    IJ_TOP(AlgaeRemovalBasePosition.IJ, AlgaeRemovalHeight.TOP),
    KL_TOP(AlgaeRemovalBasePosition.KL, AlgaeRemovalHeight.TOP),
    AB_BOTTOM(AlgaeRemovalBasePosition.AB, AlgaeRemovalHeight.BOTTOM),
    CD_BOTTOM(AlgaeRemovalBasePosition.CD, AlgaeRemovalHeight.BOTTOM),
    EF_BOTTOM(AlgaeRemovalBasePosition.EF, AlgaeRemovalHeight.BOTTOM),
    GH_BOTTOM(AlgaeRemovalBasePosition.GH, AlgaeRemovalHeight.BOTTOM),
    IJ_BOTTOM(AlgaeRemovalBasePosition.IJ, AlgaeRemovalHeight.BOTTOM),
    KL_BOTTOM(AlgaeRemovalBasePosition.KL, AlgaeRemovalHeight.BOTTOM);

    public final AlgaeRemovalBasePosition algaeRemovalBasePosition;
    public final AlgaeRemovalHeight algaeRemovalHeight;

    private AlgaeRemovalPosition(AlgaeRemovalBasePosition algaeRemovalBasePosition , AlgaeRemovalHeight algaeRemovalHeight) {
        this.algaeRemovalBasePosition = algaeRemovalBasePosition;
        this.algaeRemovalHeight = algaeRemovalHeight;
    }

    /*
     * The position of the algae removal position around the coral station
     */
    public enum AlgaeRemovalBasePosition {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL
    }

    /*
     * The height of the algae removal position inbetween any given two coral - either the top or the bottom
     */
    public enum AlgaeRemovalHeight {
        TOP,
        BOTTOM
    }

    public boolean isTopRow() {
        return algaeRemovalHeight == AlgaeRemovalHeight.TOP;
    }

    public boolean isBottomRow() {
        return algaeRemovalHeight == AlgaeRemovalHeight.BOTTOM;
    }
}
