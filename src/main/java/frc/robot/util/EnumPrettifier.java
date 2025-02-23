package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class EnumPrettifier {
    public static <E extends Enum<E>> String[] getPrettyValues(Class<E> enumToEvaluate) {
        E[] enumerationValues = enumToEvaluate.getEnumConstants();
        String[] valuesToReturn = new String[enumerationValues.length];
        for (int i = 0; i < enumerationValues.length; i++) {
            valuesToReturn[i] = getTitleCaseString(enumerationValues[i].toString());
        }

        return valuesToReturn;
    }

    public static <E extends Enum<E>> void setupSendableChooserFromEnum(SendableChooser<E> chooser, Class<E> enumToEvalute, E defaultValue) {
        chooser.setDefaultOption(getTitleCaseString(defaultValue.toString()), defaultValue);
        for (E option : enumToEvalute.getEnumConstants()) {
            chooser.addOption(getTitleCaseString(option.toString()), option);
        }
    }

    private static String getTitleCaseString(String baseValue) {
        StringBuilder prettyBuilder = new StringBuilder(baseValue.length());
        boolean lastValueWasSpace = true;
        for (char piece : baseValue.toCharArray()) {
            if (lastValueWasSpace) {
                prettyBuilder.append(Character.toTitleCase(piece));
                lastValueWasSpace = false;
            } else if (piece == '_') {
                prettyBuilder.append(' ');
                lastValueWasSpace = true;
            } else {
                prettyBuilder.append(Character.toLowerCase(piece));
            }
        }

        return prettyBuilder.toString();
    }
}
