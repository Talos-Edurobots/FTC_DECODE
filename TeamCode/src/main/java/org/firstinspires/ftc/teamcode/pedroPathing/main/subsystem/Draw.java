package org.firstinspires.ftc.teamcode.pedroPathing.main.subsystem;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

public final class Draw {
    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();
    private final Style style = new Style(
            "", "#3F51B5", .75
    );
    public void drawRobot(double x, double y, double heading){
        fieldManager.setStyle(style);
        fieldManager.moveCursor(x, y);
        fieldManager.setCursorHeading(heading);
        fieldManager.rect(x, y);
        fieldManager.update();
    }
}
