package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadButton extends Trigger {

    XboxController joystick;
    Direction direction;

    public DPadButton(XboxController joystick, Direction direction) {
        super(() -> get(joystick, direction));
        this.joystick = joystick;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    @Deprecated
    public boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction);
    }

    public static boolean get(XboxController joystick, Direction direction) {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction);
    }
}