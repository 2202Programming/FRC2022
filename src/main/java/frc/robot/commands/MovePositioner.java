package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Positioner_Subsystem;

public class MovePositioner extends InstantCommand {
    PositionerMode mode;
    Positioner_Subsystem positioner;

    public enum PositionerMode {
        High, Low, Toggle
    }

    /**
     * Constructor
     * use this version to set a specific state
     * 
     * @param mode High --> shooter angle is high
     *             Low --> shooter angle is low
     *             Toggle --> change shooter angle to opposite
     */
    public MovePositioner(PositionerMode mode) {
        this.positioner = RobotContainer.RC().positioner;
        addRequirements(positioner);
        // go where we are told
        this.mode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (mode == PositionerMode.High) {
            positioner.deploy();
        }
        if (mode == PositionerMode.Low) {
            positioner.retract();
        }
        if (mode == PositionerMode.Toggle) {
            if (positioner.isDeployed() == false) {
                positioner.deploy();
            } else {
                positioner.retract();
            }
        }

    }

}
