package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Positioner_Subsystem;

public class PositionerToggle extends InstantCommand {

    Positioner_Subsystem positioner;
    boolean toggle = true;  
    boolean deploy_requested = false;

    // Constructor
    /**
     * Use this version to toggle deploy/retract state.
     */
    public PositionerToggle() {
        this.positioner = RobotContainer.RC().positioner;
        addRequirements(positioner);  //dpl -may need to remove so it doesn't conflict with motor
    }

    /**
     * use this version to set a specific state
     * @param deployed   true  --> intake will be deployed
     *                   false --> intake will be retracted
     */ 
    public PositionerToggle(boolean deployed) {
        this();
        toggle = false;     // go where we are told
        deploy_requested = deployed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (toggle) {
            if (positioner.isDeployed() == false) { positioner.deploy(); }
            else {positioner.retract(); }
        }
        else {
            // go where requested
            if (deploy_requested) { positioner.deploy(); }
            else {positioner.retract(); }
        }
    }

}
