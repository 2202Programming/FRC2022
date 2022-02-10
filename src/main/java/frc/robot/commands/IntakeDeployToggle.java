package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeDeployToggle extends InstantCommand {

    Intake_Subsystem intake;
    boolean toggle = true;  
    boolean deploy_requested = false;

    // Constructor
    /**
     * Use this version to toggle deploy/retract state.
     */
    public IntakeDeployToggle() {
        this.intake = RobotContainer.RC().intake;
        addRequirements(intake);  //dpl -may need to remove so it doesn't conflict with motor
    }

    /**
     * use this version to set a specific state
     * @param deployed   true  --> intake will be deployed
     *                   false --> intake will be retracted
     */ 
    public IntakeDeployToggle(boolean deployed) {
        this();
        toggle = false;     // go where we are told
        deploy_requested = deployed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (toggle) {
            if (intake.isDeployed() == false) { intake.deploy(); }
            else {intake.retract(); }
        }
        else {
            // go where requested
            if (deploy_requested) { intake.deploy(); }
            else {intake.retract(); }
        }
    }

}
