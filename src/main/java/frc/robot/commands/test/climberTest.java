package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class climberTest extends CommandBase {

    private Climber climber;
    
    public climberTest(Climber climber){
        this.climber = climber;
    }

    public void execute() {
        climber.setExtension(10);
        climber.setRotation(30);
    }
}
