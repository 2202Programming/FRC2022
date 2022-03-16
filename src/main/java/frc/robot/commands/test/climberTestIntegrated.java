package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber.HigherClimbExtend;
import frc.robot.commands.climber.HigherClimbRetract;
import frc.robot.commands.climber.MidClimbExtend;
import frc.robot.commands.climber.MidClimbRetract;
import frc.robot.subsystems.climber.Climber;

public class climberTestIntegrated extends CommandBase {

    private Climber climber;
    NetworkTable table;

    // What stage being tested
    NetworkTableEntry Stage;
    double chosenNumber;
    double previousNumber;

    public climberTestIntegrated(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
        table = NetworkTableInstance.getDefault().getTable("Climber");
        Stage = table.getEntry("Stage");
        chosenNumber = 0;
        previousNumber = chosenNumber;

    }

    @Override
    public void execute() {
        chosenNumber = table.getEntry("Stage").getDouble(0);
        if (chosenNumber != previousNumber) {
            if (chosenNumber == 1)
                new MidClimbExtend(climber).schedule();
            else if (chosenNumber == 2)
                new MidClimbRetract(climber).schedule();
            else if (chosenNumber == 3)
                new HigherClimbExtend(climber).schedule();
            else if (chosenNumber == 4)
                new HigherClimbRetract(climber).schedule();
            previousNumber = chosenNumber;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //climber.stop();
    }

}
