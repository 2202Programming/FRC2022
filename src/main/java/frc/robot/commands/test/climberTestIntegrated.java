package frc.robot.commands.test;

import java.sql.Driver;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber.MoveArmsTo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.ifx.DriverControls;

public class climberTestIntegrated extends CommandBase {

    private Climber climber;
    private DriverControls dc;
    NetworkTable table;

    // What stage being tested
    NetworkTableEntry Stage;
    double chosenNumber;
    double previousNumber;

    public climberTestIntegrated(Climber climber, DriverControls dc) {
        this.climber = climber;
        this.dc = dc;
        addRequirements(climber);
        table = NetworkTableInstance.getDefault().getTable("Climber");
        Stage = table.getEntry("Stage");
        chosenNumber = 0;
        previousNumber = chosenNumber;

    }

    //TODO: just random positions, make it real or delete this cmd
    @Override
    public void execute() {
        chosenNumber = table.getEntry("Stage").getDouble(0);
        if (chosenNumber != previousNumber) {
            if (chosenNumber == 1)
                new MoveArmsTo(climber, 0.0, 0.0, dc).schedule();
            else if (chosenNumber == 2)
                new MoveArmsTo(climber, 10, 10, dc).schedule();
            else if (chosenNumber == 3)
                new MoveArmsTo(climber, 15, 5, dc).schedule();
            else if (chosenNumber == 4)
                new MoveArmsTo(climber, 19, 0, dc).schedule();
            previousNumber = chosenNumber;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // since we are scheduling other climb commands we expect this to be interrupted
        // so do nothing - dpl
    }

}
