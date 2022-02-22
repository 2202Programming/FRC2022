package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.HigherClimbExtend;
import frc.robot.commands.HigherClimbRetract;
import frc.robot.commands.MidClimbExtend;
import frc.robot.commands.MidClimbRetract;
import frc.robot.subsystems.Climber;

public class climberTestIntegrated extends CommandBase {

    private Climber climber;
    NetworkTable table;

    // Magnitude of rotation / extension left / right motors
    NetworkTableEntry ntRot;
    NetworkTableEntry ntExt;
    double last_rot;
    double last_ext;
    double chosenNumber;
    double previousNumber;

    public climberTestIntegrated(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
        table = NetworkTableInstance.getDefault().getTable("Climber");
        ntRot = table.getEntry("Rotation");
        ntRot.setDouble(0);
        last_rot = 0;
        chosenNumber = 0;
        previousNumber = chosenNumber;

    }

    @Override
    public void execute() {
        chosenNumber = table.getEntry("Rotation").getDouble(0);
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
        climber.stop();
    }

}
