package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem; 
import frc.robot.subsystems.ifx.DriverControls;
public class MagazineCommand extends CommandBase {
    private final Magazine_Subsystem magazine;
    private final DriverControls dc;
    public MagazineCommand(Magazine_Subsystem magazine, DriverControls dc){
        this.magazine = magazine;
        this.dc = dc;
    }
}
