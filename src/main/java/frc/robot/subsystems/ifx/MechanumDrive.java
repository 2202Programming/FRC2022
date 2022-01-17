package frc.robot.subsystems.ifx;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MechanumDrive extends Subsystem {
    void drive_normalized (double x, double y, double rotation);
}
