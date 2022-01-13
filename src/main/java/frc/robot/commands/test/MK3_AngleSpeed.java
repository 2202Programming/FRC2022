// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// BooleanConsumer changed: new import, below old import
import edu.wpi.first.util.function.BooleanConsumer;
// import edu.wpi.first.wpiutil.sendable.SendableBuilder.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModuleMK3;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class MK3_AngleSpeed extends CommandBase {
  
  static class ActionOnEdge {
    boolean prev;
    boolean toggle;
    final JoystickButton button;
    final BooleanConsumer func;
    final String name;

    public  ActionOnEdge(String name, JoystickButton b, BooleanConsumer toggleFunc) {
      this.name = name;
      toggle = false;
      func = toggleFunc;
      button = b;
      read();
    }
    
    private boolean read() {
      prev = button.get();
      return prev;
    }

    boolean risingEdge() {
      if (prev == true) {
          read();
          return false;
        }
      else if (read()) {
        toggle = !toggle;
        func.accept(toggle);
        System.out.println(name + " set to:"  + toggle);
        return true;
      }
      return false;
    }
  }

  final DriverControls dc;
  final SwerveDrivetrain drive;
  final SwerveModuleMK3 module;
  
  final SwerveModuleState desiredState;

  ActionOnEdge invertAngleCmd;
  ActionOnEdge invertAngleMotor;
  ActionOnEdge invertDriveMotor;
  

  public MK3_AngleSpeed(DriverControls dc, SwerveDrivetrain drive, int modID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dc = dc;
    this.drive = drive;
    this.module = drive.getMK3(modID);
  
    //setup actions
    invertAngleCmd =   new ActionOnEdge(module.getNTPrefix() + " Angle Cmd Invert", 
                            dc.bindButton(Id.Driver, XboxButton.A.value), module::_setInvertAngleCmd);
    invertAngleMotor = new ActionOnEdge(module.getNTPrefix() + " Angle Motor Invert", 
                            dc.bindButton(Id.Driver, XboxButton.B.value), module::_setInvertAngleMotor);
    invertDriveMotor = new ActionOnEdge(module.getNTPrefix() + " Drive Motor Invert", 
                            dc.bindButton(Id.Driver, XboxButton.X.value), module::_setInvertDriveMotor);
    
    desiredState = new SwerveModuleState();
    NTConfig();
    addRequirements(this.drive);
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // call any triggered actions
    invertAngleCmd.risingEdge();
    invertAngleMotor.risingEdge();
    invertDriveMotor.risingEdge();

    // set a simple speed/angle
    desiredState.speedMetersPerSecond = dc.getVelocityX() * 5.0;   // +/- 5 ft/s for testing
    desiredState.angle = Rotation2d.fromDegrees(dc.getXYRotation() * 180);  // +/- 180
    NTUpdate();
    // run the module
    module.setDesiredState(desiredState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  private NetworkTable table;
  private NetworkTableEntry nte_angle;
  private NetworkTableEntry nte_velocity;

  void NTConfig() {
    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable("Testing");
    nte_angle = table.getEntry("/"+ module.getNTPrefix() + "/cmd-angle");
    nte_velocity = table.getEntry("/" + module.getNTPrefix() + "/cmd-vel");
  }

  void NTUpdate() {
    if (table == null) return;                   // not initialized, punt
    nte_angle.setDouble(desiredState.angle.getDegrees());
    nte_velocity.setDouble(desiredState.speedMetersPerSecond);
  }

}
