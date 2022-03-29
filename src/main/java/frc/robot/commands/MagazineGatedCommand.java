package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.MovePositioner.PositionerMode;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

/**
 * 
 * Upper LG is triggered when a ball is near the flywheels but
 * not close to touching them. There should be a bit of rotation left
 * to move to flywheel and back up.
 * 
 * Lower LG is triggered when the ball just hits the base of the magazine.
 * Sometimes the ball will rest on base and not trigger the gate (maybe move the
 * gate?).
 * 
 * It is possible to move one ball from the lower lg to a non-gated zone
 * where the upper lg and lower lg are both off. This must be avoided for the
 * starting position.
 * 
 * External Driver Events:
 * Eject
 * FeedForShooting
 * 
 * Goals:
 * report count
 * keep balls off flywheels
 * simplify driver workload
 * 
 */

public class MagazineGatedCommand extends CommandBase implements MagazineController {
    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final DoubleSupplier magazineSpeed;

    final int MovingTwoCountFC = 20; // frames *.02 = .4 sec
    final int SafetyBackupFC = 5; // frames *02 = .1 sec
    final int ConfirmEmptyFC = 20; // frame

    boolean prev_lower_lg;
    boolean prev_upper_lg;
    int frame_count_down; // used for open loop position, stop on zero.

    int ball_count;

    public enum MagazineState {
        ConfirmEmpty("confirming empty"), // ball in deadzone?
        Empty("empty"),
        OneBall_Lower("1-ball-lower"),
        MovingToUpper("move 1-ball to upper"),
        OneBall_Upper("one ball in upper"),
        // OneBall_NoGate,
        MovingBallTwoIn("moving 2-ball in"),
        BackingUp("backing up for safety"),
        TwoBalls("two balls ready");

        String name;

        private MagazineState(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }

    boolean SpinUpSafe = false;
    boolean feed_request = false; // external event
    boolean eject_request = false; // external event

    MagazineState state = MagazineState.Empty;

    final Command ejectCmd;
    final Command feedCmd;
    final Command movePositionerHigh = new MovePositioner(PositionerMode.High);

    //Network reporting
    final String NT_Name = "/Magazine-Controller";
    final NetworkTableEntry nte_lowergate;
    final NetworkTableEntry nte_uppergate;
    final NetworkTableEntry nte_feed_request;
    final NetworkTableEntry nte_eject_request;
    final NetworkTableEntry nte_state;


    // Constructor
    public MagazineGatedCommand(DoubleSupplier magazineSpeedFunction) {
        this.magazine = RobotContainer.RC().magazine; // just get the magazine from RC
        this.intake = RobotContainer.RC().intake;
        this.magazineSpeed = magazineSpeedFunction;

        ejectCmd = new EjectCmd(this);
        feedCmd = new FeedCmd(this);

        //setup all the tables for debugging
        NetworkTable table  = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_lowergate = table.getEntry("/lowerGate");
        nte_uppergate = table.getEntry("/upperGate");
        nte_feed_request = table.getEntry("/feederOn");
        nte_eject_request = table.getEntry("/ejectorOn");
        nte_state = table.getEntry("/state");
    }

    public Command getEjectCmd() {
        return this.ejectCmd;
    }

    public Command getFeedCmd() {
        return this.feedCmd;
    }

    /**
     * Called on scheduling or whenever we need to figure out the state of the
     * cargo we are carrying, like after feed or eject request.
     */
    public void initialize() {
        SpinUpSafe = false;
        // read gates, save as previous values for edge detection
        prev_upper_lg = magazine.upperGateBlocked();
        prev_lower_lg = magazine.lowerGateBlocked();

        // sort out gates and ball positions for state machine
        state = (prev_lower_lg) ? MagazineState.OneBall_Lower : MagazineState.Empty;
        if (prev_upper_lg) {
            state = (state == MagazineState.Empty) ? MagazineState.OneBall_Upper : MagazineState.TwoBalls;
        }
        // if we didn't find anything, still need to check the blind zone
        state = (state == MagazineState.Empty) ? MagazineState.ConfirmEmpty : state;
        frame_count_down = ConfirmEmptyFC;
    }

    public void execute() {
        boolean lower_lg = magazine.lowerGateBlocked();
        boolean upper_lg = magazine.upperGateBlocked();

        // handle driver requests or execute state machine
        if (feed_request) {
            magazine.driveWheelOn(magazineSpeed.getAsDouble());
        } else if (eject_request) {
            magazine.driveWheelOn(-magazineSpeed.getAsDouble());
        } else // Run the normal handler
            switch (state) {
                case ConfirmEmpty:
                    // back up for a bit to make sure we don't have a ball
                    magazine.driveWheelOn(-magazineSpeed.getAsDouble());
                    if (frame_count_down <= 0 || lower_lg) {
                        magazine.driveWheelOff();
                        // goto empty either way, lg will move to next state
                        state = MagazineState.Empty;
                    }
                    SpinUpSafe = false;
                    break;

                case Empty:
                    // Looking for ball to trigger lower gate, 
                    if (lower_lg && prev_lower_lg) {
                        state = MagazineState.OneBall_Lower;
                    }
                    SpinUpSafe = false;
                    break;

                case OneBall_Lower:
                    SpinUpSafe = true;
                    if (lower_lg) {
                        state = MagazineState.MovingToUpper;
                        magazine.driveWheelOn(magazineSpeed.getAsDouble());
                    } else {
                        // must have bounced out???
                        state = MagazineState.ConfirmEmpty;
                    }
                    break;

                case MovingToUpper:
                    SpinUpSafe = true;
                    // Run the mag until the ball hits the upper limit, then next state
                    // run mag for 1 extra frame so prev and current match
                    if (upper_lg && prev_upper_lg) {
                        magazine.driveWheelOff();
                        state = MagazineState.OneBall_Upper;
                    }
                    break;

                case OneBall_Upper:
                    SpinUpSafe = true;
                    // Now we are looking for the 2nd ball to be trigger
                    if (lower_lg & prev_lower_lg) {
                        SpinUpSafe = false;
                        magazine.driveWheelOn(magazineSpeed.getAsDouble());
                        state = MagazineState.MovingBallTwoIn;
                        frame_count_down = MovingTwoCountFC;
                    }
                    break;

                case MovingBallTwoIn:
                    SpinUpSafe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        state = MagazineState.BackingUp;
                        frame_count_down = SafetyBackupFC;
                        magazine.driveWheelOn(-magazineSpeed.getAsDouble());
                    }
                    break;

                case BackingUp:
                    SpinUpSafe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        state = MagazineState.TwoBalls;
                    }

                case TwoBalls:
                    SpinUpSafe = true;
                    intake.off();
                    CommandScheduler.getInstance().schedule(movePositionerHigh);
                    break;

                default:
                    break;
            }

        // save prev
        prev_lower_lg = lower_lg;
        prev_upper_lg = upper_lg;
        updateNT();
    }

    // External Event Handling
    public void feederOn() {
        feed_request = true;
        eject_request = false;
    }

    public void feederOff() {
        feed_request = false;
        initialize(); // redo gates and shutoff feeder
    }

    public void ejectOn() {
        eject_request = true;
        feed_request = false;
    }

    public void ejectOff() {
        eject_request = true;
        initialize(); // read gates and shutoff feeder
    }

    // Safe when we have cargo and are not on the flywheels
    public boolean safeToSpinUp() {
        return SpinUpSafe;
    }

    // This command never really ends, it always runs to manage the cargo
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        magazine.driveWheelOff();
    }

    void updateNT() {
        nte_lowergate.setBoolean(prev_lower_lg);
        nte_uppergate.setBoolean(prev_upper_lg);
        nte_feed_request.setBoolean(feed_request);
        nte_eject_request.setBoolean(eject_request);
        nte_state.setString(state.toString());
    }

    /**
     * Default on/off commands for Driver Events.
     */
    class EjectCmd extends CommandBase {
        MagazineController controller;

        EjectCmd(MagazineController c) {
            controller = c;
            setName("EjectCommand");
        }

        @Override
        public void initialize() {
            controller.ejectOn();
        }

        @Override
        public void end(boolean interrupted) {
            controller.ejectOff();
        }
    }

    class FeedCmd extends CommandBase {
        MagazineController controller;

        FeedCmd(MagazineController c) {
            controller = c;
            setName("FeedCommand");
        }

        @Override
        public void initialize() {
            controller.feederOn();
        }

        @Override
        public void end(boolean interrupted) {
            controller.feederOff();
        }

    }
}
