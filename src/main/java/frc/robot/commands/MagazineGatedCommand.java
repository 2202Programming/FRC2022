package frc.robot.commands;

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
    final double magazineSpeed;

    final int MovingTwoInFC = 10;   //moves forward to get 2nd ball into position
    final int SafetyBackupFC = 5;   //back off the flywheels
    final int ConfirmEmptyFC = 8;  //moves backward to check blind spot
    final int AlignFC = 10;         //moves forward to align when both gates have a ball

    final double SlowRotate = 0.5;

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
        MovingBallTwoIn("moving 2-ball in"),
        BackingUp("backing up for safety"),
        AlignTwoBalls("align 2-ball"),
        TwoBalls("two balls ready");

        String name;

        private MagazineState(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }

    boolean spinup_safe = false;
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
    final NetworkTableEntry nte_spinup_safe;
    final NetworkTableEntry nte_state;

    // Constructor
    public MagazineGatedCommand(double magazineSpeed) {
        this.magazine = RobotContainer.RC().magazine; // just get the magazine from RC
        this.intake = RobotContainer.RC().intake;
        this.magazineSpeed = magazineSpeed;

        ejectCmd = new EjectCmd(this);
        feedCmd = new FeedCmd(this);

        //setup all the tables for debugging
        NetworkTable table  = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_lowergate = table.getEntry("/lowerGate");
        nte_uppergate = table.getEntry("/upperGate");
        nte_feed_request = table.getEntry("/feederOn");
        nte_eject_request = table.getEntry("/ejectorOn");
        nte_spinup_safe = table.getEntry("/spinupSafe");
        nte_state = table.getEntry("/state");

        addRequirements(magazine);  //required for a default command
    }

    // Accessor for supporting eject/feed commands
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
        spinup_safe = false;
        // read gates, save as previous values for edge detection
        prev_upper_lg = magazine.upperGateBlocked();
        prev_lower_lg = magazine.lowerGateBlocked();

        // sort out gates and ball positions for state machine
        state = (prev_lower_lg) ? MagazineState.OneBall_Lower : MagazineState.Empty;
        if (prev_upper_lg) {
            state = (state == MagazineState.Empty) ? MagazineState.OneBall_Upper : MagazineState.AlignTwoBalls;
        }
        // if we didn't find anything, still need to check the blind zone
        state = (state == MagazineState.Empty) ? MagazineState.ConfirmEmpty : state;
       
        if(state == MagazineState.ConfirmEmpty) {
             frame_count_down = ConfirmEmptyFC;
        }
        if (state == MagazineState.AlignTwoBalls) {
            frame_count_down = AlignFC;
        }
    }

    public void execute() {
        boolean lower_lg = magazine.lowerGateBlocked();
        boolean upper_lg = magazine.upperGateBlocked();

        // handle driver requests or execute state machine
        if (feed_request) {
            spinup_safe = true;
            magazine.driveWheelOn(magazineSpeed);
        } else if (eject_request) {
            spinup_safe = false;
            magazine.driveWheelOn(-magazineSpeed);
        } else // Run the normal handler
            switch (state) {
                case ConfirmEmpty:
                    // back up for a bit to make sure we don't have a ball
                    magazine.driveWheelOn(-magazineSpeed);
                    if (--frame_count_down <= 0 || lower_lg) {
                        magazine.driveWheelOff();
                        // goto empty either way, lg will move to next state
                        state = MagazineState.Empty;
                    }
                    spinup_safe = false;
                    break;

                case Empty:
                    // Looking for ball to trigger lower gate, 
                    if (lower_lg && prev_lower_lg) {
                        state = MagazineState.OneBall_Lower;
                    }
                    spinup_safe = false;
                    break;

                case OneBall_Lower:
                    spinup_safe = true;
                    if (lower_lg) {
                        state = MagazineState.MovingToUpper;
                        magazine.driveWheelOn(magazineSpeed*SlowRotate);
                    } else {
                        // must have bounced out???
                        state = MagazineState.ConfirmEmpty;
                    }
                    break;

                case MovingToUpper:
                    spinup_safe = true;
                    // Run the mag until the ball hits the upper limit, then next state
                    // run mag for 1 extra frame so prev and current match
                    if (upper_lg) {
                        magazine.driveWheelOff();
                        state = MagazineState.OneBall_Upper;
                    }
                    break;

                case OneBall_Upper:
                    spinup_safe = true;
                    // Now we are looking for the 2nd ball to be trigger
                    if (lower_lg & prev_lower_lg) {
                        spinup_safe = false;
                        magazine.driveWheelOn(magazineSpeed);
                        state = MagazineState.MovingBallTwoIn;
                        frame_count_down = MovingTwoInFC;
                    }
                    break;

                case MovingBallTwoIn:
                    // moving second ball into magazine
                    spinup_safe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        state = MagazineState.BackingUp;
                        frame_count_down = SafetyBackupFC;
                        magazine.driveWheelOn(-magazineSpeed);
                    }
                    break;

                case BackingUp:
                    spinup_safe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        state = MagazineState.TwoBalls;
                    }
                    break;

                case AlignTwoBalls:
                    // move balls up to flywheel
                    spinup_safe = false;
                    magazine.driveWheelOn(magazineSpeed*SlowRotate);
                    if (--frame_count_down <= 0) {
                        //done moving forward, next backup
                        magazine.driveWheelOn(-magazineSpeed);
                        frame_count_down = SafetyBackupFC;
                        state = MagazineState.BackingUp;
                    }
                    break;

                case TwoBalls:
                    spinup_safe = true;
                    intake.off();
                    intake.retract();
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
        eject_request = false;
        initialize(); // read gates and shutoff feeder
    }

    // Safe when we have cargo and are not on the flywheels
    public boolean safeToSpinUp() {
        return spinup_safe;
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
        nte_spinup_safe.setBoolean(spinup_safe);
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