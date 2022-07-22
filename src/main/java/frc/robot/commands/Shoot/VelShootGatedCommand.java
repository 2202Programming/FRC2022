package frc.robot.commands.Shoot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Shooter;
import frc.robot.commands.MagazineController;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

public class VelShootGatedCommand extends VelShootCommand {

    final MagazineController mag_ctrl;

    // use own network tables
    final String NT_Name = "ShooterGated";
    final NetworkTableEntry nte_ballVel;
    final NetworkTableEntry nte_shooterState;
    final NetworkTableEntry nte_distance;
    final NetworkTableEntry nte_outOfRange;

    final NetworkTableEntry nte_interceptMultiplier;
    final NetworkTableEntry nte_farMultiplier;
    final NetworkTableEntry nte_farDistance;

    // Gated Stages are different.
    enum GStage {
        WaitingForFlyWheel("Waiting for flywheel"),
        WaitingOnMag("MagCtrl Not ready"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("Waiting for Solution"),
        Shooting("Shooting");

        String name;

        private GStage(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    GStage stage;

    /**
     * VelShootGatedCommand - uses magazine with lightgates. Details of magazine
     * ball management is
     * completely inside the magazineController.
     * 
     * @param shooterSettings  container for shooter speeds/angle
     * @param mag_ctrl         interface for controlling magazine
     * @param solutionProvider interface for target tracking
     * @param autoVelocity     calculates and controls shooter velocity based on
     *                         distance
     */

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl,
            SolutionProvider solutionProvider, boolean autoVelocity) {
        super(shooterSettings, 0, solutionProvider);
        this.mag_ctrl = mag_ctrl;
        this.autoVelocity = autoVelocity;

        cmdSS = new ShooterSettings(m_shooterSettings);

        getRequirements().remove(magazine); // using mag_ctrl, not magazine directly

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_ballVel = table.getEntry("/VelGateShootCmd/BallVel");
        nte_shooterState = table.getEntry("/VelGateShootCmd/ShooterState");
        nte_distance = table.getEntry("/VelGateShootCmd/Distance");
        nte_outOfRange = table.getEntry("/VelGateShootCmd/OutOfRange");

        // NTEs that will be changed
        //
        // NOTE: for these values to appear in NT, they need to be set, and since
        // resources should not be wasted setting these NTEs in periodic (and they only
        // need to be set once), they are set here.
        //
        // NOTE 2: Note the division occurring. Those are the original values set, so
        // the remaining values are just the multiplier.
        nte_interceptMultiplier = table.getEntry("/VelGateShootCmd/InterceptMultiplier");
        nte_interceptMultiplier.setDouble(Shooter.INTERCEPT_MULTIPLIER);
        nte_farMultiplier = table.getEntry("/VelGateShootCmd/FarMultiplier");
        nte_farMultiplier.setDouble(Shooter.FAR_SLOPE_MULTIPLIER);
        nte_farDistance = table.getEntry("/VelGateShootCmd/FarDistance");
        nte_farDistance.setDouble(Shooter.FAR_DISTANCE);
    }

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl) {
        this(shooterSettings, mag_ctrl, null, false);
    }

    public VelShootGatedCommand(double requestedVelocity, MagazineController mag_ctrl) {
        this(new ShooterSettings(requestedVelocity, 0.0, 0.0, 0.1), mag_ctrl, null, false);
    }

    public VelShootGatedCommand(MagazineController mag_ctrl, SolutionProvider solutionProvider) {
        // CHANGE HERE FOR SPIN ADJUSTMENT
        this(new ShooterSettings(20, -5.0, 0.0, 0.1), mag_ctrl, solutionProvider, true); // negative is front spin
    }

    @Override
    public void initialize() {
        stage = GStage.WaitingOnMag;
        // should be off already shooter.off();
    }

    @Override
    public void execute() {
        // thes methods have side-effect flags
        calculateDistance(); // sets currentDistance
        calculateVelocity(); // sets outOfRange and calculatedVelocity

        // if autovelocity is true will calculate a new RPM speed based on the distance
        // and adjust positioner
        // otherwise RPMs should be constant based on the constructor parameters
        if (autoVelocity) {
            if (mag_ctrl.safeToSpinUp() && (calculatedVel != cmdSS.vel)) {
                cmdSS.vel = calculatedVel; // shouldn't have to create new object, just change vel
                shooter.spinup(cmdSS);
            }
        }

        switch (stage) {
            case WaitingOnMag:
                if (mag_ctrl.safeToSpinUp()) {
                    stage = GStage.WaitingForFlyWheel;
                    intake.off();
                    shooter.spinup(cmdSS); // spin shooter up
                }
                break;

            case WaitingForFlyWheel:
                if (shooter.isReadyToShoot()) {
                    stage = GStage.WaitingForSolution;
                }
                break;

            case WaitingForSolution:
                if (solutionProvider.isOnTarget()) {
                    stage = GStage.Shooting;
                    mag_ctrl.feederOn();
                    intake.on(0.0, 0.2);
                }
                break;

            case Shooting:
                if (!shooter.isReadyToShoot()) {
                    mag_ctrl.feederOff();
                    intake.off();
                    shooter.spinup(cmdSS); // in case a new velocity has been set due to a new distance
                    stage = GStage.WaitingForFlyWheel;
                }
                break;
            default:
                break;
        }

        NTupdates();
    }

    @Override
    public void end(boolean interrupted) {
        // useful in case if somebody forgets to write values down
        System.out.println("DEBUG -- constants used: Intercept Multiplier = " + nte_interceptMultiplier.getDouble(1)
                + ", Far Distance = " + nte_farDistance.getDouble(5)
                + ", Far Multiplier = " + nte_farMultiplier.getDouble(1));
        System.out.println(
                "WARNING: These values will not be sasved once the robot is shut down! Make sure to write values down and, if satisfactory, to change Constants.java.");

        mag_ctrl.feederOff();
        intake.off();
        shooter.off();
    }

    @Override
    public boolean isFinished() {
        return (mag_ctrl.getBallCount() <= 0);
    }

    // Only difference between this and super is the incorporation of NT
    @Override
    public void calculateVelocity() {
        // retriev NT values for latest inputs
        double interceptMultiplier = nte_interceptMultiplier.getDouble(1);
        double farDistance = nte_farDistance.getDouble(5);
        double farMultiplier = nte_farMultiplier.getDouble(1);

        double m_slope = Shooter.SLOPE;
        double m_intercept = Shooter.ORIGINAL_INTERCEPT * interceptMultiplier;

        if (currentDistance > farDistance) {
            m_intercept += (m_slope * farDistance);
            m_slope *= farMultiplier;

            calculatedVel = m_slope * (currentDistance - farDistance) + m_intercept; // distnce vs. velocity
                                                                                     // trendline for long
                                                                                     // range
                                                                                     // positioner
        } else {
            calculatedVel = m_slope * currentDistance + m_intercept; // distnce vs. velocity trendline for long range
                                                                     // positioner
        }

        if (calculatedVel > Shooter.kMaxFPS) {
            outOfRange = true;
            calculatedVel = Shooter.kMaxFPS; // don't ask shooter to go above max FPS otherwise can get stuck waiting
                                             // for impossible goals
        } else {
            outOfRange = false;
        }
    }

    void NTupdates() {
        log_counter++;
        if ((log_counter % 10) == 0) {
            nte_ballVel.setDouble(calculatedVel);
            nte_shooterState.setString(stage.toString());
            nte_distance.setDouble(currentDistance);
            nte_outOfRange.setBoolean(outOfRange);
        }
    }
}
