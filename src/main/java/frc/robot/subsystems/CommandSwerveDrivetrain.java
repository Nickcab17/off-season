package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * CTRE Phoenix 6 swerve drivetrain subsystem.
 * Includes PathPlanner AutoBuilder integration.
 */
public class CommandSwerveDrivetrain
        extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.004;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /*
     * Blue alliance sees forward as 0 degrees.
     * Red alliance sees forward as 180 degrees.
     */
    private static final Rotation2d kBlueAlliancePerspectiveRotation =
            Rotation2d.kZero;

    private static final Rotation2d kRedAlliancePerspectiveRotation =
            Rotation2d.k180deg;

    private boolean m_hasAppliedOperatorPerspective = false;

    /*
     * Dedicated request for PathPlanner.
     * Does not replace the existing teleop swerve requests.
     */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();

    /*
     * Swerve requests for SysId characterization.
     */
    private final SwerveRequest.SysIdSwerveTranslation
            m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    private final SwerveRequest.SysIdSwerveSteerGains
            m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    private final SwerveRequest.SysIdSwerveRotation
            m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for translation.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(4),
                            null,
                            state -> SignalLogger.writeString(
                                    "SysIdTranslation_State",
                                    state.toString()
                            )
                    ),
                    new SysIdRoutine.Mechanism(
                            output -> setControl(
                                    m_translationCharacterization
                                            .withVolts(output)
                            ),
                            null,
                            this
                    )
            );

    /*
     * SysId routine for steering.
     */
    private final SysIdRoutine m_sysIdRoutineSteer =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,
                            Volts.of(7),
                            null,
                            state -> SignalLogger.writeString(
                                    "SysIdSteer_State",
                                    state.toString()
                            )
                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> setControl(
                                    m_steerCharacterization
                                            .withVolts(volts)
                            ),
                            null,
                            this
                    )
            );

    /*
     * SysId routine for rotation.
     */
    private final SysIdRoutine m_sysIdRoutineRotation =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.of(Math.PI / 6).per(Second),
                            Volts.of(Math.PI),
                            null,
                            state -> SignalLogger.writeString(
                                    "SysIdRotation_State",
                                    state.toString()
                            )
                    ),
                    new SysIdRoutine.Mechanism(
                            output -> {
                                setControl(
                                        m_rotationCharacterization
                                                .withRotationalRate(
                                                        output.in(Volts)
                                                )
                                );

                                SignalLogger.writeDouble(
                                        "Rotational_Rate",
                                        output.in(Volts)
                                );
                            },
                            null,
                            this
                    )
            );

    /*
     * Select the SysId routine to apply.
     */
    private SysIdRoutine m_sysIdRoutineToApply =
            m_sysIdRoutineTranslation;

    /*
     * Constructor without an explicit odometry frequency.
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /*
     * Constructor with an odometry update frequency.
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                modules
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /*
     * Constructor with odometry and vision standard deviations.
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /*
     * Configure PathPlanner AutoBuilder.
     */
    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    // Current robot pose.
                    () -> getState().Pose,

                    // Reset pose when an autonomous path starts.
                    this::resetPose,

                    // Current robot-relative chassis speeds.
                    () -> getState().Speeds,

                    // Apply PathPlanner output to the CTRE swerve.
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds
                                    .withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(
                                            feedforwards
                                                    .robotRelativeForcesXNewtons()
                                    )
                                    .withWheelForceFeedforwardsY(
                                            feedforwards
                                                    .robotRelativeForcesYNewtons()
                                    )
                    ),

                    // Holonomic controller PID constants.
                    new PPHolonomicDriveController(
                            new PIDConstants(4.0, 0.0, 0.5),
                            new PIDConstants(3.0, 0.0, 0.5)
                    ),

                    // Robot configuration from PathPlanner GUI.
                    config,

                    // Flip the path for the red alliance.
                    () -> DriverStation
                            .getAlliance()
                            .orElse(Alliance.Blue) == Alliance.Red,

                    // Subsystem requirement.
                    this
            );

        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder: "
                            + ex.getMessage(),
                    ex.getStackTrace()
            );
        }
    }

    /*
     * Apply the specified CTRE swerve request.
     * Existing teleop control method is preserved.
     */
    public Command applyRequest(
            Supplier<SwerveRequest> request
    ) {
        return run(() -> setControl(request.get()));
    }

    /*
     * Run a SysId quasistatic test.
     */
    public Command sysIdQuasistatic(
            SysIdRoutine.Direction direction
    ) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /*
     * Run a SysId dynamic test.
     */
    public Command sysIdDynamic(
            SysIdRoutine.Direction direction
    ) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /*
     * Periodically apply the correct operator perspective.
     */
    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective
                || DriverStation.isDisabled()) {

            DriverStation.getAlliance().ifPresent(allianceColor -> {

                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation
                );

                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /*
     * Start the simulation update thread.
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime =
                    Utils.getCurrentTimeSeconds();

            double deltaTime =
                    currentTime - m_lastSimTime;

            m_lastSimTime = currentTime;

            updateSimState(
                    deltaTime,
                    RobotController.getBatteryVoltage()
            );
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /*
     * Add a vision measurement with the correct timestamp conversion.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds
    ) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds)
        );
    }

    /*
     * Add a vision measurement with custom standard deviations.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs
        );
    }

    /*
     * Sample the robot pose at a specific timestamp.
     */
    @Override
    public Optional<Pose2d> samplePoseAt(
            double timestampSeconds
    ) {
        return super.samplePoseAt(
                Utils.fpgaToCurrentTime(timestampSeconds)
        );
    }
}