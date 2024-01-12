package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.NavX;

public class SwerveDrive extends SubsystemBase {

	private static SwerveDrive instance;

	public static synchronized SwerveDrive getInstance() {
		if (instance == null)
			instance = new SwerveDrive();
		return instance;
	}

	private SwerveModule[] modules;
	private NavX gyro;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;

	public SwerveDrive() {

		modules = new SwerveModule[4];
		modules[0] = new SwerveModule(Constants.Swerve.flConfig);
		modules[1] = new SwerveModule(Constants.Swerve.frConfig);
		modules[2] = new SwerveModule(Constants.Swerve.blConfig);
		modules[3] = new SwerveModule(Constants.Swerve.brConfig);

		gyro = new NavX(I2C.Port.kMXP);

		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry = new SwerveDriveOdometry(kinematics, gyro.getUnwrappedAngle(), getModulePositions());

	}

	@Override
	public void periodic() {
		odometry.update(gyro.getUnwrappedAngle(), getModulePositions());
	}

	/*
	 * MUTATORS?
	 */

	public void driveRobotCentric(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
				discretize(targetChassisSpeeds));
		SwerveDriveKinematics.desaturateWheelSpeeds(
				states,
				Constants.Swerve.MAX_LINEAR_SPEED_METERS_PER_SECOND);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	public void zeroGyro() {
		gyro.zero();
	}

	/**
	 * Fixes situation where robot drifts in the direction it's rotating in if
	 * turning and translating at the same time
	 *
	 * @see <a href=
	 *      "https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">Chief
	 *      Delphi</a>
	 */
	private ChassisSpeeds discretize(
			ChassisSpeeds originalChassisSpeeds) {
		double vx = originalChassisSpeeds.vxMetersPerSecond;
		double vy = originalChassisSpeeds.vyMetersPerSecond;
		double omega = originalChassisSpeeds.omegaRadiansPerSecond;
		double dt = 0.02; // This should be the time these values will be used, so normally just the loop
											// time
		Pose2d desiredDeltaPose = new Pose2d(
				vx * dt,
				vy * dt,
				new Rotation2d(omega * dt));
		Twist2d twist = new Pose2d().log(desiredDeltaPose);
		return new ChassisSpeeds(
				twist.dx / dt,
				twist.dy / dt,
				twist.dtheta / dt);
	}

	/*
	 * ACCESSORS
	 */

	private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getModuleLocations();
		}
		return translations;
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {
		return gyro.getAngle();
	}

}
