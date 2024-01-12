package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.hardware.SparkMax;

public class SwerveModule {

    // Each Module Consists of 2 Motors (1 Drive and 1 Angle)
    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private Translation2d location;

    /**
     * Each individual module with 2 motors (Drive and Angle)
     * 
     * @param config The settings for the module.
     */
    public SwerveModule(ModuleConfig config) {

        driveMotor = new SparkMax(config.driveId, MotorType.kBrushless);
        angleMotor = new SparkMax(config.angleId, MotorType.kBrushless);

        driveMotor.configPID(config.driveP, config.driveI, config.driveD);
        angleMotor.configPID(config.angleP, config.angleI, config.angleD);

        driveMotor.setInverted(config.driveInverted);
        angleMotor.setInverted(config.angleInverted);

        driveMotor.configCurrentLimit(config.driveCurrentLimit);
        angleMotor.configCurrentLimit(config.angleCurrentLimit);

        this.location = config.location;
    }

    /*
     * MUTATORS, "Setters"
     */

    public void drive(SwerveModuleState initialTargetState) {
        SwerveModuleState targetState = optimizeModuleState(
            initialTargetState, 
            getModuleState().angle,
            angleMotor.hasContinuousRotation()
        );
        setVelocity(
            targetState.speedMetersPerSecond * 
            // Scale velocity by how far wheel is from target
            Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
        );
        setAngle(targetState.angle);
    }

    public void setAngle(Rotation2d targetAngle) {
        angleMotor.setAngle(
            new Rotation2d(
                targetAngle.getRadians() / Constants.Swerve.ANGLE_RATIO
            )
        );
    }

    /**
     * @param targetVelocity in m/s
     */
    public void setVelocity(double targetVelocity) {
        driveMotor.setAngularVelocity(
            new Rotation2d(
                targetVelocity * 2 /
                (Constants.Swerve.DRIVE_RATIO 
                * Constants.Swerve.WHEEL_DIAMETER_METERS)
            )
        );
    }

    /**
	 * @param desiredState the target state of the module
	 * @param currentAngle the current angle of the module
	 * @param continuousRotation whether the encoder of the angle motor of the module 
	 * supports continous rotation
	 * @see <a
	 *      href=https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/3
	 *      >Chief Delphi Post Concerning The Issue</a>
	 */
	public static SwerveModuleState optimizeModuleState(
		SwerveModuleState desiredState,
		Rotation2d currentAngle,
		boolean continuousRotation
	) {
		if (continuousRotation) {
			return SwerveModuleState.optimize(desiredState, currentAngle);
		}
		double originalAngle = currentAngle.getDegrees();
		double delta = MathUtil.inputModulus(
			desiredState.angle.getDegrees() - originalAngle + 180, 0, 360
		) - 180;
		if (Math.abs(delta) > 90) {
			return new SwerveModuleState(
				-desiredState.speedMetersPerSecond,
				Rotation2d.fromDegrees(
					originalAngle + delta - Math.signum(delta) * 180
				)
			);
		}
		return new SwerveModuleState(
			desiredState.speedMetersPerSecond,
			Rotation2d.fromDegrees(originalAngle + delta)
		);
	}

    /*
     * ACCESORS
     */

    /**
     * @return The SwerveModuleState :)
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
			driveMotor.getAngularVelocity().getRadians() *
			Constants.Swerve.DRIVE_RATIO *
			Constants.Swerve.WHEEL_DIAMETER_METERS /
			2,
			new Rotation2d(
                angleMotor.getAngle().getRadians() 
                * Constants.Swerve.ANGLE_RATIO
            )
		);
    }

    /**
     * @return The SwerveModulePosition :)
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
			driveMotor.getAngle().getRadians() /
			(2 * Math.PI) * 
			Constants.Swerve.DRIVE_RATIO *
			Constants.Swerve.WHEEL_DIAMETER_METERS *
			Math.PI,
			getModuleState().angle
		);
    }

    public double getAngularVelocity() {
		return angleMotor.getAngularVelocity().getRadians() 
                * Constants.Swerve.ANGLE_RATIO;
	}

    public Translation2d getModuleLocations() {
        return location;
    }

    /**
     * Set all valued before passing into Swerve Module
     */
    class ModuleConfig {
        public ModuleConfig() {}

        public double angleP = 0.0020645;
        public double angleI = 0.00001;
        public double angleD = 0;

        public double driveP = 0.0020645; // TODO: Check
        public double driveI = 0;
        public double driveD = 0;

        public int driveId;
        public int angleId;

        public int driveCurrentLimit = 10;
        public int angleCurrentLimit = 20;

        public Translation2d location;

        public boolean driveInverted;
        public boolean angleInverted;
    }
}
