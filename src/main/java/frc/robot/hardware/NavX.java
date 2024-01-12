package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

public class NavX {
    private AHRS ahrs;
	private Rotation2d gyroZero;

	public NavX(I2C.Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = new Rotation2d();
	}

	public NavX(SPI.Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = new Rotation2d();
	}

	public AHRS getAHRS() {
		return ahrs;
	}

	public Rotation2d getUnwrappedAngle() {
		return ahrs.getRotation2d();
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getAngle() {
		return getYaw();
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getOffsetedAngle() {
		return wrapRotation2d(getAngle().minus(getZero()));
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(-ahrs.getYaw());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getPitch() {
		return Rotation2d.fromDegrees(-ahrs.getPitch());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getRoll() {
		return Rotation2d.fromDegrees(-ahrs.getRoll());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getZero() {
		return gyroZero;
	}

	public void zero() {
		gyroZero = getAngle();
	}

	/** @param offset The new angle given by {@link frc.robot.hardware.NavX#getOffsetedAngle() 
	 * getOffsetedAngle()} for the current angle
	 */
	public void zeroWithOffset(Rotation2d offset) {
		gyroZero = wrapRotation2d(getAngle().minus(offset));
	}

    public static Rotation2d wrapRotation2d(Rotation2d rotationToWrap) {
		return Rotation2d.fromRadians(MathUtil.angleModulus(rotationToWrap.getRadians()));
	}
}
