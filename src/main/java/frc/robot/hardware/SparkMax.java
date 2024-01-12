package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMax extends CANSparkMax {

  public SparkMax(int id, MotorType motorType) {
    super(id, motorType);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getEncoder().getPosition());
  }

  public void setAngle(Rotation2d position) {
    getPIDController()
        .setReference(position.getRotations(), ControlType.kPosition);
  }

  public void setOutput(double output) {
    set(output);
  }

  public double getOutput() {
    return get();
  }

  public Rotation2d getAngularVelocity() {
    return Rotation2d.fromRotations(getEncoder().getVelocity());
  }

  public void setAngularVelocity(Rotation2d velocity) {
    getPIDController()
      .setReference(velocity.getRotations(), ControlType.kVelocity);
  }

  public boolean hasContinuousRotation() {
    return true; // Means we can use WPILIB optimize
  }

  public SparkMax configCurrentLimit(int currentLimit) {
    setSmartCurrentLimit(currentLimit);
    return this;
  }

  public SparkMax configPID(double kP, double kI, double kD) {
    SparkPIDController controller = getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    return this;
  }

  public SparkMax configMinAngle(Rotation2d minPosition) {
    setSoftLimit(
      SoftLimitDirection.kReverse,
      (float) minPosition.getRotations()
    );
    return this;
  }

  public SparkMax configMaxAngle(Rotation2d maxPosition) {
    setSoftLimit(
      SoftLimitDirection.kForward,
      (float) maxPosition.getRotations()
    );
    return this;
  }

  public SparkMax configMinOutput(double minOutput) {
    SparkPIDController controller = getPIDController();
    controller.setOutputRange(minOutput, controller.getOutputMax());
    return this;
  }

  public SparkMax configMaxOutput(double maxOutput) {
    SparkPIDController controller = getPIDController();
    controller.setOutputRange(controller.getOutputMin(), maxOutput);
    return this;
  }

  public SparkMax configInverted(boolean shouldInvert) {
    super.setInverted(shouldInvert);
    return this;
  }

  public SparkMax configBrakeOnIdle(boolean shouldBreak) {
    setIdleMode(shouldBreak ? IdleMode.kBrake : IdleMode.kCoast);
    return this;
  }
}
