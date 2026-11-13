package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Fuses wheel odometry + gyro with latency-compensated vision measurements.
 *
 * <p>The estimate is tracked together with a per-axis variance (x, y, theta). Odometry steps make
 * the estimate less certain over time; vision measurements make it more certain. Variance is
 * stored in the history buffer so a late vision measurement is weighted by how confident we were
 * at the moment the measurement was taken.
 *
 * <p>All Matrix<N3, N1> values are the diagonal of a 3x3 covariance matrix (x, y, theta), since
 * every axis is treated independently.
 */
public class RobotOdometry {
  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}
 
  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
 
  private static final double HISTORY_SECONDS = 2.0;
 
  private final SwerveDriveKinematics kinematics;
  /** Variance added per second of odometry (x, y, theta). */
  private final Matrix<N3, N1> processVariancePerSecond;
  /** Variance the estimate starts with / resets to in setPose. */
  private final Matrix<N3, N1> initialVariance;
 
  private final OdometryHistory history = new OdometryHistory(HISTORY_SECONDS);
 
  // Current state
  private Pose2d estimatedPose = new Pose2d();
  private Matrix<N3, N1> variance;
 
  // Odometry bookkeeping
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();
  private double lastTimestamp = Double.NaN;
  private Twist2d robotVelocity = new Twist2d();
 
  /**
   * @param kinematics drivetrain kinematics
   * @param odometryStdDevsPerSqrtSec how fast odometry uncertainty grows: std dev gained per
   *     sqrt(second) for (x meters, y meters, theta radians)
   * @param initialStdDevs starting uncertainty for (x, y, theta)
   */
  public RobotOdometry(
      SwerveDriveKinematics kinematics,
      Matrix<N3, N1> odometryStdDevsPerSqrtSec,
      Matrix<N3, N1> initialStdDevs) {
    this.kinematics = kinematics;
    this.processVariancePerSecond = square(odometryStdDevsPerSqrtSec);
    this.initialVariance = square(initialStdDevs);
    this.variance = initialVariance.copy();
  }
 
  // ---------------------------------------------------------------------------------------------
  // Odometry
  // ---------------------------------------------------------------------------------------------
 
  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = toTwist(observation);
 
    // Move the estimate by the same twist the odometry saw.
    estimatedPose = estimatedPose.exp(twist);
 
    // Odometry drifts, so we get less sure of where we are as time passes.
    double dt = Double.isNaN(lastTimestamp) ? 0.0 : observation.timestamp() - lastTimestamp;
    variance = variance.plus(processVariancePerSecond.times(dt));
    lastTimestamp = observation.timestamp();
 
    if (dt > 0.0) {
      robotVelocity = new Twist2d(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
    }
 
    history.addSample(observation.timestamp(), twist, variance);
  }
 
  /** Wheel-based twist, with dtheta replaced by the gyro's if the gyro is connected. */
  private Twist2d toTwist(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
 
    if (observation.gyroAngle() != null) {
      double gyroDelta = observation.gyroAngle().minus(lastGyroAngle).getRadians();
      lastGyroAngle = observation.gyroAngle();
      twist = new Twist2d(twist.dx, twist.dy, gyroDelta);
    }
    return twist;
  }
 
  // ---------------------------------------------------------------------------------------------
  // Vision
  // ---------------------------------------------------------------------------------------------
 
  public void addVisionObservation(VisionObservation observation) {
    double time = observation.timestamp();
 
    Optional<Transform2d> offsetOpt = history.getOffsetToLatest(time);
    Optional<Matrix<N3, N1>> varianceOpt = history.getVarianceAt(time);
    if (offsetOpt.isEmpty() || varianceOpt.isEmpty()) {
      return; // too old, or no odometry yet
    }
    Transform2d sampleToNow = offsetOpt.get();
    Matrix<N3, N1> varianceAtTime = varianceOpt.get();
 
    // Rewind our estimate to when the vision frame was captured.
    Pose2d estimateAtTime = estimatedPose.plus(sampleToNow.inverse());
 
    // Per-axis Kalman gain: how much to trust vision vs. our estimate at that moment.
    Matrix<N3, N1> gains = kalmanGains(varianceAtTime, square(observation.stdDevs()));
 
    // Move the old estimate part of the way toward the vision pose.
    Transform2d error = new Transform2d(estimateAtTime, observation.visionPose());
    Logger.recordOutput("Odometry/VisionTransform", error);
    Matrix<N3, N1> correction =
        gains.elementTimes(
            VecBuilder.fill(error.getX(), error.getY(), error.getRotation().getRadians()));
    Transform2d scaledCorrection =
        new Transform2d(
            correction.get(0, 0),
            correction.get(1, 0),
            Rotation2d.fromRadians(correction.get(2, 0)));
 
    // Replay the odometry that happened since then.
    estimatedPose = estimateAtTime.plus(scaledCorrection).plus(sampleToNow);
 
    // Vision made us more certain: remove K*P, the same amount as at the time of the measurement.
    variance = clampNonNegative(variance.minus(gains.elementTimes(varianceAtTime)));
  }
 
  /** Scalar Kalman gain K = P / (P + R) for each axis. */
  private static Matrix<N3, N1> kalmanGains(Matrix<N3, N1> p, Matrix<N3, N1> r) {
    Matrix<N3, N1> k = new Matrix<>(Nat.N3(), Nat.N1());
    for (int i = 0; i < 3; ++i) {
      double total = p.get(i, 0) + r.get(i, 0);
      k.set(i, 0, total == 0.0 ? 0.0 : p.get(i, 0) / total);
    }
    return k;
  }
 
  // ---------------------------------------------------------------------------------------------
  // Accessors
  // ---------------------------------------------------------------------------------------------
 
  public Pose2d getEstimatedPose2d() {
    return estimatedPose;
  }
 
  public Rotation2d getRotation2d() {
    return estimatedPose.getRotation();
  }
 
  /** Current standard deviation (x, y, theta) of the estimate. Useful for gating vision. */
  public Matrix<N3, N1> getEstimatedStdDevs() {
    Matrix<N3, N1> stdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    for (int i = 0; i < 3; ++i) {
      stdDevs.set(i, 0, Math.sqrt(variance.get(i, 0)));
    }
    return stdDevs;
  }
 
  public void setPose(Pose2d pose) {
    history.reset();
    estimatedPose = pose;
    variance = initialVariance.copy();
  }
 
  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }
 
  // Helpers
  private static Matrix<N3, N1> square(Matrix<N3, N1> m) {
    return m.elementTimes(m);
  }
 
  private static Matrix<N3, N1> clampNonNegative(Matrix<N3, N1> m) {
    Matrix<N3, N1> out = m.copy();
    for (int i = 0; i < 3; ++i) {
      out.set(i, 0, Math.max(0.0, m.get(i, 0)));
    }
    return out;
  }
 
  /**
   * Time-indexed history of raw odometry and the estimate's variance.
   *
   * <p>The odometry pose kept here is never used as a position. It exists only so we can ask "how
   * far has the robot moved since time t?" via {@link #getOffsetToLatest}. Because odometry is
   * relative, that offset applies equally to the fused estimate.
   */
  private static final class OdometryHistory {
    private final double historySeconds;
    private final TimeInterpolatableBuffer<Pose2d> odometryPoses;
    private final TimeInterpolatableBuffer<Matrix<N3, N1>> variances;
 
    private Pose2d latestOdometryPose = new Pose2d();
 
    OdometryHistory(double historySeconds) {
      this.historySeconds = historySeconds;
      this.odometryPoses = TimeInterpolatableBuffer.createBuffer(historySeconds);
      this.variances =
          TimeInterpolatableBuffer.createBuffer(
              (start, end, t) -> start.plus(end.minus(start).times(t)), historySeconds);
    }
 
    /** Advance the internal odometry pose by twist and record it with the variance at this time. */
    void addSample(double timestamp, Twist2d twist, Matrix<N3, N1> variance) {
      latestOdometryPose = latestOdometryPose.exp(twist);
      odometryPoses.addSample(timestamp, latestOdometryPose);
      variances.addSample(timestamp, variance.copy());
    }
 
    /**
     * Transform that takes the robot from where it was at {@code timestamp} to where it is now
     * (robot-relative). Empty if the timestamp is outside the buffer or there is no data.
     */
    Optional<Transform2d> getOffsetToLatest(double timestamp) {
      if (isTooOld(timestamp)) {
        return Optional.empty();
      }
      return odometryPoses
          .getSample(timestamp)
          .map(pastPose -> new Transform2d(pastPose, latestOdometryPose));
    }
 
    /** Variance of the estimate at {@code timestamp}. */
    Optional<Matrix<N3, N1>> getVarianceAt(double timestamp) {
      if (isTooOld(timestamp)) {
        return Optional.empty();
      }
      return variances.getSample(timestamp);
    }
 
    void reset() {
      odometryPoses.clear();
      variances.clear();
      latestOdometryPose = new Pose2d();
    }
 
    private boolean isTooOld(double timestamp) {
      try {
        return odometryPoses.getInternalBuffer().lastKey() - historySeconds > timestamp;
      } catch (NoSuchElementException ex) {
        return true; // empty buffer
      }
    }
  }
}