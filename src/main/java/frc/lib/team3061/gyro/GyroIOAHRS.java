/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroIOAHRS implements GyroIO {
  private final AHRS gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOAHRS() {
    gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.positionDeg = gyro.getRotation2d().getDegrees(); // degrees
    inputs.pitchDeg = gyro.getPitch(); // degrees
    inputs.velocityDegPerSec = gyro.getRate(); // degrees per second
  }
}
