/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavx implements GyroIO {
  private final AHRS gyro;

  public GyroIONavx() {
    gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.positionDeg = -gyro.getYaw(); // degrees
    inputs.velocityDegPerSec = -gyro.getRate(); // degrees per second
  }
}
