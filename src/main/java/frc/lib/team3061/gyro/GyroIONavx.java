/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavx implements GyroIO {
  private final AHRS gyro;

  public GyroIONavx() {
    gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.positionDeg = gyro.getRotation2d().getDegrees(); // degrees
    inputs.velocityDegPerSec = gyro.getRate(); // degrees per second
  }
}
