package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import java.util.Arrays;

public class BatteryTracker {

  /*
   * BATTERY NAMING CONVENTION AND RULES
   * NAME FORMAT: BAT-YEAR-NUM
   * EXAMPLE: BAT-2030-052
   * Website: https://qrcoderw.com/batch_qr_generator.php
   */
  public static final String DEFAULT_NAME = "BAT-0000-000";

  private static final int NAME_LENGTH = 12;
  private static final byte[] scanCommand =
      new byte[] {0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
  private static final byte[] responsePrefix =
      new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
  private static final byte ENDMARK = 0x0d; // CR
  private static final int FULL_RESPONSE_LENGTH = responsePrefix.length + NAME_LENGTH + 1;

  private static String name = DEFAULT_NAME;

  /**
   * Scans the battery. This should be called before the first loop cycle
   *
   * @param timeout The time to wait before giving up
   */
  public static String scanBattery(double timeout) {
    DriverStation.reportWarning("[BatteryTracker] Scanning...", false);
    DriverStation.reportWarning("[BatteryTracker] Robot is real", false);
    DriverStation.reportWarning("[BatteryTracker] Robot is supported", false);
    // Only scan on supported robots and in real mode
    try (SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
      port.setTimeout(timeout);
      port.setWriteBufferSize(scanCommand.length);
      port.setReadBufferSize(FULL_RESPONSE_LENGTH);

      port.write(scanCommand, scanCommand.length);
      byte[] response = port.read(FULL_RESPONSE_LENGTH);

      // Ensure response is correct length
      if (response.length != FULL_RESPONSE_LENGTH) {
        DriverStation.reportWarning(
            "[BatteryTracker] Expected "
                + FULL_RESPONSE_LENGTH
                + " bytes from scanner, got "
                + response.length,
            false);
        return name;
      }

      // Ensure response starts with prefix
      for (int i = 0; i < responsePrefix.length; i++) {
        if (response[i] != responsePrefix[i]) {
          DriverStation.reportWarning(
              "[BatteryTracker] Invalid prefix from scanner.  Got data:", false);
          DriverStation.reportWarning("[BatteryTracker] " + Arrays.toString(response), false);
          return name;
        }
      }

      // Ensure response ends with suffix
      if (response[response.length - 1] != ENDMARK) {
        DriverStation.reportWarning(
            "[BatteryTracker] Invalid suffix from scanner.  Got " + response[response.length - 1],
            false);
      }

      // Read name from data
      byte[] batteryNameBytes = new byte[NAME_LENGTH];
      System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0, NAME_LENGTH);
      name = new String(batteryNameBytes);
      DriverStation.reportWarning("[BatteryTracker] Scanned battery " + name, false);

    } catch (Exception e) {
      DriverStation.reportWarning("[BatteryTracker] Exception while trying to scan battery", false);
      e.printStackTrace();
    }

    return name;
  }

  /** Returns the name of the last scanned battery. */
  public static String getName() {
    return name;
  }
}
