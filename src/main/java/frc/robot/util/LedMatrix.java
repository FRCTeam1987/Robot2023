package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.awt.Color;

public class LedMatrix {
    public static short[][][] cone = new short[][][]{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}}, {{0, 0, 0}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {70, 15, 1}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
    public static short[][][] cube = new short[][][] {
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 } },
            { { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, {72,15,90}, {72,15,90}, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, {72,15,90}, {72,15,90}, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
            { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };

    private static AddressableLED MATRIX = new AddressableLED(9);
    private static AddressableLEDBuffer MATRIX_BUFFER = new AddressableLEDBuffer(256);

    public static LedMatrix getInstance() {
        return instance;
    }


    private static LedMatrix instance;
    public LedMatrix() {
        instance = this;
        MATRIX.setLength(MATRIX_BUFFER.getLength());
        MATRIX.setData(MATRIX_BUFFER);
        MATRIX.start();
    }

    public void setEntireRGB(Color color) {
        for (int i = 0; i < MATRIX_BUFFER.getLength(); i++) {
            MATRIX_BUFFER.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }
        updateMatrix();
    }

    public void setEntireOff() {
        for (int i = 0; i < MATRIX_BUFFER.getLength(); i++) {
            MATRIX_BUFFER.setRGB(i, 0,0,0);
        }
        updateMatrix();
    }

    public void setSingularRGB(Color color, int ledId) {
        MATRIX_BUFFER.setRGB(ledId, color.getRed(), color.getGreen(), color.getBlue());
    }

    public void setToDesign(short[][][] design) {
        int i = 0;
        for (short[][] rows : design) {
            for (short[] pixel : rows) {
                MATRIX_BUFFER.setRGB(i, pixel[0], pixel[1], pixel[2]);
                i++;
            }
        }
        updateMatrix();
    }
    public void updateMatrix() {
        MATRIX.setData(MATRIX_BUFFER);
        MATRIX.start();
    }

}

