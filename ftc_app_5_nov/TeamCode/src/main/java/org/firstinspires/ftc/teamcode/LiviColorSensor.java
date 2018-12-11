package org.firstinspires.ftc.teamcode;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LiviColorSensor {
    private DataLogged dl;
    private Wire cs;
    private int readCount = 0;
    private long timeStamp;              // In microseconds
    private int  clear, red, green, blue;

    public void init() {
        dl = new DataLogger("CS Test");

        dl.addField("Micros");      // Sensor reading time in microseconds
        dl.addField("Clr");
        dl.addField("Red");
        dl.addField("Blue");
        dl.addField("Green");
        dl.newLine();

        initColorSensor();
    }

    public void init_loop() {
        if (cs.responseCount() > 0) {
            cs.getResponse();
            int regNumber = cs.registerNumber();
            if (cs.isRead()) {
                int regValue    = cs.read();
                telemetry.addData("Read  " + regNumber, regValue);
                Log.i("GST init", String.format("Read reg 0x%02X = 0x%02X", regNumber, regValue));
            }
            if (cs.isWrite()) {
                int regValue    = cs.read();
                telemetry.addData("Write " + regNumber, regValue);
                Log.i("GST init",String.format("Write reg 0x%02X = 0x%02X",regNumber,regValue));
            }
        }
    }

    public void start() {
        startColorPolling();
    }

    @Override
    public void loop() {
        if (isColorUpdate()) {
            dl.addField(timeStamp);
            dl.addField(clear);
            dl.addField(red);
            dl.addField(green);
            dl.addField(blue);
            dl.newLine();

            readCount++;
            telemetry.addData("Count",  readCount);
            telemetry.addData("Time",   timeStamp/1e6);
            telemetry.addData("Colors", "C:"+ clear +
                    " R:" + red +
                    " G:" + green +
                    " B:" + blue);
        }
    }

    public void stop() {
        dl.closeDataLogger();
        cs.close();
    }

    private void initColorSensor() {
        cs = new Wire(hardwareMap,"Color",2*0x29);

        cs.write(0x80,0x03);                // R[00] = 3    to enable power
        cs.requestFrom(0x92, 1);            // R[12]        is the device ID
        cs.write(0x8F,0x02);                // R[0F] = 2    to set gain 16
        cs.write(0x81,0xEC);                // R[01] = EC   to set integration time to 20* 2.4 ms
        // 256 - 20 = 236 = 0xEC
    }

    private void startColorPolling() {
        cs.requestFrom(0x93, 1);            // Get sensor status
    }

    private boolean isColorUpdate() {
        boolean isNew = false;
        if (cs.responseCount() > 0) {
            cs.getResponse();
            int regNumber = cs.registerNumber();
            if (cs.isRead()) {
                int regCount = cs.available();
                switch (regNumber) {
                    case 0x93:
                        if (regCount == 1) {
                            int status = cs.read();
                            if ((status & 1) != 0) {
                                cs.requestFrom(0x94,8);             // Get colors
                            } else {
                                cs.requestFrom(0x93,1);             // Keep polling
                            }
                        } else {
                            telemetry.addData("Error", regNumber + " length 1 != " + regCount);
                            Log.i("GST", String.format("ERROR reg 0x%02X Len = 0x%02X (!= 1)",
                                    regNumber, regCount));
                        }
                        break;
                    case 0x94:
                        cs.requestFrom(0x93,1);                     // Keep polling
                        if (regCount == 8) {                        // Check register count
                            timeStamp   = cs.micros();              // Reading time
                            clear       = cs.readLH();              // Clear color
                            red         = cs.readLH();              // Red color
                            green       = cs.readLH();              // Green color
                            blue        = cs.readLH();              // Blue color
                            isNew       = true;
                        } else {
                            telemetry.addData("Error", regNumber + " length 8 != " + regCount);
                            Log.i("GST", String.format("ERROR reg 0x%02X Len = 0x%02X (!= 8)",
                                    regNumber, regCount));
                        }
                        break;
                    default:
                        telemetry.addData("Error", "Unexpected register " + regNumber);
                        break;
                }
            }
        }
        return isNew;
    }
}
