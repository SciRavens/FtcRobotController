package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo servo;
    private boolean closed = false;
    double pos_close, pos_open;
    public Claw(Servo servo, double close, double open) {
        this.servo = servo;
        this.pos_close = close;
        this.pos_open = open;
        servo.setPosition(pos_open);
    }

    public void open()
    {
        if(closed) {
            servo.setPosition(pos_open);
            closed = false;
        }
    }

    public void close()
    {
        if (!closed) {
            servo.setPosition(pos_close);
            closed = true;
        }
    }
}
