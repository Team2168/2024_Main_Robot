package org.team2168;

import org.team2168.utils.F310;
import org.team2168.Constants.Joysticks;

public class OI {

    public final F310 driverJoystick = new F310(Joysticks.DRIVER_JOYSTICK);
    public final F310 operatorJoystick = new F310(Joysticks.OPERATOR_JOYSTICK);
    public final F310 testJoystick = new F310(Joysticks.PID_TEST_JOYSTICK);

    private static OI instance = null;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
}
