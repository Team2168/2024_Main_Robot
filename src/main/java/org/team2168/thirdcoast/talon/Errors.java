package org.team2168.thirdcoast.talon;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** Utility class to check for and display TalonSRX configuration errors. */
public class Errors {

  private static boolean summarized = true;
  private static int count;

  public static void check(ErrorCode error) {
    if (error != null && error != ErrorCode.OK) {
      if (summarized) count++;
      else System.out.printf("error while configuring Talon: {}", error);
    }
  }

  public static void check(TalonSRX talon, String method, ErrorCode error) {
    if (error != null && error != ErrorCode.OK) {
      if (summarized) count++;
      else System.out.printf("Talon {}: {} error {}", talon.getDeviceID(), method, error);
    }
  }

  public static boolean isSummarized() {
    return summarized;
  }

  public static void setSummarized(boolean summarized) {
    Errors.summarized = summarized;
  }

  public static int getCount() {
    return count;
  }

  public static void setCount(int count) {
    Errors.count = count;
  }
}
