package org.team2168.thirdcoast.motion;

import java.io.*;

public class Trajectory {

  private double[] velocity;
  private double[] acceleration;
  private double[] heading;
  private double[] position;
  private double[] x;
  private double[] y;
  private int length;
  private BufferedReader bufferedReader;
  private File csvFile;

  public Trajectory(File csvFile) {
    this.csvFile = csvFile;
    length = 0;
    findLength();

    x = new double[length];
    y = new double[length];
    velocity = new double[length];
    acceleration = new double[length];
    heading = new double[length];
    position = new double[length];

    read();
  }

  private void findLength() {
    try {

      if (csvFile.exists()) {

        FileReader fileReader = new FileReader(csvFile);
        LineNumberReader lineNumberReader = new LineNumberReader(fileReader);

        length = 0;

        while (lineNumberReader.readLine() != null) {
          length++;
        }
        lineNumberReader.close();

      } else {
        System.out.println("File does not exist");
      }

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private void read() {
    try {
      bufferedReader = new BufferedReader(new FileReader(csvFile));
      int iteration = 0;
      String line;
      String headers = bufferedReader.readLine();
      System.out.println(headers);
      while ((line = bufferedReader.readLine()) != null) {
        String csvSplit = ",";
        String[] values = line.split(csvSplit);

        x[iteration] = Double.parseDouble(values[1]);
        y[iteration] = Double.parseDouble(values[2]);
        position[iteration] = Double.parseDouble(values[3]);
        velocity[iteration] = Double.parseDouble(values[4]);
        acceleration[iteration] = Double.parseDouble(values[5]);
        heading[iteration] = Double.parseDouble(values[7]);

        iteration++;
      }

    } catch (IOException e) {
      //TODO: check if this works
      System.err.println(e); // original line: > logger.error("{}", e);
    } finally {
      if (bufferedReader != null) {
        try {
          bufferedReader.close();
        } catch (IOException e) {
          //TODO: check if this works
          System.err.println(e);
        }
      }
    }
  }

  public int length() {
    return length;
  }

  public Segment getIteration(int iteration) {
    return new Segment(
        x[iteration],
        y[iteration],
        heading[iteration],
        acceleration[iteration],
        velocity[iteration],
        position[iteration]);
  }

  public static class Segment {

    public double heading;
    double acceleration;
    public double velocity;
    public double position;
    double x;
    double y;

    Segment(
        double x, double y, double heading, double acceleration, double velocity, double position) {
      this.heading = heading;
      this.acceleration = acceleration;
      this.velocity = velocity;
      this.position = position;
      this.x = x;
      this.y = y;
    }
  }
}