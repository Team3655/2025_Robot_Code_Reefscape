package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Filesystem;

public class PathPlannerUtil {

  @SuppressWarnings("unchecked")
  public static void writeSettings(RobotConfig robot, ModuleConfig module, double driveGearing)
      throws IOException, ParseException {

    File file = new File(Filesystem.getDeployDirectory(), "pathplanner/settings.json");
    BufferedReader br = new BufferedReader(new FileReader(file));

    StringBuilder fileContentBuilder = new StringBuilder();
    String line;
    while ((line = br.readLine()) != null) {
      fileContentBuilder.append(line);
    }
    br.close();

    String fileContent = fileContentBuilder.toString();
    JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    json.put("holonomicMode", robot.isHolonomic);
    json.put("robotMass", robot.massKG);
    json.put("robotMOI", robot.MOI);
    json.put("driveWheelRadius", module.wheelRadiusMeters);
    json.put("driveGearing", driveGearing);
    json.put("maxDriveSpeed", module.maxDriveVelocityMPS);
    json.put("wheelCOF", module.wheelCOF);
    json.put("driveCurrentLimit", module.driveCurrentLimit);

    if (robot.isHolonomic) {
      json.put("flModuleX", robot.moduleLocations[0].getX());
      json.put("flModuleY", robot.moduleLocations[0].getY());

      json.put("frModuleX", robot.moduleLocations[1].getX());
      json.put("frModuleY", robot.moduleLocations[1].getY());

      json.put("blModuleX", robot.moduleLocations[2].getX());
      json.put("blModuleY", robot.moduleLocations[2].getY());

      json.put("brModuleX", robot.moduleLocations[3].getX());
      json.put("brModuleY", robot.moduleLocations[3].getY());
    }

    FileWriter writer = new FileWriter(file);
    json.writeJSONString(writer);
    writer.flush();
    writer.close();
  }
}