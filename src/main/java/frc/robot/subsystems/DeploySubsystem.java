package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.*;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class DeploySubsystem extends SubsystemBase {
  private Deploy deploy = new Deploy();

  public DeploySubsystem() {
    this.addChild("Data", deploy);
  }

  public void Log() {
    System.out.println("--** BUILD INFORMATION START **--");
    System.out.printf("Machine: %s\n", deploy.BuildMachine());
    System.out.printf("Branch: %s\n", deploy.BuildBranch());
    System.out.printf("Commit: %s\n", deploy.BuildCommit());
    System.out.printf("Date: %s\n", deploy.BuildDate());
    System.out.println("--** BUILD INFORMATION END **--");
  }

  public class Deploy implements Sendable {
    private final String buildBranch;
    private final String buildCommit;
    private final String buildMachine;
    private final String buildDate;

    public String BuildBranch() {
      return buildBranch;
    }

    public String BuildCommit() {
      return buildCommit;
    }

    public String BuildMachine() {
      return buildMachine;
    }

    public String BuildDate() {
      return buildDate;
    }

    public Deploy() {
      File deployDir = Filesystem.getDeployDirectory();

      File branchFile = new File(deployDir, "branch.txt");
      buildBranch = readFile(branchFile);

      File commitFile = new File(deployDir, "commit.txt");
      buildCommit = readFile(commitFile);

      File hostnameFile = new File(deployDir, "hostname.txt");
      buildMachine = readFile(hostnameFile);

      File dateFile = new File(deployDir, "date.txt");
      buildDate = readFile(dateFile);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("DeployData");
      builder.addStringProperty("machine", this::BuildBranch, null);
      builder.addStringProperty("branch", this::BuildBranch, null);
      builder.addStringProperty("commit", this::BuildCommit, null);
      builder.addStringProperty("date", this::BuildDate, null);
    }

    private String readFile(File name) {
      String str = "";

      try {
        str = new String(Files.readAllBytes(Paths.get(name.getCanonicalPath())));
      } catch (IOException e) {
        return "MISSING FILE";
      }

      return str;
    }
  }
}
