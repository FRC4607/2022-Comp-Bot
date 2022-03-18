package frc.robot;

import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;

public class Paths {

	public static class redPaths {
		public static Path Start_Ball2 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Start-Ball2-Red.wpilib.json");
		public static Path Start_Ball2B = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Start-Ball2B-Red.wpilib.json");
		public static Path Ball2_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball2-Hub-Red.wpilib.json");
		public static Path Ball2B_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball2B-HubB-Red.wpilib.json");
		public static Path Hub_Ball3 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Hub-Ball3-Red.wpilib.json");
		public static Path Ball3_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball3-Hub-Red.wpilib.json");
		public static Path Hub_Ball3_Ball4 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Hub-Ball3-Ball4-Red.wpilib.json");
		public static Path Ball4_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball4-Hub-Red.wpilib.json");
	}

	public static class bluePaths {
		public static Path Start_Ball2 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Start-Ball2-Blue.wpilib.json");
		public static Path Start_Ball2B = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Start-Ball2B-Blue.wpilib.json");
		public static Path Ball2_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball2-Hub-Blue.wpilib.json");
		public static Path Ball2B_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball2B-HubB-Blue.wpilib.json");
		public static Path Hub_Ball3 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Hub-Ball3-Blue.wpilib.json");
		public static Path Ball3_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball3-Hub-Blue.wpilib.json");
		public static Path Hub_Ball3_Ball4 = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Hub-Ball3-Ball4-Blue.wpilib.json");
		public static Path Ball4_Hub = Filesystem.getDeployDirectory().toPath()
				.resolve("./pathplanner/generatedJSON/Ball4-Hub-Blue.wpilib.json");
	}
}
