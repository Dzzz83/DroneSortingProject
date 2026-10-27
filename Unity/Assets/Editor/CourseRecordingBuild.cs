using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>Builds only the existing ROS course, without the Editor's focus throttling.</summary>
[InitializeOnLoad]
public static class CourseRecordingBuild
{
    public const string PlayerPath = "Builds/CourseRecording/CourseDemo.x86_64";
    const string RequestPath = "Temp/course-recording-build.request";
    const string ResultPath = "Logs/course-recording-build.result";
    static double nextCheck;

    static CourseRecordingBuild() => EditorApplication.update += CheckBuildRequest;

    static void CheckBuildRequest()
    {
        if (EditorApplication.timeSinceStartup < nextCheck) return;
        nextCheck = EditorApplication.timeSinceStartup + 1;
        if (EditorApplication.isPlayingOrWillChangePlaymode || EditorApplication.isCompiling ||
            EditorApplication.isUpdating || !File.Exists(RequestPath)) return;
        File.Delete(RequestPath);
        try { Build(); }
        catch (Exception exception) { Debug.LogException(exception); }
    }

    [MenuItem("Drone Delivery/Build Recording Player")]
    public static void Build()
    {
        Directory.CreateDirectory("Logs");
        if (File.Exists(ResultPath)) File.Delete(ResultPath);
        try
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Stop Play before building the recording player.");
            // Never silently discard an unsaved scene edit when building the saved course.
            for (int i = 0; i < UnityEngine.SceneManagement.SceneManager.sceneCount; i++)
                if (UnityEngine.SceneManagement.SceneManager.GetSceneAt(i).isDirty)
                    throw new InvalidOperationException("Save intentional scene edits before building the recording player.");
            TwoWarehouseCourseVerification.VerifyNavigationSmoothness();
            Directory.CreateDirectory(Path.GetDirectoryName(PlayerPath));
            BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { TwoWarehouseCourseBuilder.ScenePath },
                locationPathName = PlayerPath,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None
            });
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException("Recording player build failed: " + report.summary.result);
            File.WriteAllText(ResultPath,"OK\n" + Path.GetFullPath(PlayerPath));
            Debug.Log("COURSE_RECORDING_BUILD_PASS: " + Path.GetFullPath(PlayerPath));
        }
        catch (Exception exception)
        {
            File.WriteAllText(ResultPath,"ERROR\n" + exception.Message);
            throw;
        }
    }
}
