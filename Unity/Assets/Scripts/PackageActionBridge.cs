using System.Collections;
using DroneDelivery;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;

public class PackageActionBridge : MonoBehaviour
{
    private const string AutoInstallScene = "SampleScene";

    private ROSConnection ros;
    private PackageHandling packageHandling;

    private string lastAction = "";
    private bool processingAction;

    private void Start()
    {
        packageHandling =
            GetComponent<PackageHandling>();

        if (packageHandling == null)
        {
            Debug.LogError(
                "PackageActionBridge: PackageHandling " +
                "is missing from this drone."
            );

            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<StringMsg>(
            DroneRosTopics.PackageActionStatus
        );

        ros.Subscribe<StringMsg>(
            DroneRosTopics.PackageAction,
            OnPackageAction
        );

        Debug.Log(
            "PackageActionBridge ready with real package handling."
        );
    }

    private void OnPackageAction(StringMsg message)
    {
        string action =
            message.data.Trim().ToUpperInvariant();

        // M3 clears the latched command after confirmation.
        if (string.IsNullOrEmpty(action))
        {
            lastAction = "";
            return;
        }

        if (processingAction || action == lastAction)
        {
            return;
        }

        lastAction = action;

        Debug.Log(
            "ROS package action received: " +
            action
        );

        if (action == "PICKUP")
        {
            StartCoroutine(HandlePickup());
        }
        else if (action == "DROP")
        {
            HandleDrop();
        }
        else
        {
            Debug.LogWarning(
                "Unknown package action: " +
                action
            );
        }
    }

    private IEnumerator HandlePickup()
    {
        processingAction = true;

        Debug.Log(
            "Starting physical package pickup."
        );

        bool started =
            packageHandling.PickUpPackage();

        if (!started)
        {
            Debug.LogError(
                "Pickup could not start. " +
                "PackageHandling status: " +
                packageHandling.Status
            );

            processingAction = false;
            yield break;
        }

        yield return new WaitUntil(
            () => !packageHandling.IsBusy
        );

        if (!packageHandling.HasPackage)
        {
            Debug.LogError(
                "Pickup finished but no package " +
                "was attached. Status: " +
                packageHandling.Status
            );

            processingAction = false;
            yield break;
        }

        Debug.Log(
            "Physical pickup completed."
        );

        PublishStatus("PICKUP_DONE");
        processingAction = false;
    }

    private void HandleDrop()
    {
        processingAction = true;

        Debug.Log(
            "Starting physical package drop."
        );

        bool dropped =
            packageHandling.DropPackage();

        if (!dropped)
        {
            Debug.LogError(
                "Drop failed. " +
                "PackageHandling status: " +
                packageHandling.Status
            );

            processingAction = false;
            return;
        }

        if (packageHandling.HasPackage)
        {
            Debug.LogError(
                "Drop returned successfully but " +
                "the drone still has a package."
            );

            processingAction = false;
            return;
        }

        Debug.Log(
            "Physical drop completed."
        );

        PublishStatus("DROP_DONE");
        processingAction = false;
    }

    private void PublishStatus(string status)
    {
        ros.Publish(
            DroneRosTopics.PackageActionStatus,
            new StringMsg(status)
        );

        Debug.Log(
            "Unity package status sent: " +
            status
        );
    }

    [RuntimeInitializeOnLoadMethod(
        RuntimeInitializeLoadType.AfterSceneLoad
    )]
    private static void InstallForSampleScene()
    {
        if (
            SceneManager.GetActiveScene().name
            != AutoInstallScene
        )
        {
            return;
        }

        PackageHandling packageHandling =
            Object.FindFirstObjectByType<PackageHandling>();

        if (packageHandling == null)
        {
            Debug.LogError(
                "PackageActionBridge: delivery drone not found."
            );
            return;
        }

        GameObject drone =
            packageHandling.gameObject;

        if (drone.GetComponent<PackageActionBridge>() == null)
        {
            drone.AddComponent<PackageActionBridge>();
        }
    }
}
