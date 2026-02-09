using System;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;
using CentralControl;
using CentralControl.RobotControl;

namespace CameraSystem
{
    /// <summary>
    /// Controls a Cinemachine VirtualCamera that follows robots in the warehouse.
    /// Attach to an empty GameObject in the scene. Requires CinemachineBrain on Main Camera.
    /// </summary>
    public class RobotCameraController : MonoBehaviour
    {
        [Header("Follow Settings")]
        [SerializeField] private Vector3 followOffset = new Vector3(0f, 1.0f, -0.5f);
        [SerializeField] private Vector3 damping = new Vector3(0.5f, 0.3f, 0.5f);

        [Header("Aim Settings")]
        [SerializeField] private Vector3 trackedObjectOffset = new Vector3(0f, 0f, 1.5f);

        [Header("Lens Settings")]
        [SerializeField] private float fieldOfView = 65f;
        [SerializeField] private float nearClip = 0.1f;
        [SerializeField] private float farClip = 100f;

        public int CurrentRobotId { get; private set; } = -1;
        public bool IsOnboardActive { get; private set; }
        public RobotBehavior CurrentRobotBehavior => currentRobot != null ? currentRobot.Behavior : RobotBehavior.Idle;
        public string CurrentFloor => currentRobot != null ? currentRobot.CurrentFloor : "";

        public event Action<int> OnCameraTargetChanged;

        private CinemachineVirtualCamera virtualCamera;
        private RobotManager robotManager;
        private RobotController currentRobot;
        private List<RobotController> robotList = new List<RobotController>();
        private int currentIndex;

        private const int PRIORITY_ACTIVE = 10;
        private const int PRIORITY_INACTIVE = 0;

        private void Start()
        {
            EnsureCinemachineBrain();
            CreateVirtualCamera();
        }

        private void Update()
        {
            if (!EnsureRobotManager()) return;

            HandleInput();
            ValidateCurrentRobot();
        }

        private void EnsureCinemachineBrain()
        {
            var mainCamera = Camera.main;
            if (mainCamera == null)
            {
                Debug.LogWarning("[RobotCamera] No Main Camera found.");
                return;
            }

            if (mainCamera.GetComponent<CinemachineBrain>() == null)
            {
                mainCamera.gameObject.AddComponent<CinemachineBrain>();
            }
        }

        private void CreateVirtualCamera()
        {
            var vcamObj = new GameObject("RobotFollowCam");
            vcamObj.transform.SetParent(transform);

            virtualCamera = vcamObj.AddComponent<CinemachineVirtualCamera>();
            virtualCamera.m_Lens.FieldOfView = fieldOfView;
            virtualCamera.m_Lens.NearClipPlane = nearClip;
            virtualCamera.m_Lens.FarClipPlane = farClip;

            // Body: Transposer for positional follow
            var transposer = virtualCamera.AddCinemachineComponent<CinemachineTransposer>();
            transposer.m_BindingMode = CinemachineTransposer.BindingMode.LockToTargetWithWorldUp;
            transposer.m_FollowOffset = followOffset;
            transposer.m_XDamping = damping.x;
            transposer.m_YDamping = damping.y;
            transposer.m_ZDamping = damping.z;

            // Aim: Composer for look-at with offset
            var composer = virtualCamera.AddCinemachineComponent<CinemachineComposer>();
            composer.m_TrackedObjectOffset = trackedObjectOffset;

            // Start inactive
            virtualCamera.Priority = PRIORITY_INACTIVE;
        }

        private bool EnsureRobotManager()
        {
            if (robotManager != null) return true;

            if (CentralController.Instance == null) return false;
            robotManager = CentralController.Instance.robotManager;
            return robotManager != null;
        }

        private void RefreshRobotList()
        {
            robotList = robotManager.GetAllRobots();
        }

        private void HandleInput()
        {
            if (Input.GetKeyDown(KeyCode.C))
                ToggleOnboard();

            if (Input.GetKeyDown(KeyCode.N))
                SwitchToNextRobot();

            if (Input.GetKeyDown(KeyCode.P))
                SwitchToPreviousRobot();

            // Number keys 1-9 for direct selection
            for (int i = 0; i < 9; i++)
            {
                if (Input.GetKeyDown(KeyCode.Alpha1 + i))
                {
                    AttachToRobot(i + 1);
                    if (!IsOnboardActive) ToggleOnboard();
                    break;
                }
            }
        }

        private void ValidateCurrentRobot()
        {
            if (!IsOnboardActive) return;
            if (currentRobot != null) return;

            // Current robot was destroyed — switch to next available
            RefreshRobotList();
            if (robotList.Count > 0)
            {
                currentIndex = Mathf.Clamp(currentIndex, 0, robotList.Count - 1);
                SetTarget(robotList[currentIndex]);
            }
            else
            {
                ToggleOnboard();
            }
        }

        public void ToggleOnboard()
        {
            IsOnboardActive = !IsOnboardActive;

            if (IsOnboardActive)
            {
                virtualCamera.Priority = PRIORITY_ACTIVE;

                if (currentRobot == null)
                {
                    RefreshRobotList();
                    if (robotList.Count > 0)
                    {
                        currentIndex = 0;
                        SetTarget(robotList[0]);
                    }
                    else
                    {
                        IsOnboardActive = false;
                        virtualCamera.Priority = PRIORITY_INACTIVE;
                        Debug.LogWarning("[RobotCamera] No robots available.");
                    }
                }
            }
            else
            {
                virtualCamera.Priority = PRIORITY_INACTIVE;
            }
        }

        public void AttachToRobot(int robotId)
        {
            if (!EnsureRobotManager()) return;

            var robot = robotManager.GetRobotById(robotId);
            if (robot == null)
            {
                Debug.LogWarning($"[RobotCamera] Robot {robotId} not found.");
                return;
            }

            RefreshRobotList();
            currentIndex = robotList.IndexOf(robot);
            SetTarget(robot);
        }

        public void SwitchToNextRobot()
        {
            RefreshRobotList();
            if (robotList.Count == 0) return;

            currentIndex = (currentIndex + 1) % robotList.Count;
            SetTarget(robotList[currentIndex]);

            if (!IsOnboardActive) ToggleOnboard();
        }

        public void SwitchToPreviousRobot()
        {
            RefreshRobotList();
            if (robotList.Count == 0) return;

            currentIndex = (currentIndex - 1 + robotList.Count) % robotList.Count;
            SetTarget(robotList[currentIndex]);

            if (!IsOnboardActive) ToggleOnboard();
        }

        private void SetTarget(RobotController robot)
        {
            currentRobot = robot;
            CurrentRobotId = robot.Id;

            virtualCamera.Follow = robot.transform;
            virtualCamera.LookAt = robot.transform;

            OnCameraTargetChanged?.Invoke(CurrentRobotId);
        }
    }
}
