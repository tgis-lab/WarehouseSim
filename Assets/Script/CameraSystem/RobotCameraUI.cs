using UnityEngine;
using CentralControl.RobotControl;
using System.Collections.Generic;

namespace CameraSystem
{
    /// <summary>
    /// IMGUI overlay for the robot camera system.
    /// Displays robot info and provides controls for switching targets and view modes.
    /// Attach to the same GameObject as RobotCameraController.
    /// </summary>
    public class RobotCameraUI : MonoBehaviour
    {
        // --- Customize these to adjust the panel appearance ---
        [Header("Panel Settings")]
        [SerializeField] private Rect panelRect = new Rect(10, 10, 220, 200);
        [SerializeField] private Color backgroundColor = new Color(0f, 0f, 0f, 0.7f);
        [SerializeField] private Color textColor = new Color(0.9f, 0.9f, 0.9f, 1f);
        [SerializeField] private int fontSize = 13;

        private RobotCameraController cameraController;
        private GUIStyle boxStyle;
        private GUIStyle labelStyle;
        private GUIStyle headerStyle;
        private GUIStyle buttonStyle;
        private bool stylesInitialized;
        private int selectedRobotIndex;
        private bool showRobotDropdown;

        private void Start()
        {
            cameraController = GetComponent<RobotCameraController>();
            if (cameraController == null)
            {
                Debug.LogWarning("[RobotCameraUI] RobotCameraController not found on this GameObject.");
                enabled = false;
            }
        }

        private void InitStyles()
        {
            if (stylesInitialized) return;

            var bgTex = new Texture2D(1, 1);
            bgTex.SetPixel(0, 0, backgroundColor);
            bgTex.Apply();

            boxStyle = new GUIStyle(GUI.skin.box)
            {
                normal = { background = bgTex, textColor = textColor },
                fontSize = fontSize,
                padding = new RectOffset(8, 8, 8, 8)
            };

            labelStyle = new GUIStyle(GUI.skin.label)
            {
                normal = { textColor = textColor },
                fontSize = fontSize,
                wordWrap = true
            };

            headerStyle = new GUIStyle(GUI.skin.label)
            {
                normal = { textColor = Color.white },
                fontSize = fontSize + 2,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };

            buttonStyle = new GUIStyle(GUI.skin.button) { fontSize = fontSize };

            stylesInitialized = true;
        }

        private void OnGUI()
        {
            if (cameraController == null) return;

            InitStyles();

            if (!cameraController.IsOnboardActive)
            {
                DrawMinimalHint();
                return;
            }

            DrawOnboardPanel();
        }

        private void DrawMinimalHint()
        {
            GUI.Label(new Rect(10, 10, 300, 24), "Press [C] for onboard view", labelStyle);
        }

        private void DrawOnboardPanel()
        {
            GUI.Box(panelRect, "", boxStyle);

            GUILayout.BeginArea(new Rect(panelRect.x + 8, panelRect.y + 8,
                panelRect.width - 16, panelRect.height - 16));

            // Header
            GUILayout.Label($"Onboard: Robot {cameraController.CurrentRobotId}", headerStyle);
            GUILayout.Space(4);

            // Robot info
            GUILayout.Label($"State: {cameraController.CurrentRobotBehavior}", labelStyle);
            GUILayout.Label($"Floor: {cameraController.CurrentFloor}", labelStyle);
            GUILayout.Space(8);

            // Navigation buttons
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Prev [P]", buttonStyle))
                cameraController.SwitchToPreviousRobot();
            if (GUILayout.Button("Next [N]", buttonStyle))
                cameraController.SwitchToNextRobot();
            GUILayout.EndHorizontal();

            GUILayout.Space(4);

            if (GUILayout.Button("Overview [C]", buttonStyle))
                cameraController.ToggleOnboard();

            GUILayout.Space(4);
            GUILayout.Label("Keys: 1-9 = select robot", labelStyle);

            GUILayout.EndArea();
        }
    }
}
