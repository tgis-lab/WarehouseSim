using System;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;
using CentralControl.RobotControl;
using PathPlanning;

namespace CentralControl
{
    /// <summary>
    /// Automated experiment runner that orchestrates batch experiment execution.
    /// Survives scene reloads via DontDestroyOnLoad with singleton dedup.
    /// Each trial gets a full scene reload for independent state.
    ///
    /// Lifecycle:
    ///   Awake() → singleton check → DontDestroyOnLoad
    ///   Start() → if autoStart, begin batch
    ///   SceneManager.sceneLoaded → set RobotManager.numberOfRobots, then run next trial
    /// </summary>
    public class ExperimentRunner : MonoBehaviour
    {
        // Singleton for scene-reload persistence
        private static ExperimentRunner _instance;

        [Header("Experiment Identity")]
        [Tooltip("Experiment configuration identifier (e.g., C1, C2, C3, C4, C5)")]
        public string configId = "C1";

        [Tooltip("Scenario identifier (robot density setting)")]
        public int scenarioId = 1;

        [Header("Algorithm Configuration")]
        public SearchAlgorithm algorithm = SearchAlgorithm.AnytimeSIPP;

        [Tooltip("Enable temporal planning (DynamicOccupancyLayer). Set false for C3 config.")]
        public bool temporalEnabled = true;

        [Header("Robot Parameters")]
        [Tooltip("Number of robots per floor. Controls RobotManager.numberOfRobots on scene load.")]
        public int robotsPerFloor = 3;

        [Tooltip("Number of floors in the warehouse (used to compute total robot count)")]
        public int floorCount = 3;

        public float defaultSpeed = 1.5f;
        public float maxAcceleration = 1.0f;
        public float maxDeceleration = 1.0f;

        [Header("Trial Configuration")]
        [Tooltip("Number of trials to run for this config × density")]
        public int trialCount = 5;

        [Tooltip("Duration of each trial in seconds")]
        public float trialDurationSeconds = 300f;

        [Header("Metrics Configuration")]
        [Tooltip("Fraction of robots that must be EmergencyStopped to declare breakdown")]
        [Range(0.5f, 1.0f)]
        public float breakdownThreshold = 0.8f;

        [Tooltip("Time series snapshot interval in seconds")]
        public float snapshotIntervalSeconds = 5f;

        [Header("Output")]
        [Tooltip("Output directory for experiment results. Empty = Application.persistentDataPath")]
        public string outputDirectory = "";

        [Header("Automation")]
        [Tooltip("Auto-start experiment on scene load")]
        public bool autoStart = false;

        [Tooltip("Auto-quit application after all trials complete")]
        public bool autoQuitOnComplete = false;

        // Runtime state (survives scene reloads via singleton)
        private int currentTrial = 0;
        private bool isRunning = false;
        private float trialStartTime;
        private ExperimentMetricsCollector metricsCollector;
        private string activeSceneName;

        /// <summary>
        /// Resolved output directory (defaults to persistentDataPath if empty).
        /// </summary>
        public string ResolvedOutputDirectory =>
            string.IsNullOrEmpty(outputDirectory)
                ? Path.Combine(Application.persistentDataPath, "ExperimentResults")
                : outputDirectory;

        void Awake()
        {
            // Singleton dedup: if a persisted instance already exists, destroy this scene copy
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }

            _instance = this;
            DontDestroyOnLoad(gameObject);

            // Register for scene-loaded callback (fires after every LoadScene)
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        void OnDestroy()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded;
            if (_instance == this) _instance = null;
        }

        void Start()
        {
            if (autoStart && !isRunning)
            {
                StartCoroutine(RunExperimentBatch());
            }
        }

        /// <summary>
        /// Called after every scene load (including reloads between trials).
        /// Sets RobotManager.numberOfRobots BEFORE CentralController.InitializeSystem runs.
        /// Then starts the next trial coroutine.
        /// </summary>
        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            if (!isRunning) return;

            // Set robot count on RobotManager before it initializes robots
            // RobotManager reads numberOfRobots in InitializeRobots() which is called
            // from CentralController.InitializeSystem() (a coroutine that yields first frame)
            var robotManager = FindObjectOfType<RobotManager>();
            if (robotManager != null)
            {
                int totalRobots = robotsPerFloor * floorCount;
                robotManager.numberOfRobots = totalRobots;
                Debug.Log($"[ExperimentRunner] Set RobotManager.numberOfRobots = {totalRobots} " +
                    $"({robotsPerFloor}/floor × {floorCount} floors)");
            }

            // Start the next trial after scene init completes
            StartCoroutine(RunCurrentTrial());
        }

        /// <summary>
        /// Starts the full experiment batch (all trials for the current config).
        /// </summary>
        public void StartExperiment()
        {
            if (isRunning)
            {
                Debug.LogWarning("[ExperimentRunner] Experiment already running.");
                return;
            }
            StartCoroutine(RunExperimentBatch());
        }

        private IEnumerator RunExperimentBatch()
        {
            isRunning = true;
            activeSceneName = SceneManager.GetActiveScene().name;
            currentTrial = 1;

            Debug.Log($"[ExperimentRunner] Starting batch: {configId} S{scenarioId} " +
                $"algo={algorithm} temporal={temporalEnabled} " +
                $"robots/floor={robotsPerFloor} trials={trialCount}");

            // Ensure output directory exists
            string outDir = ResolvedOutputDirectory;
            if (!Directory.Exists(outDir))
            {
                Directory.CreateDirectory(outDir);
            }

            // For the first trial, set robot count on the already-loaded scene's RobotManager
            var robotManager = FindObjectOfType<RobotManager>();
            if (robotManager != null)
            {
                robotManager.numberOfRobots = robotsPerFloor * floorCount;
            }

            // Run the first trial (scene is already loaded, no reload needed)
            yield return StartCoroutine(RunCurrentTrial());
        }

        /// <summary>
        /// Runs a single trial. Called after scene load (or on first start).
        /// </summary>
        private IEnumerator RunCurrentTrial()
        {
            Debug.Log($"[ExperimentRunner] === Trial {currentTrial}/{trialCount} ===");

            // Wait for CentralController to finish initialization
            yield return new WaitUntil(() =>
                CentralController.Instance != null &&
                CentralController.Instance.robotManager != null);

            // Wait a bit more for robots to be fully initialized
            yield return new WaitUntil(() =>
                CentralController.Instance.robotManager.GetAllRobots() != null &&
                CentralController.Instance.robotManager.GetAllRobots().Count > 0);
            yield return new WaitForSeconds(0.5f);

            // Configure robots for this experiment config
            ConfigureRobots();

            // Setup metrics collector for this trial
            SetupTrialMetrics();

            trialStartTime = Time.time;

            // Wait for trial duration or system breakdown
            float elapsed = 0f;
            while (elapsed < trialDurationSeconds)
            {
                yield return new WaitForSeconds(1f);
                elapsed = Time.time - trialStartTime;

                if (metricsCollector != null && metricsCollector.HasSystemBreakdown)
                {
                    Debug.Log($"[ExperimentRunner] System breakdown at t={elapsed:F1}s, ending trial early");
                    break;
                }
            }

            Debug.Log($"[ExperimentRunner] Trial {currentTrial} ended at t={Time.time - trialStartTime:F1}s");

            currentTrial++;

            if (currentTrial <= trialCount)
            {
                // Reload scene for a clean state — independent trial
                // MetricsCollector.OnDestroy() will save results during scene teardown
                Debug.Log($"[ExperimentRunner] Reloading scene for trial {currentTrial}...");
                SceneManager.LoadScene(activeSceneName);
                // OnSceneLoaded callback will fire → RunCurrentTrial starts
            }
            else
            {
                Debug.Log($"[ExperimentRunner] Batch complete: {configId} S{scenarioId}, {trialCount} trials");
                isRunning = false;

                if (autoQuitOnComplete)
                {
                    Debug.Log("[ExperimentRunner] Auto-quitting application.");
                    Application.Quit();
                }
            }
        }

        /// <summary>
        /// Sets up the metrics collector for the current trial.
        /// </summary>
        private void SetupTrialMetrics()
        {
            var cc = CentralController.Instance;
            if (cc != null)
            {
                metricsCollector = cc.GetComponent<ExperimentMetricsCollector>();
            }

            if (metricsCollector == null)
            {
                metricsCollector = cc != null
                    ? cc.gameObject.AddComponent<ExperimentMetricsCollector>()
                    : gameObject.AddComponent<ExperimentMetricsCollector>();
            }

            metricsCollector.experimentId = configId;
            metricsCollector.scenarioId = scenarioId;
            metricsCollector.trialId = currentTrial;
            metricsCollector.breakdownThreshold = breakdownThreshold;
            metricsCollector.outputDirectory = ResolvedOutputDirectory;
            metricsCollector.snapshotInterval = snapshotIntervalSeconds;

            var robots = CentralController.Instance?.robotManager?.GetAllRobots();
            if (robots != null)
            {
                metricsCollector.SetTotalRobots(robots.Count);
            }
        }

        /// <summary>
        /// Configures all robots in the scene to match experiment parameters.
        /// </summary>
        private void ConfigureRobots()
        {
            if (CentralController.Instance == null || CentralController.Instance.robotManager == null)
            {
                Debug.LogError("[ExperimentRunner] CentralController or RobotManager not ready");
                return;
            }

            var robots = CentralController.Instance.robotManager.GetAllRobots();
            if (robots == null) return;

            var cellSpaceGraph = CentralController.Instance.GetCellSpaceGraph();

            foreach (var robot in robots)
            {
                robot.selectedAlgorithm = algorithm;
                robot.temporalEnabled = temporalEnabled;
                robot.defaultSpeed = defaultSpeed;
                robot.maxAcceleration = maxAcceleration;
                robot.maxDeceleration = maxDeceleration;

                var planner = robot.GetPathPlanner();
                if (planner != null)
                {
                    planner.SetAlgorithm(algorithm);
                    planner.SetDynamics(maxAcceleration, maxDeceleration);

                    if (algorithm == SearchAlgorithm.CellSpaceAStar && cellSpaceGraph != null)
                    {
                        planner.SetCellSpaceGraph(cellSpaceGraph);
                    }
                }
            }

            Debug.Log($"[ExperimentRunner] Configured {robots.Count} robots: " +
                $"algo={algorithm}, temporal={temporalEnabled}, speed={defaultSpeed}");
        }

        /// <summary>
        /// Returns a summary string for logging.
        /// </summary>
        public string GetStatusSummary()
        {
            if (!isRunning) return "Idle";
            float elapsed = Time.time - trialStartTime;
            return $"[{configId} S{scenarioId}] Trial {currentTrial}/{trialCount} " +
                $"t={elapsed:F0}s/{trialDurationSeconds:F0}s";
        }
    }
}
