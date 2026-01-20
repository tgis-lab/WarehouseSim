using System;
using System.Diagnostics;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace CentralControl
{
    public enum LogLevel
    {
        None = 0,
        Error = 1,
        Warning = 2,
        Info = 3,
        Verbose = 4
    }

    public enum LogCategory
    {
        General,
        Robot,
        PathPlanning,
        Collision,
        Order,
        Event,
        Deadlock,
        Performance
    }

    public static class DebugLogger
    {
        private static LogLevel currentLogLevel = LogLevel.Info;
        private static bool[] categoryEnabled = new bool[Enum.GetValues(typeof(LogCategory)).Length];

        static DebugLogger()
        {
            for (int i = 0; i < categoryEnabled.Length; i++)
            {
                categoryEnabled[i] = true;
            }
        }

        public static LogLevel CurrentLogLevel
        {
            get => currentLogLevel;
            set => currentLogLevel = value;
        }

        public static void SetCategoryEnabled(LogCategory category, bool enabled)
        {
            categoryEnabled[(int)category] = enabled;
        }

        public static bool IsCategoryEnabled(LogCategory category)
        {
            return categoryEnabled[(int)category];
        }

        public static void EnableAllCategories()
        {
            for (int i = 0; i < categoryEnabled.Length; i++)
            {
                categoryEnabled[i] = true;
            }
        }

        public static void DisableAllCategories()
        {
            for (int i = 0; i < categoryEnabled.Length; i++)
            {
                categoryEnabled[i] = false;
            }
        }

        public static void SetProductionMode()
        {
            currentLogLevel = LogLevel.Warning;
            DisableAllCategories();
            SetCategoryEnabled(LogCategory.General, true);
        }

        public static void SetDebugMode()
        {
            currentLogLevel = LogLevel.Verbose;
            EnableAllCategories();
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void Log(string message, LogCategory category = LogCategory.General)
        {
            if (currentLogLevel >= LogLevel.Info && categoryEnabled[(int)category])
            {
                Debug.Log($"[{category}] {message}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogVerbose(string message, LogCategory category = LogCategory.General)
        {
            if (currentLogLevel >= LogLevel.Verbose && categoryEnabled[(int)category])
            {
                Debug.Log($"[{category}:V] {message}");
            }
        }

        public static void LogWarning(string message, LogCategory category = LogCategory.General)
        {
            if (currentLogLevel >= LogLevel.Warning && categoryEnabled[(int)category])
            {
                Debug.LogWarning($"[{category}] {message}");
            }
        }

        public static void LogError(string message, LogCategory category = LogCategory.General)
        {
            if (currentLogLevel >= LogLevel.Error)
            {
                Debug.LogError($"[{category}] {message}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogRobot(int robotId, string message)
        {
            if (currentLogLevel >= LogLevel.Info && categoryEnabled[(int)LogCategory.Robot])
            {
                Debug.Log($"[Robot {robotId}] {message}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogCollision(string message)
        {
            if (currentLogLevel >= LogLevel.Info && categoryEnabled[(int)LogCategory.Collision])
            {
                Debug.LogWarning($"[Collision] {message}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogEvent(string eventName, string details = null)
        {
            if (currentLogLevel >= LogLevel.Verbose && categoryEnabled[(int)LogCategory.Event])
            {
                string msg = string.IsNullOrEmpty(details) ? eventName : $"{eventName}: {details}";
                Debug.Log($"[Event] {msg}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogDeadlock(string message)
        {
            if (currentLogLevel >= LogLevel.Warning && categoryEnabled[(int)LogCategory.Deadlock])
            {
                Debug.LogWarning($"[Deadlock] {message}");
            }
        }

        [Conditional("UNITY_EDITOR"), Conditional("DEVELOPMENT_BUILD")]
        public static void LogPerformance(string message)
        {
            if (currentLogLevel >= LogLevel.Verbose && categoryEnabled[(int)LogCategory.Performance])
            {
                Debug.Log($"[Performance] {message}");
            }
        }
    }
}
