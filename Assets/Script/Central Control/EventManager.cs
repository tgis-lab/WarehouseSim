using System;
using UnityEngine;

namespace CentralControl
{
    public class EventManager : MonoBehaviour
    {
        public static EventManager Instance { get; private set; }

        public event Action OnMapLoaded;
        public event Action OnRobotsInitialized;

        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
                DontDestroyOnLoad(gameObject);
            }
            else
            {
                Destroy(gameObject);
            }
        }

        public void TriggerMapLoaded()
        {
            OnMapLoaded?.Invoke();
        }

        public void TriggerRobotsInitialized()
        {
            OnRobotsInitialized?.Invoke();
        }
    }
}