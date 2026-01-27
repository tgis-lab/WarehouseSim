using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CentralControl;
using CentralControl.PickerControl;

namespace CentralControl.RobotControl
{
    public class RobotInteractionZone : MonoBehaviour
    {
        public RobotController robotController;
        
        private void OnTriggerEnter(Collider other)
        {
            PickerController picker = other.GetComponent<PickerController>();
            if (picker == null)
            {
                picker = other.GetComponentInParent<PickerController>();
                if (picker == null) return;
            }
            
            if (robotController.IsPicking && robotController.CurrentOrder != null)
            {
                EventManager.Instance.TriggerRobotReadyForPicking(
                    robotController.Id.ToString(),
                    robotController.CurrentOrder.PickingPoint,
                    robotController.transform.position
                );
            }
        }
    }
}
