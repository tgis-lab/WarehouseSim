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
        
        // 检测拣货员进入交互区域
        private void OnTriggerEnter(Collider other)
        {
            // 检查是否是Picker
            PickerController picker = other.GetComponent<PickerController>();
            if (picker == null)
            {
                // 尝试在父对象上查找
                picker = other.GetComponentInParent<PickerController>();
                if (picker == null) return;
            }
            
            // 只有当机器人正在等待拣货且有当前订单时才处理
            if (robotController.IsPicking && robotController.CurrentOrder != null)
            {
                Debug.Log($"拣货员 {picker.Id} 进入机器人 {robotController.Id} 的交互区域");
                
                // 通知拣货员有拣货机会
                EventManager.Instance.TriggerRobotReadyForPicking(
                    robotController.Id.ToString(),
                    robotController.CurrentOrder.PickingPoint,
                    robotController.transform.position
                );
            }
        }
    }
}
