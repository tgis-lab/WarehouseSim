using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl.PickerControl
{
    public class ModelSynchronizer : MonoBehaviour
    {
        public GameObject pickerModel;
        private Transform pickerTransform;
        private Vector3 localPositionOffset;
        private Quaternion localRotationOffset;
        
        void Start()
        {
            if (pickerModel != null)
            {
                pickerTransform = pickerModel.transform;
                localPositionOffset = pickerTransform.localPosition;
                localRotationOffset = pickerTransform.localRotation;
            }
        }
        
        void LateUpdate()
        {
            if (pickerTransform != null)
            {
                pickerTransform.localPosition = localPositionOffset;
                pickerTransform.localRotation = localRotationOffset;
            }
        }
    }
}
