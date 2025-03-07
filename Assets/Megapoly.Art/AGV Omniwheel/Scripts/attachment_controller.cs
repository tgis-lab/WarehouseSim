using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class attachment_controller : MonoBehaviour
{

    public bool on;
    public omniwheel_controller attachedBody;


    // Start is called before the first frame update
    void Start()
    {
       
       
    }

    private void Update()
    {
        if (on)
        {
            attachedBody = transform.parent.GetComponent<omniwheel_controller>();
            if (!attachedBody.controllable)
            {
                attachedBody.controllable = true;
                attachedBody.attached = true;
            }
           

        }     

    }


}
