using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class pinlock : MonoBehaviour
{

    public attachment_controller attachment;
    private Animator animator;


    void Start()
    {
        animator = attachment.GetComponent<Animator>();
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.gameObject.name == "PinTarget")
        {
            animator.SetBool("Off", true);
            attachment.on = true;

            attachment_controller target = other.transform.parent.GetComponent<attachment_controller>();
            target.on = true;
            Animator targetAnimator = target.GetComponent<Animator>();
            targetAnimator.SetBool("Off", true);

            omniwheel_controller parentOmni = attachment.transform.parent.GetComponent<omniwheel_controller>();
            parentOmni.attachedOmniWheel = other.transform.parent.transform.parent;

           
        }
    }
}
