using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Animations;

public class omniwheel_controller : MonoBehaviour
{

    public Transform Wheel_L1;
    public Transform Wheel_L2;
    public Transform Wheel_L3;
    public Transform Wheel_L4;
    public Transform Wheel_R1;
    public Transform Wheel_R2;
    public Transform Wheel_R3;
    public Transform Wheel_R4;

    public List<beacon_controller> beacons;
    public attachment_controller attachment;

    public bool controllable;
    public bool attached;

    public Transform attachedOmniWheel;

    Rigidbody m_Rigidbody;


    public float maxMotorTorque;
    public float maxSteeringTorque;
    public float maxRotateSpeed;


    void Start()
    {
        //Fetch the Rigidbody component you attach from your GameObject
        m_Rigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        if (controllable)
        {
            if (attached)
            {



                float motor = -maxMotorTorque * Input.GetAxis("Vertical");
                float steering = -maxSteeringTorque * Input.GetAxis("Horizontal");
                m_Rigidbody.velocity = transform.forward * motor + transform.right * steering;

                //forward/backward movement
                if (motor != 0 && steering == 0)
                {
                    Wheel_L1.Rotate(-motor * 2, 0, 0);
                    Wheel_L2.Rotate(-motor * 2, 0, 0);
                    Wheel_L3.Rotate(-motor * 2, 0, 0);
                    Wheel_L4.Rotate(-motor * 2, 0, 0);

                    Wheel_R1.Rotate(motor * 2, 0, 0);
                    Wheel_R2.Rotate(motor * 2, 0, 0);
                    Wheel_R3.Rotate(motor * 2, 0, 0);
                    Wheel_R4.Rotate(motor * 2, 0, 0);
                }

                //latreal movement 
                if (motor == 0 && steering != 0)
                {
                    Wheel_L1.Rotate(-steering * 2, 0, 0);
                    Wheel_L2.Rotate(-steering * 2, 0, 0);
                    Wheel_L3.Rotate(steering * 2, 0, 0);
                    Wheel_L4.Rotate(steering * 2, 0, 0);

                    Wheel_R1.Rotate(-steering * 2, 0, 0);
                    Wheel_R2.Rotate(-steering * 2, 0, 0);
                    Wheel_R3.Rotate(steering * 2, 0, 0);
                    Wheel_R4.Rotate(steering * 2, 0, 0);
                }

                //diagonal movement
                if (motor > 0 && steering > 0)
                {
                    Wheel_L3.Rotate(-steering * 2, 0, 0);
                    Wheel_L4.Rotate(-steering * 2, 0, 0);
                    Wheel_R1.Rotate(steering * 2, 0, 0);
                    Wheel_R2.Rotate(steering * 2, 0, 0);
                }

                if (motor > 0 && steering < 0)
                {
                    Wheel_L1.Rotate(steering * 2, 0, 0);
                    Wheel_L2.Rotate(steering * 2, 0, 0);
                    Wheel_R3.Rotate(-steering * 2, 0, 0);
                    Wheel_R4.Rotate(-steering * 2, 0, 0);
                }

                if (motor < 0 && steering < 0)
                {
                    Wheel_L1.Rotate(-steering * 2, 0, 0);
                    Wheel_L2.Rotate(-steering * 2, 0, 0);
                    Wheel_R3.Rotate(steering * 2, 0, 0);
                    Wheel_R4.Rotate(steering * 2, 0, 0);
                }

                if (motor < 0 && steering > 0)
                {
                    Wheel_L3.Rotate(steering * 2, 0, 0);
                    Wheel_L4.Rotate(steering * 2, 0, 0);
                    Wheel_R1.Rotate(-steering * 2, 0, 0);
                    Wheel_R1.Rotate(-steering * 2, 0, 0);
                }


                //rotate
                if (Input.GetKey(KeyCode.E))
                {
                   // transform.Rotate(0, -maxRotateSpeed * Time.deltaTime, 0);
                    Wheel_L1.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L2.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L3.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L4.Rotate(maxRotateSpeed * 0.25f, 0, 0);

                    Wheel_R1.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R2.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R3.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R4.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                }


                if (Input.GetKey(KeyCode.Q))
                {
                    //transform.Rotate(0, maxRotateSpeed * Time.deltaTime, 0);
                    Wheel_L1.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L2.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L3.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L4.Rotate(-maxRotateSpeed * 0.25f, 0, 0);

                    Wheel_R1.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R2.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R3.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R4.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                }

                if (Input.GetKeyDown(KeyCode.B))
                {
                    foreach (beacon_controller item in beacons)
                    {
                        item.on = !item.on;
                    }
                }

                if (Input.GetKeyDown(KeyCode.T))
                {
                    attachment.on = false;
                    controllable = false;
                    attached = false;

                    Animator attachAnimator = attachment.GetComponent<Animator>();
                    attachAnimator.SetBool("Off", false);


                }
            }
            else
            {
                if (attachedOmniWheel)
                {
                    attachedOmniWheel.transform.SetParent(gameObject.transform, true);
                }

                float motor = maxMotorTorque * Input.GetAxis("Vertical");
                float steering = maxSteeringTorque * Input.GetAxis("Horizontal");
                m_Rigidbody.velocity = transform.forward * motor + transform.right * steering;

                //forward/backward movement
                if (motor != 0 && steering == 0)
                {
                    Wheel_L1.Rotate(-motor * 2, 0, 0);
                    Wheel_L2.Rotate(-motor * 2, 0, 0);
                    Wheel_L3.Rotate(-motor * 2, 0, 0);
                    Wheel_L4.Rotate(-motor * 2, 0, 0);

                    Wheel_R1.Rotate(motor * 2, 0, 0);
                    Wheel_R2.Rotate(motor * 2, 0, 0);
                    Wheel_R3.Rotate(motor * 2, 0, 0);
                    Wheel_R4.Rotate(motor * 2, 0, 0);
                }

                //latreal movement 
                if (motor == 0 && steering != 0)
                {
                    Wheel_L1.Rotate(-steering * 2, 0, 0);
                    Wheel_L2.Rotate(-steering * 2, 0, 0);
                    Wheel_L3.Rotate(steering * 2, 0, 0);
                    Wheel_L4.Rotate(steering * 2, 0, 0);

                    Wheel_R1.Rotate(-steering * 2, 0, 0);
                    Wheel_R2.Rotate(-steering * 2, 0, 0);
                    Wheel_R3.Rotate(steering * 2, 0, 0);
                    Wheel_R4.Rotate(steering * 2, 0, 0);
                }

                //diagonal movement
                if (motor > 0 && steering > 0)
                {
                    Wheel_L1.Rotate(-steering * 2, 0, 0);
                    Wheel_L2.Rotate(-steering * 2, 0, 0);
                    Wheel_R3.Rotate(steering * 2, 0, 0);
                    Wheel_R4.Rotate(steering * 2, 0, 0);
                }

                if (motor > 0 && steering < 0)
                {
                    Wheel_L3.Rotate(steering * 2, 0, 0);
                    Wheel_L4.Rotate(steering * 2, 0, 0);
                    Wheel_R1.Rotate(-steering * 2, 0, 0);
                    Wheel_R2.Rotate(-steering * 2, 0, 0);
                }

                if (motor < 0 && steering < 0)
                {
                    Wheel_L3.Rotate(-steering * 2, 0, 0);
                    Wheel_L4.Rotate(-steering * 2, 0, 0);
                    Wheel_R1.Rotate(steering * 2, 0, 0);
                    Wheel_R2.Rotate(steering * 2, 0, 0);
                }

                if (motor < 0 && steering > 0)
                {
                    Wheel_L1.Rotate(steering * 2, 0, 0);
                    Wheel_L2.Rotate(steering * 2, 0, 0);
                    Wheel_R3.Rotate(-steering * 2, 0, 0);
                    Wheel_R4.Rotate(-steering * 2, 0, 0);
                }


                //rotate
                if (Input.GetKey(KeyCode.E))
                { 
                    transform.Rotate(0, -maxRotateSpeed * Time.deltaTime, 0);
                    Wheel_L1.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L2.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L3.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L4.Rotate(maxRotateSpeed * 0.25f, 0, 0);

                    Wheel_R1.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R2.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R3.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R4.Rotate(maxRotateSpeed * 0.25f, 0, 0);
                }


                if (Input.GetKey(KeyCode.Q))
                {
                    transform.Rotate(0, maxRotateSpeed * Time.deltaTime, 0);
                    Wheel_L1.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L2.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L3.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_L4.Rotate(-maxRotateSpeed * 0.25f, 0, 0);

                    Wheel_R1.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R2.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R3.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                    Wheel_R4.Rotate(-maxRotateSpeed * 0.25f, 0, 0);
                }

                if (Input.GetKeyDown(KeyCode.B))
                {
                    foreach (beacon_controller item in beacons)
                    {
                        item.on = !item.on;
                    }
                }

                if (Input.GetKeyDown(KeyCode.T))
                {
                    if (attachedOmniWheel)
                    {
                        omniwheel_controller attachedOmniController = attachedOmniWheel.GetComponent<omniwheel_controller>();

                        attachment.on = false;


                        attachedOmniController.attached = false;
                        attachedOmniController.controllable = false;



                        attachedOmniWheel.transform.Translate(0, 0, -0.1f);
                        Animator attachAnimator = attachment.GetComponent<Animator>();

                        attachAnimator.SetBool("Off", false);
                        attachedOmniWheel.transform.SetParent(null, true);


                        attachedOmniController.attached = false;
                        attachedOmniController.controllable = false;


                        attachedOmniWheel = null;
                    }

                    
                }
            }
           
        }
        
       


    }




}
