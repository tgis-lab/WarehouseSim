using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class beacon_controller : MonoBehaviour
{
    public bool on;
    public Transform rotor;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (on)
        {
            rotor.Rotate(0, 0, 200f * Time.deltaTime);
        }
    }
}
