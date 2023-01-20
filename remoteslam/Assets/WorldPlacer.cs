using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WorldPlacer : MonoBehaviour
{
    public Transform cameraTransform;
    public Transform rotator;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        rotator.rotation = Quaternion.Inverse(cameraTransform.rotation);
        transform.position = new Vector3(-cameraTransform.position.x, -cameraTransform.position.y, -cameraTransform.position.z);
    }
}
