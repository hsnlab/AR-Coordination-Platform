using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneManager : MonoBehaviour
{
    public GameObject parent;
    public GameObject obj;
    public Camera cam;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnCreateBtn()
    {
        GameObject go= Instantiate(obj);
        go.transform.position = cam.transform.position + cam.transform.forward;

    }
}
