using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if PLATFORM_ANDROID
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Contexts;
using UnityEngine.Android;

#endif

public class ImuStreamerService : MonoBehaviour
{

    const string pluginName = "nbgy.hsn.imustreamerlib.ImuStreamer";

    static AndroidJavaClass _pluginClass;
    static AndroidJavaObject _pluginInstance;
    public static string rosIpAddress;
    public string portNumber="9090";
#if PLATFORM_ANDROID
    public static AndroidJavaClass PluginClass
    {
        get
        {
            if (_pluginClass == null)
            {
                _pluginClass = new AndroidJavaClass(pluginName);
            }
            return _pluginClass;
        }
    }

    public static AndroidJavaObject PluginInstance
    {
        get
        {
            if (_pluginInstance == null)
            {
                _pluginInstance = PluginClass.CallStatic<AndroidJavaObject>("getInstance");
            }
            return _pluginInstance;
        }
    }

        GameObject dialog = null;
    // Start is called before the first frame update
    void Awake()
    {

        AndroidJavaClass jc = new AndroidJavaClass("com.unity3d.player.UnityPlayer");
        AndroidJavaObject jo = jc.GetStatic<AndroidJavaObject>("currentActivity");
        PluginInstance.Call("startImuStreaming", jo, rosIpAddress+":"+portNumber);

}
#endif

    // Update is called once per frame
    void Update()
    {
        
    }
}
