using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
#if PLATFORM_ANDROID
using UnityEngine.Android;
using UnityEngine.UI;
#endif
public class NativeCameraHandler : MonoBehaviour
{
    [DllImport("native-lib")]
    private static extern void logString(String msg);

    [DllImport("native-lib")]
    private static extern void AddApplicationDataPath(String path);

    [DllImport("native-lib")]
    private static extern void InitStream(String address, UInt16 port, Int32 width, Int32 height);

    [DllImport("native-lib")]
    private static extern void SetTextureFromUnity(System.IntPtr texture);

    [DllImport("native-lib")]
    private static extern void SetTexturesFromUnity(System.IntPtr textureY, System.IntPtr textureU, System.IntPtr textureV);

    [DllImport("native-lib")]
    private static extern IntPtr GetRenderEventFunc();

    [DllImport("native-lib")]
    private static extern void InitCamera();

    [DllImport("native-lib")]
    private static extern void DeinitCamera();

    [DllImport("native-lib")]
    private static extern void SetTimestampFromUnity(Int64 timestamp);

    [DllImport("native-lib")]
    private static extern Int64 GetPhotoTimestamp(Int64 timestamp);

    [DllImport("native-lib")]
    private static extern long getCurrentTime();

    public Material displayMaterial;
    public Material YUV_material;

    private AndroidJavaObject _androidJavaPlugin = null;

    public RawImage rawImage;
    //public RawImage y_rawImage;
    //public RawImage u_rawImage2;
    //public RawImage v_rawImage3;

    Texture2D tex;

    Texture2D m_YImageTex;
    Texture2D m_UImageTex;
    Texture2D m_VImageTex;


    Shader YUVTex;

    public int width= 1280;
    public int height =720;
    public static string ipAddress = "18.195.64.236";
    public string portNumber = "9090";

    GameObject dialog = null;

    public static long currentPositionTimestamp;

    static List< MeasureData> measureList;
    public static bool measure_is_running = true;

    StreamWriter sw1;
    struct MeasureData
    {
        public MeasureData(long start, long stop, Vector3 pos, Quaternion rot)
        {
            startTime = start;
            stopTime = stop;
            position = pos;
            rotation = rot;
        }
        public long startTime { get; set; }
        public long stopTime { get; set; }
        public Vector3 position { get; set; }
        public Quaternion rotation { get; set; }
    }

    string filePath;

    // Start is called before the first frame update
    IEnumerator Start()
    {
        Application.targetFrameRate = 33;
        measureList = new List<MeasureData>();
        //string id = DateTime.Now.ToLongTimeString();
        filePath = Application.persistentDataPath;
        AddApplicationDataPath(filePath);
        //sw1 = File.CreateText(filePath);
        //sw1.WriteLine("Start,Stop,Position,Rotation");
        Debug.Log("Create java class");
        using (AndroidJavaClass javaClass = new AndroidJavaClass("hu.bme.nbgy.nativecamerahandler.NativeCameraHandlerActivity"))
        {
            Debug.Log(javaClass.ToString());
            _androidJavaPlugin = javaClass.GetStatic<AndroidJavaObject>("_context");

            InitStream(ipAddress, Convert.ToUInt16(portNumber), width, height);
            CreateTextureAndPassToPlugin();
            InitCamera();
            
            if (Application.platform == RuntimePlatform.Android)
            {
                yield return StartCoroutine("CallPluginAtEndOfFrames");
            }
            
        }

        //getCameraPermission();
    }
    // Update is called once per frame
    void Update()
    {

    }

    public static void  LogString(string msg)
    {
        logString(msg);
    }
    public static long GetTime()
    {
        return getCurrentTime();
    }

    public static void SetPositionTimestamp(long timestamp, Vector3 pos, Quaternion rot)
    {
        currentPositionTimestamp = timestamp;
        long time = getCurrentTime();
        Int64 createTime = GetPhotoTimestamp(Convert.ToInt64( timestamp));
        //Debug.Log("Sent: " + createTime+ " Arrived:" + time );
        measureList.Add(new MeasureData(createTime, time, pos, rot));
    }
    private void CreateTextureAndPassToPlugin()
    {
        m_YImageTex = new Texture2D(width, height, TextureFormat.ARGB32, false);
        m_YImageTex.Apply();
        m_UImageTex = new Texture2D(width / 2, height / 2, TextureFormat.Alpha8, false);
        m_UImageTex.Apply();
        m_VImageTex = new Texture2D(width / 2, height / 2, TextureFormat.Alpha8, false);
        m_VImageTex.Apply();

        YUV_material.mainTexture = m_YImageTex;
        //YUV_material.SetTexture("_MainTex", m_YImageTex);
        //YUV_material.SetTexture("_UTex", m_UImageTex);
        //YUV_material.SetTexture("_VTex", m_VImageTex);

        SetTexturesFromUnity(m_YImageTex.GetNativeTexturePtr(), m_UImageTex.GetNativeTexturePtr(), m_VImageTex.GetNativeTexturePtr());
    }
    
    private IEnumerator CallPluginAtEndOfFrames()
    {
        while (true)
        {
            // Wait until all frame rendering is done
            yield return new WaitForEndOfFrame();

            // Issue a plugin event with arbitrary integer identifier.
            // The plugin can distinguish between different
            // things it needs to do based on this ID.
            // For our simple plugin, it does not matter which ID we pass here.
            SetTimestampFromUnity(currentPositionTimestamp);
            GL.IssuePluginEvent(GetRenderEventFunc(), 1);
            //Debug.Log("CallPluginAtEndOfFrames");

            // skip one frame
            yield return new WaitForEndOfFrame();
        }
    }
    
    public void OnSaveButton()
    {
        if (measure_is_running)
        {
            measure_is_running = false;
            Thread thread = new Thread(() => SaveMeasureData());
            thread.Start();
        }
    }

    void SaveMeasureData()
    {
        string id = DateTime.Now.ToLongTimeString();
        string path = filePath +"_times_"+ id+".txt";
        sw1 = File.CreateText(path);
        sw1.WriteLine("Start,Stop,Position,Rotation");

        foreach (MeasureData delta in measureList)
        {
            sw1.WriteLine(delta.startTime.ToString() + "," + delta.stopTime.ToString() + ",(" + delta.position.x + "," + delta.position.y + "," + delta.position.z + "),(" + delta.rotation.x + "," + delta.rotation.y + "," + delta.rotation.z + "," + delta.rotation.w + ")");
        }
        sw1.WriteLine("--end--");
        sw1.Close();
    }

    private void OnApplicationPause(bool pause)
    {
        DeinitCamera();
        //foreach (MeasureData delta in measureList)
        //{
        //    sw1.WriteLine(delta.startTime.ToString() + "," + delta.stopTime.ToString() + "," + delta.position.ToString()+","+delta.rotation.ToString());
        //}
        //sw1.WriteLine("--end--");
        //sw1.Close();
        //foreach (float delta in timesWithServerCalc)
        //{
        //    sw2.WriteLine(delta.ToString());
        //}
        //sw2.Close();
    }

}
