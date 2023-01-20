using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuManager : MonoBehaviour
{
    public TextMeshProUGUI ipaddress;

    public TextMeshProUGUI rostopic;

    string fileName = "/lastIpAddresses.dat";
    public GameObject menuCanvas;
    IpAdresses ipAddresses;

    string ip;
    string topic;


    // Start is called before the first frame update
    void Start()
    {
        if (LoadIps())
        {
            Debug.Log("ip loaded: " + ipAddresses.rosServer + " , " + ipAddresses.rosTopic);
            ipaddress.text= ipAddresses.rosServer + " ";
            ip = ipAddresses.rosServer;
            rostopic.text = ipAddresses.rosTopic + " ";
            topic = ipAddresses.rosTopic;
            Debug.Log("Textfield filled "+ ipaddress.text+ rostopic.text);
        }
    }

    public void OnStartButton()
    {
        Debug.Log("OnStartButton ipaddress:" + ipaddress.text);

        if (ipaddress.text.Length > 0)
        {
            ip = ipaddress.text.Substring(0, ipaddress.text.Length - 1);
            topic = rostopic.text.Substring(0, rostopic.text.Length - 1);
            SaveIps(ip, topic);
        }

        RosConnector.RosBridgeServerUrl = "ws://" + ip + ":9090";
        ImuStreamerService.rosIpAddress = ip;
        NativeCameraHandler.ipAddress = ip;

        PoseStampedSubscriber.RosTopic = topic;

        UnityEngine.SceneManagement.SceneManager.LoadScene("GameScene", LoadSceneMode.Single);
    }


    private bool LoadIps()
    {
        if (File.Exists(Application.persistentDataPath + fileName))
        {
            BinaryFormatter binaryFormatter = new BinaryFormatter();
            FileStream file = File.OpenRead(Application.persistentDataPath + fileName);
            if (ipAddresses == null)
            {
                ipAddresses = new IpAdresses();
            }
            ipAddresses = (IpAdresses)binaryFormatter.Deserialize(file);
            file.Close();

            Debug.Log("ip loaded: "+ ipAddresses.rosServer + " , " + ipAddresses.rosTopic);
            return true;

        }
        else
        {
            Debug.Log("Can't load ips");
            return false;
        }
    }

    private void SaveIps( string rosServer, string rostopic)
    {
        BinaryFormatter binaryFormatter = new BinaryFormatter();
        FileStream file = File.Open(Application.persistentDataPath + fileName, FileMode.OpenOrCreate);
        if (ipAddresses == null)
        {
            ipAddresses = new IpAdresses();
        }

        ipAddresses.rosServer = rosServer;
        ipAddresses.rosTopic = rostopic;
        binaryFormatter.Serialize(file, ipAddresses);
        file.Close();
    }
}

[Serializable]
class IpAdresses
{
    public string rosServer;
    public string rosTopic;
}