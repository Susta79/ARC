using System;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;

public class EthSocketServer : MonoBehaviour
{
    public Transform[] Robot = new Transform[6];
    System.Threading.Thread SocketThread;
    volatile bool keepReading = false;
    volatile double[] joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Start is called before the first frame update
    void Start()
    {
        Application.runInBackground = true;
        startServer();
    }

    void startServer()
    {
        SocketThread = new System.Threading.Thread(networkCode);
        SocketThread.IsBackground = true;
        SocketThread.Start();
    }

    private string getIPAddress()
    {
        IPHostEntry host;
        string localIP = "";
        host = Dns.GetHostEntry(Dns.GetHostName());
        foreach (IPAddress ip in host.AddressList)
        {
            if (ip.AddressFamily == AddressFamily.InterNetwork)
            {
                localIP = ip.ToString();
            }

        }
        return localIP;
    }

    Socket listener;
    Socket handler;

    void networkCode()
    {
        string data;

        // Data buffer for incoming data.
        byte[] bytes = new Byte[1024];

        // host running the application.
        //Debug.Log("Ip " + getIPAddress().ToString());
        Debug.Log("Ip " + "127.0.0.1");
        //IPAddress[] ipArray = Dns.GetHostAddresses(getIPAddress());
        IPAddress[] ipArray = Dns.GetHostAddresses("127.0.0.1");
        IPEndPoint localEndPoint = new IPEndPoint(ipArray[0], 1755);

        // Create a TCP/IP socket.
        listener = new Socket(ipArray[0].AddressFamily,
            SocketType.Stream, ProtocolType.Tcp);

        // Bind the socket to the local endpoint and 
        // listen for incoming connections.

        try
        {
            listener.Bind(localEndPoint);
            listener.Listen(10);

            // Start listening for connections.
            while (true)
            {

                // Program is suspended while waiting for an incoming connection.
                Debug.Log("Waiting for Connection");     //It works

                handler = listener.Accept();
                Debug.Log("Client Connected");     //It doesn't work
                while (handler.Connected)
                {
                    Debug.Log("6");
                    // An incoming connection needs to be processed.
                    data = null;
                    keepReading = true;
                    while (keepReading)
                    {
                        Debug.Log("8");
                        bytes = new byte[1024];
                        int bytesRec = 0;
                        if (handler.Connected) {
                            bytesRec = handler.Receive(bytes);
                            Debug.Log("Received from Server. Bytes: " + bytesRec);
                        }
                        if (bytesRec <= 0)
                        {
                            Debug.Log("bytesRec <= 0");
                            keepReading = false;
                            Debug.Log("handler.Connected: " + handler.Connected.ToString());
                            break;
                        } else {
                            Debug.Log("bytesRec > 0");
                            data += Encoding.ASCII.GetString(bytes, 0, bytesRec);
                            if (data.IndexOf("\n") > -1)
                            {
                                Debug.Log("Received: " + data);
                                data = data.Replace('.', ',');
                                String[] datas = data.Split(" ", StringSplitOptions.None);
                                Debug.Log("datas Length: " + datas.Length);
                                if (datas.Length == 6) {
                                    double.TryParse(datas[0], out joints[0]);
                                    double.TryParse(datas[1], out joints[1]);
                                    double.TryParse(datas[2], out joints[2]);
                                    double.TryParse(datas[3], out joints[3]);
                                    double.TryParse(datas[4], out joints[4]);
                                    double.TryParse(datas[5], out joints[5]);
                                    Debug.Log("datas 0: " + joints[0].ToString());
                                    Debug.Log("datas 1: " + joints[1].ToString());
                                    Debug.Log("datas 2: " + joints[2].ToString());
                                    Debug.Log("datas 3: " + joints[3].ToString());
                                    Debug.Log("datas 4: " + joints[4].ToString());
                                    Debug.Log("datas 5: " + joints[5].ToString());
                                }
                                handler.Send(Encoding.ASCII.GetBytes(data));
                            }
                            break;
                        }
                        Debug.Log("1");
                        System.Threading.Thread.Sleep(1);
                    }
                    Debug.Log("2");
                    if (handler.Connected)
                        handler.Disconnect(false);
                    Debug.Log("9");
                    System.Threading.Thread.Sleep(1);
                }
                Debug.Log("3");
                handler.Close();
                Debug.Log("4");
                listener.Bind(localEndPoint);
                Debug.Log("5");
                listener.Listen(10);

                System.Threading.Thread.Sleep(1);
            }
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
            Debug.Log("7");
        }
    }

    void stopServer()
    {
        keepReading = false;

        //stop thread
        if (SocketThread != null)
        {
            SocketThread.Abort();
        }

        if (handler != null && handler.Connected)
        {
            handler.Disconnect(false);
            Debug.Log("Disconnected!");
        }
    }

    void OnDisable()
    {
        stopServer();
    }
    
    // Update is called once per frame
    void Update()
    {
        //update real joint axes positions
        Robot [0].localEulerAngles = new Vector3 (0,(float)-joints[0],0);
        Robot [1].localEulerAngles = new Vector3 (0,0,(float)-joints[1]);
        Robot [2].localEulerAngles = new Vector3 (0,0,(float)-joints[2]);
        Robot [3].localEulerAngles = new Vector3 ((float)-joints[3],0,0);
        Robot [4].localEulerAngles = new Vector3 (0,0,(float)-joints[4]);
        Robot [5].localEulerAngles = new Vector3 ((float)-joints[5],0,0);
    }
}




