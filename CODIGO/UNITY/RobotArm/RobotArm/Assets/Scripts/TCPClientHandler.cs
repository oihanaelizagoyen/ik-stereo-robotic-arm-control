using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class TCPClientHandler
{

    private IPEndPoint _serverEndPoint;
    private Socket _clientSocket;

    // Constructor (inizializes the endpoint and the socket)
    public TCPClientHandler(string ipAddress, int port)
    {
        _serverEndPoint = new IPEndPoint(IPAddress.Parse(ipAddress), port);
        _clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
    }

    // Connect to the server
    public bool Connect()
    {
        if (!_clientSocket.Connected)
        {
            try
            {
                _clientSocket.Connect(_serverEndPoint);
                Debug.Log("TCP Connect - Connected to server.");
                return true;
            }
            catch (Exception ex)
            {
                Debug.Log($"TCP Connect - Connection error: {ex.Message}");
                return false;
            }
        }
        Debug.Log("TCP Connect - Already connected.");
        return true;
    }

    // Check if the client is connected
    public bool IsConnected()
    {
        return _clientSocket.Connected;
    }

    // Send a message to the server
    public void SendMessage(string message)
    {
        if (_clientSocket.Connected)
        {
            try
            {
                byte[] messageBytes = Encoding.UTF8.GetBytes(message);
                _clientSocket.Send(messageBytes);
                Debug.Log($"TCP Send - Message sent: {message}");
            }
            catch (SocketException se) when (se.SocketErrorCode == SocketError.ConnectionReset || se.SocketErrorCode == SocketError.Shutdown)
            {
                Debug.Log("TCP Send - Lost connection to server.");
                Disconnect();
            }
            catch (Exception ex)
            {
                Debug.Log($"TCP Send - Send error: {ex.Message}");
            }
        }
        else
        {
            Debug.Log("TCP Send - Not connected to server.");
        }
    }

    // Disconnect from the server
    public void Disconnect()
    {
        if (_clientSocket.Connected)
        {
            try
            {
                _clientSocket.Shutdown(SocketShutdown.Both); //Block communication in both directions.
                _clientSocket.Close();
                Debug.Log("TCP Disconnect - Disconnected from server.");
            }
            catch (Exception ex)
            {
                Debug.Log($"TCP Disconnect - Disconnection error: {ex.Message}");
            }
        }
        else
        {
            Debug.Log("TCP Disconnect - Already disconnected.");
        }
    }
}
