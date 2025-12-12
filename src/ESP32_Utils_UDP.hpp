#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>

#define UDP_TX_PACKET_MAX_SIZE 2048

extern WiFiUDP UDP;
extern unsigned int localPort;
extern unsigned int remotePort;
extern const char *ip_server;

// Parser del mensaje
extern void processMessage(const char *);

// 🔒 mismo mutex para TODAS las operaciones con UDP
extern SemaphoreHandle_t udpMutex;

// Para responder ACK al último remitente
static IPAddress lastSenderIP;
static uint16_t lastSenderPort = 0;

// Buffer único de recepción
static char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// Prototipos
inline bool udp_send_bytes(const uint8_t *data, size_t len, const char *host, uint16_t port);
inline void SendUDP_PacketToServer(const char *data, size_t len);
inline void SendUDP_PacketToLastPeer(const char *data, size_t len);

// -------------------- Conexión --------------------
inline bool ConnectUDP()
{
    Serial.println();
    Serial.println("Starting UDP");

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected yet");
        return false;
    }
    if (UDP.begin(localPort) != 1)
    {
        Serial.println("UDP begin() failed");
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    Serial.println("UDP successful");
    return true;
}

// -------------------- Rx loop --------------------
inline void GetUDP_Packet(bool /*sendACK*/)
{
    if (!udpMutex)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }

    if (xSemaphoreTake(udpMutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }

    int packetSize = UDP.parsePacket();
    if (packetSize > 0)
    {
        int len = UDP.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE - 1);
        if (len < 0)
            len = 0;
        packetBuffer[len] = '\0';

        // Actualiza remitente bajo el mismo mutex
        lastSenderIP = UDP.remoteIP();
        lastSenderPort = UDP.remotePort();

        xSemaphoreGive(udpMutex);

        Serial.printf("\nRx %d bytes from %s:%u\n",
                      packetSize, lastSenderIP.toString().c_str(), lastSenderPort);
        Serial.print("Payload: ");
        Serial.write((uint8_t *)packetBuffer, (size_t)len);
        Serial.println();

        processMessage(packetBuffer);
    }
    else
    {
        xSemaphoreGive(udpMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}

// -------------------- Tx helpers --------------------
inline bool udp_send_bytes(const uint8_t *data, size_t len, const char *host, uint16_t port)
{
    if (!udpMutex)
        return false;
    if (WiFi.status() != WL_CONNECTED)
        return false;

    IPAddress ip;
    if (!ip.fromString(host))
    {
        // Si algún día usas hostname real, resuélvelo y cachea aquí.
        return false;
    }

    bool ok = false;
    if (xSemaphoreTake(udpMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        auto try_send = [&]() -> bool
        {
            if (UDP.beginPacket(ip, port) != 1)
                return false;
            size_t w = UDP.write(data, len);
            if (w != len)
            {
                UDP.endPacket();
                return false;
            }
            return UDP.endPacket() == 1;
        };
        ok = try_send();
        if (!ok)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            ok = try_send();
        }
        xSemaphoreGive(udpMutex);
    }
    return ok;
}

// Enviar a server
inline void SendUDP_PacketToServer(const char *data, size_t len)
{
    udp_send_bytes(reinterpret_cast<const uint8_t *>(data), len, ip_server, remotePort);
}

// Enviar al último peer (si existe)
inline void SendUDP_PacketToLastPeer(const char *data, size_t len)
{
    // Copiamos bajo el mutex para coherencia
    IPAddress ip;
    uint16_t port;
    if (!udpMutex)
        return;
    if (xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5)) != pdTRUE)
        return;
    ip = lastSenderIP;
    port = lastSenderPort;
    xSemaphoreGive(udpMutex);
    if (ip == IPAddress() || port == 0)
        return;
    udp_send_bytes(reinterpret_cast<const uint8_t *>(data), len, ip.toString().c_str(), port);
}

// API “amigable”: decide destino (peer conocido o server)
inline void SendUDP_Packet(const String &data)
{
    // Tomamos snapshot del peer bajo mutex
    IPAddress ip;
    uint16_t port;
    if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        ip = lastSenderIP;
        port = lastSenderPort;
        xSemaphoreGive(udpMutex);
    }

    if (port != 0 && ip != IPAddress())
    {
        udp_send_bytes(reinterpret_cast<const uint8_t *>(data.c_str()), data.length(),
                       ip.toString().c_str(), port);
    }
    else
    {
        udp_send_bytes(reinterpret_cast<const uint8_t *>(data.c_str()), data.length(),
                       ip_server, remotePort);
    }
}
