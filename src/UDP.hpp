#include <ArduinoJson.h>
using namespace ArduinoJson;

WiFiUDP UDP;
SemaphoreHandle_t udpMutex;

unsigned int localPort = 8888;
unsigned int remotePort = 8889;

extern WiFiUDP UDP;
extern const char *ip_server;

bool udp_send_bytes(const uint8_t *data, size_t len, const char *host, uint16_t port);
extern void processMessage(const char *);
void SendTelemetry()
{
    static bool printed_info = false;
    if (!printed_info)
    {
        Serial.println("===== TELEMETRY CONFIG =====");
        Serial.print("ESP32 local IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Dest (PC) IP  : ");
        Serial.println(ip_server);
        Serial.print("Dest (PC) Port: ");
        Serial.println(remotePort);
        Serial.println("============================");
        printed_info = true;
    }

    // 5 Hz
    const unsigned long telemetryInterval = 10;
    static unsigned long lastTelemetry = 0;
    unsigned long now = millis();
    if (now - lastTelemetry < telemetryInterval)
        return;
    lastTelemetry = now;

    JsonDocument doc;

    doc["AngleRoll_est"] = AngleRoll_est;
    doc["AnglePitch_est"] = AnglePitch_est;
    doc["AngleYaw"] = AngleYaw;
    doc["gyroRateRoll"] = gyroRateRoll;
    doc["gyroRatePitch"] = gyroRatePitch;
    doc["RateYaw"] = RateYaw;
    doc["AccX"] = AccX;
    doc["AccY"] = AccY;
    doc["AccZ"] = AccZ;
    doc["tau_x"] = tau_x;
    doc["tau_y"] = tau_y;
    doc["tau_z"] = tau_z;
    doc["AngleRoll"] = AngleRoll;
    doc["AnglePitch"] = AnglePitch;
    doc["error_phi"] = error_phi;
    doc["error_theta"] = error_theta;
    doc["InputThrottle"] = InputThrottle;
    doc["DesiredAngleRoll"] = DesiredAngleRoll;
    doc["DesiredAnglePitch"] = DesiredAnglePitch;
    doc["DesiredRateYaw"] = DesiredRateYaw;
    doc["MotorInput1"] = MotorInput1;
    doc["MotorInput2"] = MotorInput2;
    doc["MotorInput3"] = MotorInput3;
    doc["MotorInput4"] = MotorInput4;
    doc["k1"] = k1;
    doc["k2"] = k2;
    doc["g1"] = g1;
    doc["g2"] = g2;
    doc["m1"] = m1;
    doc["m2"] = m2;
    doc["modoActual"] = modoActual;

    // Calcula tamaño y serializa a buffer de stack
    size_t need = measureJson(doc);
    if (need >= 1472)
    { // límite seguro de UDP payload < MTU
        Serial.printf("⚠️ Telemetría grande (%u B), recorta campos\n", (unsigned)need);
        // Opcional: quitar campos menos críticos antes de enviar
    }

    char buf[1200]; // suficiente para tu payload actual
    size_t len = serializeJson(doc, buf, sizeof(buf));
    if (!len)
    {
        Serial.println("❌ serializeJson devolvió 0");
        return;
    }

    if (!udp_send_bytes(reinterpret_cast<const uint8_t *>(buf), len, ip_server, remotePort))
    {
        // Si falla, no spamees logs cada tick
        static uint32_t lastErr = 0;
        if (millis() - lastErr > 1000)
        {
            Serial.println("❌ UDP telemetría: fallo de envío");
            lastErr = millis();
        }
    }
}
