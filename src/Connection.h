// Connection.h
#ifndef CONNECTION_H
#define CONNECTION_H

#include <string>
#include <vector>

class SerialConnection
{
private:
    int serialPort; // Dosya tanımlayıcısı (File Descriptor)
    bool connected;

public:
    SerialConnection();
    ~SerialConnection();

    // Portu açar ve ayarları yapar (Örn: /dev/ttyUSB0, Baud: 9600)
    bool connect(const std::string &portName, int baudRate);

    // Porttan veri okur
    int readData(char *buffer, unsigned int bufSize);

    // Bağlantıyı kapatır
    void disconnect();

    // Bağlantı durumunu kontrol eder
    bool isConnected() const;

    // Heartbeat mesajını gönderir
    bool ConnectionHeartbeat(int timeout);
};

#endif // CONNECTION_H