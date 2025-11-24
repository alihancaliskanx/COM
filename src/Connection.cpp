// Connection.cpp
#include "Connection.h"
#include <iostream>
#include <fcntl.h>   // Dosya kontrol tanımları
#include <errno.h>   // Hata numarası tanımları
#include <termios.h> // POSIX terminal kontrol tanımları
#include <unistd.h>  // UNIX standart tanımları
#include <cstring>   // memset vb. için

SerialConnection::SerialConnection() : serialPort(-1), connected(false) {}

SerialConnection::~SerialConnection()
{
    disconnect();
}

bool SerialConnection::connect(const std::string &portName, int baudRate)
{
    // Portu aç: Read/Write, No Controlling TTY, No Delay
    serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY);

    if (serialPort < 0)
    {
        std::cerr << "Hata: " << portName << " acilamadi. Kod: " << errno << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Mevcut ayarları al, alamazsan hata ver
    if (tcgetattr(serialPort, &tty) != 0)
    {
        std::cerr << "Hata: tty ayarlari alinamadi." << std::endl;
        return false;
    }

    // --- Baud Rate Ayarı (Örnek olarak 9600 ve 115200 eklendi) ---
    speed_t speed;
    if (baudRate == 115200)
        speed = B115200;
    else
        speed = B9600; // Varsayılan

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // --- Kontrol Ayarları (8N1 - Standart Seri Haberleşme) ---
    tty.c_cflag &= ~PARENB;        // Parity biti yok
    tty.c_cflag &= ~CSTOPB;        // 1 stop biti
    tty.c_cflag &= ~CSIZE;         // Maskeyi temizle
    tty.c_cflag |= CS8;            // 8 data biti
    tty.c_cflag |= CREAD | CLOCAL; // Okumayı aç ve modem kontrol hatlarını yoksay

    // --- Input Ayarları ---
    // Özel karakterleri işleme (Raw mod için gerekli)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Yazılımsal akış kontrolünü kapat
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // --- Output Ayarları ---
    tty.c_oflag &= ~OPOST; // İşlenmiş çıktıyı kapat
    tty.c_oflag &= ~ONLCR; // Newline -> CR/LF dönüşümünü kapat

    // --- Local Mod Ayarları ---
    tty.c_lflag &= ~ICANON; // Canonical modu kapat (Satır satır değil, bayt bayt oku)
    tty.c_lflag &= ~ECHO;   // Echo'yu kapat (Gönderdiğini geri okuma)
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG; // Sinyal karakterlerini kapat

    // --- Okuma Ayarları ---
    // VMIN ve VTIME, read() fonksiyonunun bloklama davranışını belirler.
    tty.c_cc[VMIN] = 1;  // En az 1 bayt gelene kadar bekle (Blokla)
    tty.c_cc[VTIME] = 0; // Zaman aşımı yok (Sonsuz bekleme)

    // Ayarları uygula
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
    {
        std::cerr << "Hata: Ayarlar uygulanamadi." << std::endl;
        return false;
    }

    connected = true;
    std::cout << "Baglanti basarili: " << portName << std::endl;
    return true;
}

int SerialConnection::readData(char *buffer, unsigned int bufSize)
{
    if (!connected)
        return -1;

    // read() fonksiyonu VMIN ayarından dolayı veri gelene kadar bekler
    int n = read(serialPort, buffer, bufSize);
    return n; // Okunan bayt sayısını döndür
}

void SerialConnection::disconnect()
{
    if (connected && serialPort >= 0)
    {
        close(serialPort);
        connected = false;
        serialPort = -1;
        std::cout << "Port kapatildi." << std::endl;
    }
}

bool SerialConnection::isConnected() const
{
    return connected;
}

bool SerialConnection::ConnectionHeartbeat(int timeout)
{
    int out = 0;
    while (true)
    {

        if (!connected)
        {
            continue;
            if (out > timeout)
            {
                return false;
            }
        }
        else
        {
            std::cout << "[INFO] There is heartbeat signal" << std::endl;
            std::clog << "[INFO] There is heartbeat signal" << std::endl;
            return true;
        }
    }
}