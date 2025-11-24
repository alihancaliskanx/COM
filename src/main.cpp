// main.cpp
#include <iostream>
#include <thread>
#include <chrono>
#include "Connection.h"
#include <cstring> // memset vb. için

// Linux'ta USB portları genelde /dev/ttyUSB0 veya /dev/ttyACM0 olur.
// Windows'ta ise "COM3", "COM4" şeklinde string verilir (Windows uyarlaması gerekir).
#define PORT_NAME "/dev/ttyUSB0"
#define BAUD_RATE 9600

int main()
{
    SerialConnection serial;
    char buffer[256]; // Okuma tamponu

    std::cout << "Seri port okuyucu baslatiliyor..." << std::endl;

    // Bağlantı denemesi
    if (!serial.connect(PORT_NAME, BAUD_RATE))
    {
        std::cerr << "Baglanti kurulamadi! Cihazi kontrol edin." << std::endl;
        return 1;
    }

    // Durmadan okuyan ana döngü
    while (serial.isConnected())
    {
        // Tamponu temizle
        memset(buffer, 0, sizeof(buffer));

        // Veri oku (Bloklayıcı moddadır, veri gelene kadar burada bekler)
        int bytesRead = serial.readData(buffer, sizeof(buffer) - 1);

        if (bytesRead > 0)
        {
            // Gelen veriyi string olarak işle veya ekrana bas
            std::string receivedData(buffer);
            std::cout << "[Gelen Veri - " << bytesRead << " byte]: " << receivedData << std::endl;

            // HEX olarak görmek isterseniz:
            /*
            std::cout << "HEX: ";
            for(int i=0; i<bytesRead; i++)
                printf("%02X ", (unsigned char)buffer[i]);
            std::cout << std::endl;
            */
        }
        else if (bytesRead < 0)
        {
            std::cerr << "Okuma hatasi!" << std::endl;
            break;
        }

        // CPU'yu %100 kullanmamak için çok küçük bir uyku eklenebilir (isteğe bağlı)
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    serial.disconnect();
    return 0;
}