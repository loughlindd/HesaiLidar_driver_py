#include <iostream>

#include <iostream>
#include <streambuf>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#endif

class UdpStreamBuf : public std::streambuf {
public:
    UdpStreamBuf(const std::string& ip, int port) {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
#endif
        sock = socket(AF_INET, SOCK_DGRAM, 0);

        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
    }

    ~UdpStreamBuf() {
        sync();
#ifdef _WIN32
        closesocket(sock);
        WSACleanup();
#else
        close(sock);
#endif
    }

protected:
    int overflow(int c) override {
        if (c == EOF) return c;

        buffer += static_cast<char>(c);

        // send when newline
        if (c == '\n') {
            send_buffer();
        }

        return c;
    }

    int sync() override {
        if (!buffer.empty()) {
            send_buffer();
        }
        return 0;
    }

private:
    void send_buffer() {
        sendto(sock, buffer.c_str(), buffer.size(), 0,
               (sockaddr*)&addr, sizeof(addr));
        buffer.clear();
    }

    int sock;
    sockaddr_in addr{};
    std::string buffer;
};


class TeeStreamBuf : public std::streambuf {
public:
    TeeStreamBuf(std::streambuf* console_buf, std::streambuf* udp_buf) 
        : console_buf_(console_buf), udp_buf_(udp_buf) {}

protected:
    int overflow(int c) override {
        if (c == EOF) return c;
        
        // Write to both console and UDP
        console_buf_->sputc(c);
        udp_buf_->sputc(c);
        
        return c;
    }

    int sync() override {
        console_buf_->pubsync();
        udp_buf_->pubsync();
        return 0;
    }

private:
    std::streambuf* console_buf_;
    std::streambuf* udp_buf_;
};
