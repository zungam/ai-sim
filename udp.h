/*
udp.h
--------------
UDP socket wrapper for OSX, Linux and Windows
Provides basic socket functionality of opening,
closing a socket, and sending and receiving data.

How to compile
--------------
This file contains both the header file and the implementation file.
To create the implementation in one source file include this header:

    #define UDP_IMPLEMENTATION
    #include "udp.h"

#define UDP_ASSERT if you do not want to include <cassert>.
*/

#ifndef UDP_HEADER_INCLUDE
#define UDP_HEADER_INCLUDE
#include <stdint.h>
#ifndef UDP_ASSERT
#include <cassert>
#define UDP_ASSERT assert
#endif

struct udp_addr
{
    uint8_t ip0, ip1, ip2, ip3; // represents ip0.ip1.ip2.ip3
    uint16_t port;
};

bool udp_open(uint16_t listen_port, bool non_blocking);
int  udp_recv(char *data, uint32_t size, udp_addr *src);
int  udp_send(char *data, uint32_t size, udp_addr dst);
void udp_close();

#ifdef UDP_IMPLEMENTATION

#if defined(__linux) || defined(__APPLE__)
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>

static int udp_socket = 0;

bool udp_open(uint16_t listen_port, bool non_blocking)
{
    network.initialized = false;
    network.socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (network.socket < 0)
    {
        // Failed to open socket
        UDP_ASSERT(false);
        return false;
    }

    struct sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listen_port);

    if (bind(network.socket, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        // Failed to bind socket
        UDP_ASSERT(false);
        return false;
    }

    return true;
}

int udp_recv(char *data, uint32_t max_size, udp_addr *src)
{
    if (!udp_socket)
    {
        // Socket not initialized
        UDP_ASSERT(false);
    }

    struct sockaddr_in from;
    int from_length = sizeof(from);
    int bytes_read = recvfrom(
        network.socket, data, max_size, 0,
        (struct sockaddr*)&from, &from_length);
    if (bytes_read <= 0)
        return 0;

    uint32 from_address = ntohl(from.sin_addr.s_addr);
    src->ip0  = (from_address >> 24) & 0xff;
    src->ip1  = (from_address >> 16) & 0xff;
    src->ip2  = (from_address >>  8) & 0xff;
    src->ip3  = (from_address >>  0) & 0xff;
    src->port = ntohs(from.sin_port);

    return bytes_read;
}

int udp_send(char *data, uint32_t size, udp_addr dst)
{
    if (!udp_socket)
    {
        // Socket not initialized
        UDP_ASSERT(false);
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(
        (dst.ip0 << 24) |
        (dst.ip1 << 16) |
        (dst.ip2 <<  8) |
        (dst.ip3));
    address.sin_port = htons(dst.port);

    int bytes_sent = sendto(network.socket, data, size, 0,
        (struct sockaddr*)&address, sizeof(struct sockaddr_in));

    return bytes_sent;
}

void udp_close()
{
    // Nothing to do here!
}

#elif _WIN32
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
static uint32_t udp_socket = 0;

bool udp_open(uint16_t listen_port, bool non_blocking)
{
    WSADATA WsaData;
    if (WSAStartup(MAKEWORD(2, 2), &WsaData) != NO_ERROR)
    {
        // Windows failure
        UDP_ASSERT(false);
        return false;
    }

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket <= 0)
    {
        // Failed to create socket
        UDP_ASSERT(false);
        return false;
    }

    // Bind socket to a port
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listen_port);
    if (bind(udp_socket, (const sockaddr*)&address, sizeof(sockaddr_in)) < 0)
    {
        // Failed to bind socket (maybe port was taken?)
        UDP_ASSERT(false);
        return false;
    }

    if (non_blocking)
    {
        // Set port to not block when calling recvfrom
        DWORD non_blocking = 1;
        if (ioctlsocket(udp_socket, FIONBIO, &non_blocking) != 0)
        {
            // Failed to set port to non-blocking
            UDP_ASSERT(false);
            return false;
        }
    }

    return true;
}

int udp_recv(char *data, uint32_t size, udp_addr *src)
{
    if (!udp_socket)
    {
        // Socket not initialized
        UDP_ASSERT(false);
    }

    sockaddr_in from;
    int from_length = sizeof(from);
    int bytes_read = recvfrom(
        udp_socket, data, size, 0, (sockaddr*)&from, &from_length);

    if (bytes_read <= 0)
        return 0;

    uint32_t from_address = ntohl(from.sin_addr.s_addr);
    if (src)
    {
        src->ip0 = (from_address >> 24) & 0xff;
        src->ip1 = (from_address >> 16) & 0xff;
        src->ip2 = (from_address >>  8) & 0xff;
        src->ip3 = (from_address >>  0) & 0xff;
        src->port = ntohs(from.sin_port);
    }
    return bytes_read;
}

int udp_send(char *data, uint32_t size, udp_addr dst)
{
    if (!udp_socket)
    {
        // Socket not initialized
        UDP_ASSERT(false);
    }

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(
        (dst.ip0 << 24) |
        (dst.ip1 << 16) |
        (dst.ip2 <<  8) |
        (dst.ip3));
    address.sin_port = htons(dst.port);
    int bytes_sent = sendto(udp_socket, data, size,
        0, (sockaddr*)&address, sizeof(sockaddr_in));
    return bytes_sent;
}

void udp_close()
{
    WSACleanup();
}
#endif

#endif // UDP_IMPLEMENTATION
#endif // UDP_HEADER_INCLUDE
