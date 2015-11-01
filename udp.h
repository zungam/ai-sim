#ifndef UDP_HEADER_INCLUDE
#define UDP_HEADER_INCLUDE
#include <stdint.h>
typedef uint64_t    u64;
typedef uint32_t    u32;
typedef uint16_t    u16;
typedef uint8_t     u08;
typedef int32_t     s32;
typedef int16_t     s16;
typedef int8_t      s08;

struct udp_addr
{
    u08 ip0, ip1, ip2, ip3; // represents ip0.ip1.ip2.ip3
    u16 port;
};

bool udp_open(u16 listen_port, bool non_blocking);
int  udp_recv(char *data, u32 size, udp_addr *src);
int  udp_send(char *data, u32 size, udp_addr dst);
void udp_close();

#ifdef UDP_IMPLEMENTATION

#if defined(__linux) || defined(__APPLE__)

#elif _WIN32
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
static u32 udp_socket = 0;

bool udp_open(u16 listen_port, bool non_blocking)
{
    WSADATA WsaData;
    if (WSAStartup(MAKEWORD(2, 2), &WsaData) != NO_ERROR)
        return false;

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket <= 0)
        return false;

    // Bind socket to a port
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listen_port);
    if (bind(udp_socket, (const sockaddr*)&address, sizeof(sockaddr_in)) < 0)
        return false;

    if (non_blocking)
    {
        // Set port to not block when calling recvfrom
        DWORD non_blocking = 1;
        if (ioctlsocket(udp_socket, FIONBIO, &non_blocking) != 0)
            return false;
    }

    return true;
}

int udp_recv(char *data, u32 size, udp_addr *src)
{
    sockaddr_in from;
    int from_length = sizeof(from);
    int bytes_read = recvfrom(
        udp_socket, data, size, 0, (sockaddr*)&from, &from_length);

    if (bytes_read <= 0)
        return 0;

    u32 from_address = ntohl(from.sin_addr.s_addr);
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

int udp_send(char *data, u32 size, udp_addr dst)
{
    if (!udp_socket)
        return 0;
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
