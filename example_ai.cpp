// This illustrates how you might

#include "simulator.h"
#include <stdio.h>
int main(int argc, char **argv)
{
    // The second parameter specifies whether or not
    // we want the socket to be non-blocking. A blocking
    // socket will stall the calling thread until a packet
    // is available. Here I open a blocking socket, which
    // prevents the loop below from burning cycles.
    udp_open(12345, false);

    // In your application you probably want to use
    // non-blocking sockets and poll once per roll
    // in your own main loop. Like so:

    // State latest_state
    // while (running)
    //     state_updated = false
    //     State state
    //     if (udp_recv(state))
    //         latest_data = state
    //         // Process new data if you need to
    //
    //     // ... Your AI algorithm here ...
    //

    // If your main loop is running much slower than the
    // send rate of packets, and you _need_ to process
    // all the packets that are sent, then you will need
    // to poll for and process packets on a seperate thread.

    while (1)
    {
        Robot data = {};
        udp_addr sender = {};

        udp_recv((char*)&data, sizeof(Robot), &sender);
        printf("Received data from %d.%d.%d.%d:%d\n",
               sender.ip0, sender.ip1, sender.ip2, sender.ip3, sender.port);

        printf("%.2f %.2f %.2f %.2f %.2f\n",
               data.x, data.y, data.q, data.vl, data.vr);

        printf("\n");
    }

    udp_close();
}
