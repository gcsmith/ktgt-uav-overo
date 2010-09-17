#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

int main()
{
    int ld;
    struct sockaddr_in skaddr;
    socklen_t length;
    char *msg = "Datagram from server.";

    if ((ld = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        fprintf(stderr, "Problem creating socket.\n");
        exit(1);
    }

    skaddr.sin_family = AF_INET;
    skaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    skaddr.sin_port = htons(2010);

    if (bind(ld, (struct sockaddr *)&skaddr, sizeof(skaddr)) < 0)
    {
        fprintf(stderr, "Problem binding.\n");
        exit(0);
    }

    length = sizeof(skaddr);
    if (getsockname(ld, (struct sockaddr *)&skaddr, &length) < 0)
    {
        fprintf(stderr, "Error getsockname.\n");
        exit(1);
    }

    fprintf(stderr, "The server UDP port number is %d.\n", ntohs(skaddr.sin_port));

    while (1)
    {
        if (sendto(ld, msg, sizeof(msg), 0, (struct sockaddr *)&skaddr, sizeof(skaddr)) < 0)
        {
            fprintf(stderr, "Error sendto\n");
            exit(1);
        }
    }

    return 0;
}
