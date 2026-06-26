/*
******************************************************************************
**  CarMaker - Version 14.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Raw Signal Data Stream example client for IPGMovie 8.0.
**
** This example looks quite complex at first but is actually quite simple.
** - establish the RSDS connection: connect()
** - get the RSDS data: recv_hdr() and get_data()
** everything else has to do with saving the data or actualising the statistics
**
** Have a look at rsds-client-camera-basics.c for a much simpler example.
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c -lws2_32
*/

#include <memory>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <getopt.h>
#include <fcntl.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

typedef enum
{
    SaveFormat_DataNotSaved = 0,
    SaveFormat_Raw,
    SaveFormat_PPM,
    SaveFormat_PGM_byte,
    SaveFormat_PGM_short,
    SaveFormat_PGM_float
} tSaveFormat;

enum class Encoding
{
    RGB8,
    MONO8,
    MONO16,
};

struct Image
{
    std::vector<char> data;
    Encoding encoding;
    size_t width;
    size_t height;
    size_t step;
    bool is_bigendian;
};

std::ostream &operator<<(std::ostream &os, Encoding encoding)
{
    switch (encoding)
    {
    case Encoding::RGB8:
        return os << "RGB8";
    case Encoding::MONO8:
        return os << "MONO8";
    default:
        return os << "MONO16";
    }
}

std::ostream &operator<<(std::ostream &os, const Image &image)
{
    return os << "Image[ encoding: " << image.encoding << ", width: " << image.width << ", height: " << image.height << ", size: " << image.data.size() << " ]";
}

typedef struct
{
    FILE *EmbeddedDataCollectionFile;
    std::string MovieHost; /* pc on which IPGMovie runs          */
    int MoviePort;         /* TCP/IP port for RSDS               */
    int sock;              /* TCP/IP Socket                      */
    char sbuf[64];         /* Buffer for transmitted information */
    int RecvFlags;         /* Receive Flags                      */
    int Verbose;           /* Logging Output     			      */
    int ConnectionTries;
    tSaveFormat SaveFormat;
    int TerminationRequested;
    int Channel;
    int CameraNo;
} RSDScfg;

typedef struct
{
    double tFirstDataTime;
    double tStartSim;
    double tEndSim;
    double tLastSimTime;
    unsigned long long int nBytesTotal;
    unsigned long long int nBytesSim;
    unsigned long int nImagesTotal;
    unsigned long int nImagesSim;
    unsigned char nChannels;
} RSDSIF;

class RSDS_Client
{
public:
    RSDS_Client(const std::string &host, int port, const std::string &dest_host = "", int dest_port = 0)
    {
        rsdscfg_.MovieHost = host;
        rsdscfg_.MoviePort = port;
        rsdscfg_.Verbose = 0;
        rsdscfg_.SaveFormat = SaveFormat_DataNotSaved;
        rsdscfg_.EmbeddedDataCollectionFile = NULL;
        rsdscfg_.RecvFlags = 0;
        rsdscfg_.ConnectionTries = 5;
        rsdscfg_.TerminationRequested = 0;
        rsdscfg_.Channel = 0;
        rsdscfg_.CameraNo = 0;

        rsdsif_.tFirstDataTime = 0.0;
        rsdsif_.tStartSim = 0.0;
        rsdsif_.tEndSim = 0.0;
        rsdsif_.tLastSimTime = -1.0;
        rsdsif_.nImagesSim = 0;
        rsdsif_.nImagesTotal = 0;
        rsdsif_.nBytesTotal = 0;
        rsdsif_.nBytesSim = 0;
        rsdsif_.nChannels = 0;

        dest_host_ = dest_host;
        dest_port_ = dest_port;
        dest_sock_ = -1;
    }

    void run();

private:
    /*! Scan TCP/IP Socket and writes to buffer */
    int recv_hdr(int sock, char *hdr);

    /* Connect over TCP/IP socket */
    int connect(void);

    /* Connect to destination forward server */
    int connect_destination(void);

    /* Forward raw image bytes over destination TCP connection */
    void forward_image_bytes(const Image &image);

    /*! Data and image processing */
    int get_data();

    void print_node_info();
    void print_sim_info();
    void print_closing_info();

    inline double get_time();

    // Helpers for RSDSIF : RSDS information ( stats about current status)
    void update_end_sim_time();
    // misc helpers
    void print_embedded_data(const char *data, unsigned int dataLen);

    /*! RSDS interface parameters structure */
    RSDSIF rsdsif_;

    /*! RSDS configuration structure */
    RSDScfg rsdscfg_;

    /* Destination Host Variables */
    std::string dest_host_;
    int dest_port_;
    int dest_sock_;
};

/*
 ** recv_hdr()
 **
 ** Scan TCP/IP Socket and writes to buffer
 */
int RSDS_Client::recv_hdr(int sock, char *hdr)
{
    constexpr int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1)
    {
        if (rsdscfg_.TerminationRequested)
            return -1;
        for (; len < HdrSize; len += i)
        {
            if ((i = recv(sock, hdr + len, HdrSize - len, rsdscfg_.RecvFlags)) <= 0)
            {
                if (!rsdscfg_.TerminationRequested)
                    printf("recv_hdr Error during recv (received: '%s' (%d))\n", hdr, len);
                return -1;
            }
        }
        if (hdr[0] == '*' && hdr[1] >= 'A' && hdr[1] <= 'Z')
        {
            /* remove white spaces at end of line */
            while (len > 0 && hdr[len - 1] <= ' ')
                len--;
            hdr[len] = 0;
            if (rsdscfg_.Verbose == 1 && nSkipped > 0)
                printf("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
            return 0;
        }
        for (i = 1; i < len && hdr[i] != '*'; i++)
            ;
        len -= i;
        nSkipped += i;
        memmove(hdr, hdr + i, len);
    }
}

/*
 ** connect()
 **
 ** Connect over TCP/IP socket
 */
int RSDS_Client::connect(void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2, 2), &WSAdata) != 0)
    {
        fprintf(stderr, "RSDS: WSAStartup ((2,2),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in DestAddr;
    struct hostent *he;
    int tries = rsdscfg_.ConnectionTries;

    if ((he = gethostbyname(rsdscfg_.MovieHost.c_str())) == NULL)
    {
        printf("RSDS: unknown host: %s\n", rsdscfg_.MovieHost.c_str());
        return -2;
    }
    DestAddr.sin_family = AF_INET;
    DestAddr.sin_port = htons((unsigned short)rsdscfg_.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    rsdscfg_.sock = socket(AF_INET, SOCK_STREAM, 0);

    while (::connect(rsdscfg_.sock, (struct sockaddr *)&DestAddr, sizeof(DestAddr)) < 0 && tries > 0)
    {
        printf("RSDS: can't connect '%s:%d'\n", rsdscfg_.MovieHost.c_str(), rsdscfg_.MoviePort);
        if (tries > 1)
        {
            printf("\tretrying in 1 second... (%d)\n", --tries);
            sleep(1);
        }
        else
        {
            return -4;
        }
    }
    if (recv_hdr(rsdscfg_.sock, rsdscfg_.sbuf) < 0)
        return -3;

    printf("RSDS: Connected: %s\n", rsdscfg_.sbuf + 1);

    memset(rsdscfg_.sbuf, 0, 64);

    return 0;
}

int RSDS_Client::connect_destination(void)
{
    if (dest_host_.empty() || dest_port_ == 0)
    {
        return 0;
    }

    struct sockaddr_in DestAddr;
    struct hostent *he;

    if ((he = gethostbyname(dest_host_.c_str())) == NULL)
    {
        printf("Destination: unknown host: %s\n", dest_host_.c_str());
        return -1;
    }

    DestAddr.sin_family = AF_INET;
    DestAddr.sin_port = htons((unsigned short)dest_port_);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    dest_sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    unsigned long mode = 0;
    int res = ioctlsocket(dest_sock_, FIONBIO, &mode);
    if (res == -1)
    {
        printf("Cannot make socket non-blocking\n");
        dest_sock_ = -1;
        return -2;
    }

    if (::connect(dest_sock_, (struct sockaddr *)&DestAddr, sizeof(DestAddr)) < 0)
    {
        printf("Destination: can't connect '%s:%d' -> %s\n", dest_host_.c_str(), dest_port_, strerror(errno));
        dest_sock_ = -1;
        return -2;
    }
    ioctlsocket(dest_sock_, FIONBIO, 0);
    
    printf("Destination: Connected to %s:%d\n", dest_host_.c_str(), dest_port_);
    return 0;
}

void RSDS_Client::forward_image_bytes(const Image &img)
{
    constexpr int DIM = 3;
    if (dest_sock_ < 0)
    {
        return;
    }

    size_t total_sent = 0;
    size_t to_send = img.data.size();
    const char *ptr = img.data.data();

    size_t meta[3] = {img.width, img.height, img.data.size()};
    // return;

    int sent = send(dest_sock_, reinterpret_cast<const char*>(meta), sizeof(meta), 0);
    if (sent < 0)
    {
        printf("Destination: Transmission failure 1. Closing target connection.\n");
        return;
    }

    while (total_sent < to_send)
    {
        int sent = send(dest_sock_, ptr + total_sent, to_send - total_sent, 0);
        if (sent < 0)
        {
            printf("Destination: Transmission failure 2: %s. Closing target connection.\n", strerror(errno));
#ifdef WIN32
            closesocket(dest_sock_);
#else
            close(dest_sock_);
#endif
            dest_sock_ = -1;
            break;
        }
        total_sent += sent;
    }
}

void RSDS_Client::run()
{
    int res;
    if ((res = connect()) != 0)
    {
        printf("Can't initialise RSDS Client (returns %d, %s)\n", res, res == -4 ? "No server" : strerror(errno));
        return;
    }

    if (!dest_host_.empty() && dest_port_ != 0)
    {
        int res = connect_destination();
        if (res != 0) return;
    }

    while (true)
    {
        /* Read from TCP/IP-Port and process the image */
        if (rsdscfg_.TerminationRequested || recv_hdr(rsdscfg_.sock, rsdscfg_.sbuf) != 0)
        {

            printf("Stopping\n");
            break;
        }

        printf("Getting data\n");
        get_data();
    }

#ifdef WIN32
    closesocket(rsdscfg_.sock);
    if (dest_sock_ >= 0)
        closesocket(dest_sock_);
    WSACleanup();
#else
    close(rsdscfg_.sock);
    if (dest_sock_ >= 0)
        close(dest_sock_);
#endif
}

/*
 ** get_data()
 **
 ** Data and image processing
 */
int RSDS_Client::get_data()
{
    unsigned int len = 0;
    ssize_t res = 0;
    int Channel;

    /* Variables for Image Processing */
    char ImgType[32], AniMode[16];
    int ImgWidth, ImgHeight, ImgLen, dataLen;
    float SimTime;

    static Image image{};

    if (strlen(rsdscfg_.sbuf) == 0)
    {
        printf("RSDS: empty buf\n");
    }
    else if (sscanf(rsdscfg_.sbuf, "*RSDS %d %s %f %dx%d %d", &Channel, ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6)
    {
        if (rsdscfg_.Verbose == 1)
            printf("[RSDS] %-6.3f : %-2d : %-8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);

        if (ImgLen > 0)
        {

            image.data.resize(ImgLen);

            // this is how we get the data
            for (int len = 0; len < ImgLen; len += res)
            {
                if ((res = recv(rsdscfg_.sock, image.data.data() + len, ImgLen - len, rsdscfg_.RecvFlags)) < 0)
                {
                    printf("RSDS: Socket Reading Failure\n");
                    break;
                }
            }

            const std::string encoding = static_cast<std::string>(ImgType);

            if (encoding == "rgb")
            {
                image.encoding = Encoding::RGB8;
            }
            else if (encoding == "grey")
            {
                image.encoding = Encoding::MONO8;
            }
            else if (encoding == "grey16")
            {
                image.encoding = Encoding::MONO16;
            }
            else if (encoding == "depth16")
            {
                image.encoding = Encoding::MONO16;
                // const _Float16* f = reinterpret_cast<const _Float16*>(image.data.data());
                // _Float16 max = 0;
                // for (int i = 0; i < ImgWidth * ImgHeight; i++) {
                //     max = max < f[i] ? f[i] : max;
                // }
                // printf("Max: %f\n", static_cast<double>(max));
            }
            else
            {
                printf("Incompatible image type/encoding: %s. Supported output formats: rgb, grey, grey16 and depth16.", ImgType);
            }

            image.width = static_cast<unsigned int>(ImgWidth);
            image.height = static_cast<unsigned int>(ImgHeight);
            image.step = 1;
            image.is_bigendian = false;
            // std::cout << image << "\n";
        }
        // needed for all channels, since we want the time until the last image
        update_end_sim_time();
    }
    else if (sscanf(rsdscfg_.sbuf, "*RSDSEmbeddedData %d %f %d %s", &Channel, &SimTime, &dataLen, AniMode) == 4)
    {
        printf("Getting depth\n");

        if (rsdscfg_.Verbose == 1)
            printf("Embedded Data: %d %f %d %s\n", &Channel, SimTime, dataLen, AniMode);

        if (dataLen > 0)
        {
            char *data = (char *)malloc(dataLen);

            // get the data
            for (int len = 0; len < dataLen; len += res)
            {
                if ((res = recv(rsdscfg_.sock, data + len, dataLen - len, rsdscfg_.RecvFlags)) < 0)
                {
                    printf("RSDS: Socket Reading Failure\n");
                    free(data);
                    break;
                }
            }

            free(data);
        }
    }
    else
    {
        printf("RSDS: not handled: %s\n", rsdscfg_.sbuf);
    }

    forward_image_bytes(image);

    return 0;
}

void RSDS_Client::print_node_info()
{
    printf("Node Parameters--------------------------\n");
    printf("MovieHost:             %s\n", rsdscfg_.MovieHost.c_str());
    printf("MoviePort:             %d\n", rsdscfg_.MoviePort);
    printf("TerminationRequested:  %d\n", rsdscfg_.TerminationRequested);
    printf("Channel:               %d\n", rsdscfg_.Channel);
}

void RSDS_Client::print_sim_info()
{
    double dtSimReal = rsdsif_.tEndSim - rsdsif_.tStartSim;
    // at least 1 sec of data is required
    if (dtSimReal > 1.0)
    {
        printf("\nLast Simulation------------------\n");
        double MiBytes = rsdsif_.nBytesSim / (1024.0 * 1024.0);
        printf("Duration: %.3f (real) %.3f (sim) -> x%.2f\n", dtSimReal, rsdsif_.tLastSimTime, rsdsif_.tLastSimTime / dtSimReal);
        printf("Channels: %d\n", rsdsif_.nChannels);
        printf("Images:   %ld (%.3f FPS)\n", rsdsif_.nImagesSim, rsdsif_.nImagesSim / dtSimReal);
        printf("Bytes:    %.3f MiB (%.3f MiB/s)\n\n", MiBytes, MiBytes / dtSimReal);
    }
    if (rsdscfg_.EmbeddedDataCollectionFile != NULL)
        fflush(rsdscfg_.EmbeddedDataCollectionFile);
}

void RSDS_Client::print_closing_info()
{
    // from the very first image to the very last
    double dtSession = rsdsif_.tEndSim - rsdsif_.tFirstDataTime;
    printf("\n-> Closing RSDS-Client...\n");

    // at least 1 sec of data is required
    if (dtSession > 1.0)
    {
        print_sim_info();
        printf("Session--------------------------\n");
        double MiBytes = rsdsif_.nBytesTotal / (1024.0 * 1024.0);
        printf("Duration: %g seconds\n", dtSession);
        printf("Images:   %ld (%.3f FPS)\n", rsdsif_.nImagesTotal, rsdsif_.nImagesTotal / dtSession);
        printf("Bytes:    %.3f MiB (%.3f MiB per second)\n", MiBytes, MiBytes / dtSession);
    }
    fflush(stdout);

    if (rsdscfg_.EmbeddedDataCollectionFile != NULL)
        fclose(rsdscfg_.EmbeddedDataCollectionFile);
}

// on a system with properly configured timers, calling this function should need less then 0.1us
inline double RSDS_Client::get_time() // in seconds
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}


void RSDS_Client::update_end_sim_time()
{
    rsdsif_.tEndSim = get_time();
}

void RSDS_Client::print_embedded_data(const char *data, unsigned int dataLen)
{
    double *buf = (double *)data;
    unsigned int len = dataLen / sizeof(double), i;
    for (i = 0; i < len; i++)
    {
        printf("(%d) %f ", i, buf[i]);
    }
    printf("\n");
}

int main(int argc, char *argv[])
{
    int opt;
    int port = 2210;
    std::string host = "127.0.0.1";
    std::string dest_host = "";
    int dest_port = 0;
    while ((opt = getopt(argc, argv, "p:h:d:x:")) != -1)
    {
        switch (opt)
        {
        case 'p':
            port = std::stoi(optarg);
            std::cout << "Setting port to '" << port << "'\n";
            break;
        case 'h':
            host = optarg;
            std::cout << "Setting host to '" << host << "'\n";
            break;
        case 'd':
            dest_host = optarg;
            std::cout << "Setting destination host to '" << dest_host << "'\n";
            break;
        case 'x': // maps to long option --dest_port
            dest_port = std::stoi(optarg);
            std::cout << "Setting destination port to '" << dest_port << "'\n";
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " (-h host) (-p port) (-d dest_host) (--dest_port port)\n";
            return 1;
        }
    }
    RSDS_Client c{host, port, dest_host, dest_port};
    c.run();
    return 0;
}
