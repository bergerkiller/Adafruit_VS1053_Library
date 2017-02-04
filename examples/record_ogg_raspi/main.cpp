#include "Adafruit_VS1053.h"
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <vector>
#include <signal.h>

// These are the pins used for the breakout example
#define VS1053_RESET  22     // VS1053 reset pin (output)
#define VS1053_CS     25     // VS1053 chip select (output)
#define VS1053_DCS    27     // VS1053 Data/command select pin (output)
#define VS1053_DREQ   4      // VS1053 Data request, ideally an Interrupt pin

#define MODE_DEFAULT  0
#define MODE_OUTPUT   1
#define MODE_STREAM   2

struct OggPacket {
  OggPacket() : size(0), data(NULL), next(NULL) {}
  ~OggPacket() { delete data; data = NULL; }

  uint32_t size;
  uint8_t *data;
  OggPacket *next;
};

pthread_t accept_thread;
volatile int listen_fd = -1;
volatile int accept_fd = -1;
volatile bool exiting = false;
OggPacket *backBuffer = new OggPacket();

void intHandler(int dummy) {
  exiting = true;
}

bool writePacket(uint8_t* data, uint32_t size) {
  if (accept_fd == -1)
    return false;
    
  uint32_t written_size = 0;
  while (written_size < size) {
    ssize_t written_cnt = write(accept_fd, (const void*) (data + written_size),
                                size - written_size);
    if (written_cnt < 0) {
      exiting = true;
      break;
    }
    written_size += written_cnt;
  }
  return true;
}

bool sendNextPacket(bool forceIt = false) {
  if ((backBuffer == NULL) || (!forceIt && (backBuffer->next == NULL)))
    return false;

  OggPacket *packet = backBuffer;
  if (!writePacket(packet->data, packet->size))
    return false;

  backBuffer = backBuffer->next;
  delete packet;
  return true;
}

void* server_accept_thread_run(void *arg) {
  struct sockaddr_in serv_addr;
  int option = 1;
  listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd == -1) {
    std::cerr << "Failed to create socket." << std::endl;
    exiting = true;
    return NULL;
  }
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
  memset(&serv_addr, '0', sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(3000); 
  if (bind(listen_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) {
    std::cerr << "Bind failed." << std::endl;
    exiting = true;
    close(listen_fd);
    return NULL;
  }
  if (listen(listen_fd, 1) == -1) {
    std::cerr << "Listen failed." << std::endl;
    exiting = true;
    close(listen_fd);
    return NULL;
  }
  while (1) {
    int connfd = accept(listen_fd, (struct sockaddr*)NULL, NULL); 
    if (connfd != -1) {
      accept_fd = connfd;
      std::cerr << "Connected to receiver!" << std::endl;
      break;
    }
  }
  return NULL;
}

int main(int argc, char **argv) {
  int runningMode = MODE_DEFAULT;
  if (argc == 2) {
    char* mode = argv[1];
    if (!strcmp(mode, "-")) {
      runningMode = MODE_OUTPUT;
    } else if (!strcmp(mode, "stream")) {
      runningMode = MODE_STREAM;
    }
  }
  std::cerr << "RUN: " << runningMode << std::endl;

  // Handle forceful exit gracefully
  signal(SIGINT, intHandler);
  
  // Start a server instance right away listening locally on port 3000
  if (runningMode == MODE_DEFAULT || runningMode == MODE_STREAM) {
    pthread_create(&accept_thread, NULL, &server_accept_thread_run, NULL);
  }

  if (runningMode == MODE_STREAM) {
    // We need binary mode!
    freopen(NULL, "rb", stdin);
    
    // Read data from stdin and stream it over tcp
    OggPacket *currPacket = backBuffer;
    uint8_t readBuffer[2000];
    while (!exiting) {
      std::cin.read((char*) readBuffer, sizeof(readBuffer));
      std::streamsize readSize = std::cin.gcount();
      if (readSize < 0) {
        exiting = true;
      } else {
          // Fill current packet with the data received and add a new one to the chain
          currPacket->size = readSize;
          currPacket->data = new uint8_t[readSize];
          memcpy(currPacket->data, readBuffer, readSize);
          currPacket->next = new OggPacket();
          //std::cerr << "Store: " << currPacket->size << std::endl;
          currPacket = currPacket->next;
          
          // If connected, send the first one in the chain and free its memory
          if (accept_fd != -1) {
            sendNextPacket();
          }
      }
    }
    
  } else {
    // Read audio from device and either stream it (default) or write to stdout
  
    // Meanwhile, initialize the VS1053 chip and start recording
    wiringPiSetupGpio(); // Use GPIO port numbering scheme
    Adafruit_VS1053_FilePlayer musicPlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, 0);
    if (musicPlayer.begin()) {
      if (musicPlayer.prepareRecordOgg("/home/pi/vs1053b/v44k1q05.img")) {
        // Start recording
        // true = microphone; false = linein
        std::cerr << "Starting recording." << std::endl;
        delay(200);
        musicPlayer.startRecordOgg(true);
        std::cerr << "Begin reading recorded data." << std::endl;
      } else {
        std::cerr << "Couldn't load plugin!" << std::endl;
        exiting = true;
      }
    } else {
      std::cerr << "VS1053 not found" << std::endl;
      exiting = true;
    }

    // Write data packets into a backbuffer until a connection with a listener is received
    OggPacket *currPacket = backBuffer;
    uint8_t readBuffer[100000];
    uint32_t readSize;
    uint32_t send_ctr = 0;
    while (!exiting) {
      // Read new data from chip into a buffer
      // We do not expect more than 10KB of data at a time
      readSize = musicPlayer.recordedReadData(readBuffer, sizeof(readBuffer));
      if (readSize > 0) {
        if (runningMode == MODE_OUTPUT) {
          // Directly write the packet data to stdout
          std::cout.write((const char*) readBuffer, readSize);
        
        } else if (backBuffer != NULL) {
          // Fill current packet with the data received and add a new one to the chain
          currPacket->size = readSize;
          currPacket->data = new uint8_t[readSize];
          memcpy(currPacket->data, readBuffer, readSize);
          currPacket->next = new OggPacket();
          //std::cerr << "Store: " << currPacket->size << std::endl;
          currPacket = currPacket->next;
          
          // If connected, send the first one in the chain and free its memory
          if (accept_fd != -1) {
            //sendNextPacket();
            send_ctr++;
          }
        } else {
          // No more data in back buffer, directly write it out to the socket
          writePacket(readBuffer, readSize);
        }
      } else if (!sendNextPacket(true)) {
        usleep(20000);
      }
      
      //} else if (send_ctr > 0 && sendNextPacket()) {
      //  send_ctr--;
      //} else {
      //  delay(20);
      //}
    }
  }
  
  if (accept_fd != -1)
    close(accept_fd);
  if (listen_fd != -1)
    close(listen_fd);
  return 0;
}