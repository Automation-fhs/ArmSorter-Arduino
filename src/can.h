#include <mcp_can.h>
#include <SPI.h>

void canSend(unsigned long id, byte lenght, byte *buf);
void can_init();
void can_read();
