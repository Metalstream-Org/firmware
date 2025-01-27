#include <string.h>
#include "driver/usb_serial_jtag.h"

#include "config.h"
#include "serial_task.h"

void serial_task(void* parameters) // |\label{line:serial_task_implementation}|
{
    auto params = static_cast<SerialTaskParams*>(parameters);

    static char tx_buf[MAX_MESSAGE_LEN*2];
    memset(tx_buf, 0, sizeof(tx_buf));
    static char rx_buf[MAX_MESSAGE_LEN];
    memset(rx_buf, 0, sizeof(rx_buf));

    size_t rd_len = 0;
    static char temp_rx_buf[USB_FIFO_BUF_SIZE];

    while (true)
    {
        // Een while loopt zorgt er voor dat zolang de queue nog items ontvangt, hij blijft ontvangen.
        // Dit wordt gedaan zodat de queue altijd leeg is en hij dus niet kan overlopen.
        while (xQueueReceive(params->tx_queue, tx_buf, pdMS_TO_TICKS(1))) {
            usb_serial_jtag_write_bytes(tx_buf, strlen(tx_buf), 20 / portTICK_PERIOD_MS);
            memset(tx_buf, 0x00, sizeof(tx_buf));
        }

        memset(temp_rx_buf, 0x00, sizeof(temp_rx_buf));
        int len = usb_serial_jtag_read_bytes(temp_rx_buf, sizeof(temp_rx_buf), 20 / portTICK_PERIOD_MS);

        // Write data back to the USB SERIAL JTAG
        if (len > 0) {
            temp_rx_buf[len] = '\0';

            // check if we have enough space in rd_buf
            if ((rd_len + len + 1)  < sizeof (rx_buf)) {
                // copy in new received data
                memcpy(&rx_buf[rd_len], temp_rx_buf, len);
                memset(temp_rx_buf, 0, sizeof(temp_rx_buf));
                rd_len += len;
            } else {
                // discard everything when buffer overflows
                rd_len = 0;
            }

            // make sure string is terminated
            rx_buf[rd_len] = '\0';

            // parse message
            if (rd_len > 0 ) {
                char* start = strchr((const char*)rx_buf, '$');
                
                // remove everything before start
                if (start != nullptr)
                {
                    rd_len = rd_len - ((size_t)start - (size_t)rx_buf);
                    memmove(rx_buf, start, rd_len + 1);

                    // check if we have a complete message
                    char* end = strchr((const char*)rx_buf, '#');
                    
                    if (end != nullptr) {
                        // construct output message (in data), skip for character ($)
                        size_t olen = (size_t)end - (size_t)rx_buf;
                        rx_buf[olen] = '\0';

                        xQueueSend(params->rx_queue, &rx_buf[1], portMAX_DELAY);
                        
                        rd_len = 0;
                        memset(rx_buf, 0x00, sizeof(rx_buf));
                    }
                }
            }
        }
    }
}