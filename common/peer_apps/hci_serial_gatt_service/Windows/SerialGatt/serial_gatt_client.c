#include <Windows.h>
#include "../../../../include/wiced_bt_serial_gatt.h"
typedef void(serial_client_cback_t)(unsigned long arg);

extern void ProcessedReceivedData(unsigned char* p_data, unsigned short len);
extern void StartTimer(unsigned short duration, serial_client_cback_t* p_timer_callback);
extern void StopTimer(void);
extern void RegisterForNotifications(void);
extern void SendSerialClientData(unsigned char* p_data, unsigned short len);
extern void SerilaGattTxComplete(unsigned char result);

void SetMtu(unsigned short);
/*******************************************************************
* Structures
******************************************************************/
// Application state control block
typedef struct
{
    unsigned char   tx_credits;             // number of credits we have to send OTA data
    unsigned char   rx_credits_max;         // maximum number of credits to give to peer
    unsigned char   rx_credits;             // current number of credits to give to peer
    unsigned char   send_mtu_size;          // should always be false on the client
    unsigned short  peer_mtu;               // negotiated MTU
    unsigned short  conn_id;                // connection ID to send data
    unsigned short  tx_len;                 // Length of the data in the TX buffer
    unsigned short  tx_offset;              // Offset of the data in the TX buffer
    unsigned char*  p_tx_buffer;            // buffer currently being transmitted
    unsigned long   bytes_rx;               // bytes received
    unsigned long   bytes_tx;               // bytes transmitted
} serial_gatt_state_t;

/*******************************************************************
* Variable Definitions
******************************************************************/
serial_gatt_state_t serial_gatt_state;

//extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
//extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
extern ods(char *p_msg);

/*******************************************************************
* Function Prototypes
******************************************************************/
static void          serial_gatt_client_timer_callback(unsigned long arg);
static void          serial_gatt_client_fine_timer_callback(unsigned long arg);

int                  serial_gatt_client_process_forward_data(unsigned char *p_buffer, unsigned short offset, unsigned short length);
static int           serial_gatt_client_send_data(int return_credits);
static void          serial_gatt_client_process_mtu(unsigned short mtu);

/*******************************************************************
* Function Definitions
******************************************************************/
/*
* This function is executed in the BTM_ENABLED_EVT management callback.
*/
void serial_gatt_client_app_init(void)
{
    memset(&serial_gatt_state, 0, sizeof(serial_gatt_state));
}

/*
 * Ack timeout
 */
void serial_gatt_client_ack_timer_callback(unsigned long arg)
{
}

/*
* Handles Notifications received from Server device
*/
void serial_gatt_client_process_notification(unsigned short conn_id, unsigned char *p_data, unsigned short len)
{
    unsigned char   flags;
    unsigned char   credit = 0;
    unsigned short  mtu = 0;

    // parse data received from the peer
    if (len < 2)
    {
        ods("illegal data len\n");
        return;
    }

    flags = *p_data++;
    len--;

    // If Credit Field is present, it should be the first field
    if (flags & SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT)
    {
        credit = *p_data++;
        len--;
    }
    // Next 2 bytes is MTU if flags indicates that the field is present
    if (flags & SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT)
    {
        if (len < 2)
        {
            ods("illegal mtu len\n");
            return;
        }
        mtu = p_data[0] + (p_data[1] << 8);
        p_data += 2;
        len -= 2;
    }
    // Next comes data if flags indicates that the field is present
    if (flags & SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT)
    {
        if (len < 1)
        {
            ods("illegal data len\n");
            return;
        }
    }


    // if we received data send it to the app
    if (len > 0)
    {
        serial_gatt_state.bytes_rx += len;

        ProcessedReceivedData(p_data, len);

        // increase number of credits we need to deliver to the peer.  serial_gatt_client_send_data
        // will send credits if needed.
        serial_gatt_state.rx_credits++;
    }

    // process number of credits if received from the peer
    if (credit + serial_gatt_state.tx_credits > SERIAL_GATT_MAX_CREDITS)
    {
        ods("illegal credits\n");
        return;
    }

    // if received MTU in the data packet and we did not know it before, set it up now.
    if ((mtu != 0) && (serial_gatt_state.peer_mtu == 0))
    {
        serial_gatt_client_process_mtu(mtu);
    }

    // tx_credits indicates how many packets we can send to the peer.
    serial_gatt_state.tx_credits += credit;

    // as we got more credits, we might be able to send data out, or may need to send credits.
    serial_gatt_client_send_data(FALSE);

    // if ack timer is not running, start one now.
    StartTimer(100, serial_gatt_client_fine_timer_callback);
}

void SetMtu(unsigned short mtu)
{
    serial_gatt_state.peer_mtu = mtu;
}

/*
 * Connection Up processing.
 */
void serial_gatt_client_connection_up()
{
    RegisterForNotifications();

    serial_gatt_state.tx_credits = 0;

    //serial_gatt_state.peer_mtu = (byte)23;
    if ((byte)(1024 / serial_gatt_state.peer_mtu) < 20)
        serial_gatt_state.rx_credits_max = (byte)(1024 / serial_gatt_state.peer_mtu);
    else
        serial_gatt_state.rx_credits_max = 20;

    serial_gatt_state.rx_credits = serial_gatt_state.rx_credits_max;

    serial_gatt_client_send_data(1);
}

void serial_gatt_client_process_mtu(unsigned short mtu)
{
    serial_gatt_state.peer_mtu = mtu;
    serial_gatt_state.rx_credits_max = (1024 / serial_gatt_state.peer_mtu) < 20 ? (1024 / serial_gatt_state.peer_mtu) : 20;
    serial_gatt_state.rx_credits = serial_gatt_state.rx_credits_max;
    serial_gatt_client_send_data(FALSE);
}

/*
* Connection Down processing
*/
void serial_gatt_client_connection_down()
{
    // ToDo free tx buffer
    serial_gatt_state.p_tx_buffer = 0;

    StopTimer();
}

/*
* Handle forward data packet
*/
int serial_gatt_client_process_forward_data(unsigned char *p_buffer, unsigned short offset, unsigned short length)
{
    if (serial_gatt_state.p_tx_buffer != 0)
    {
        ods("error fwd data with buffer present\n");
        return 1;
    }

    if(!p_buffer || !length)
    {
        ods("Invalid buffer or length\n");
        return 1;
    }

    serial_gatt_state.p_tx_buffer = p_buffer;
    serial_gatt_state.tx_offset = offset;
    serial_gatt_state.tx_len = length;

    return (serial_gatt_client_send_data(FALSE));
}

/*
 * Calculate header size depending if we need to return credits or send mtu_size
 */
unsigned short serial_gatt_get_header_size()
{
    // Minimum header size is 4 which is ATT header size for notifications and write command and flags field
    unsigned short header_size = 4;

    if (serial_gatt_state.rx_credits != 0)
    {
        header_size += 1;
    }
    if (serial_gatt_state.send_mtu_size)
    {
        header_size += 2;
    }
    return header_size;
}

/*
 * Format and send data over the GATT transport
 */
void serial_gatt_format_and_send_data(unsigned short data_len)
{
    unsigned char buffer[4];
    unsigned char *p_buffer;
    unsigned char *p_data;

    if (data_len == 0)
    {
        p_data = buffer;
    }
    else
    {
        // incoming data should have at least 4 bytes of offset
        // first byte is always for credits
        p_data = serial_gatt_state.p_tx_buffer + serial_gatt_state.tx_offset;

        p_data--;
        if (serial_gatt_state.rx_credits != 0)
        {
            p_data--;
        }
        if (serial_gatt_state.send_mtu_size != 0)
        {
            p_data -= 2;
        }
    }
    p_buffer = p_data;

    *p_data++ = ((serial_gatt_state.rx_credits != 0) ? SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT : 0) |
        ((serial_gatt_state.send_mtu_size != 0) ? SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT : 0) |
        ((data_len != 0) ? SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT : 0);
    data_len++;

    if (serial_gatt_state.rx_credits != 0)
    {
        *p_data++ = serial_gatt_state.rx_credits;
        data_len++;

        // because we gave all credits we had to the peer, we do not need to give anymore
        serial_gatt_state.rx_credits = 0;
    }

    if (serial_gatt_state.send_mtu_size != 0)
    {
        *p_data++ = serial_gatt_state.peer_mtu & 0xff;
        *p_data++ = (serial_gatt_state.peer_mtu >> 8) & 0xff;
        data_len += 2;

        // We send MTU once per connection
        serial_gatt_state.send_mtu_size = 0;
    }

    SendSerialClientData(p_buffer, data_len);
}

/*
 * if we have credits and there is a tx buffer, send it now.  Return TRUE if buffer was sent out.
 * Parameter return_credits is set to TRUE if it is time to send credits to peer
 */
int serial_gatt_client_send_data(int return_credits)
{
    // if tx buffer is empty, we may still need to send credits and/or MTU.
   // We also always send credits if we owe more than half credits initially given to us
    if (serial_gatt_state.p_tx_buffer == NULL)
    {
        if (serial_gatt_state.send_mtu_size ||
            ((serial_gatt_state.rx_credits != 0) &&
            (return_credits || (serial_gatt_state.rx_credits >= (serial_gatt_state.rx_credits_max / 2)))))
        {
            serial_gatt_format_and_send_data(0);
        }
        return 0;
    }

    while ((serial_gatt_state.tx_credits > 0) && (serial_gatt_state.p_tx_buffer != NULL))
    {
        // send up to peer MTU number of bytes
        unsigned short header_size = serial_gatt_get_header_size();
        unsigned short bytes_to_send = serial_gatt_state.tx_len < serial_gatt_state.peer_mtu - header_size ? serial_gatt_state.tx_len : serial_gatt_state.peer_mtu - header_size;

        serial_gatt_format_and_send_data(bytes_to_send);

        // as we sent one packet with data we have one less tx credit
        serial_gatt_state.tx_credits--;

        serial_gatt_state.tx_len -= bytes_to_send;
        serial_gatt_state.tx_offset += bytes_to_send;

        // if we are done with this buffer, send notification to the MCU and release the buffer
        if (serial_gatt_state.tx_len == 0)
        {
            serial_gatt_state.p_tx_buffer = 0;
            SerilaGattTxComplete(0);
        }
    }
    return 0;
}

// ack timer expired send data
void serial_gatt_client_fine_timer_callback(unsigned long count)
{
    serial_gatt_client_send_data(TRUE);
}
