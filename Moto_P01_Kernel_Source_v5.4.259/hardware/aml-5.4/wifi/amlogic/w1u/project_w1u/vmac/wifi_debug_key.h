#ifndef _AML_DEBUG_KEY_H_
#define _AML_DEBUG_KEY_H_


#define CMD_ADDRESS_MASK           (0xfff)
#define CMD_TYPE_READ              (0x0)
#define CMD_TYPE_WRITE             (0x1)

#define UNICAST_KEY 1
#define MULTICAST_KEY 0
#define MULTICAST_KEY_ENTRY_WORD 9
#define MULTICAST_KEY_ENTRY_NUM 4
#define UNICAST_KEY_ENTRY_WORD 10
#define UNICAST_KEY_ENTRY_NUM 64
#define BYTES_NUM_PER_LINE 12

#define CLEAR_DATA  (BIT(0))
#define CLEAR_DATA2 (BIT(1))
#define CLEAR_DATA3 (BIT(2))
void  wifi_mac_key_ctrl_read_key_table_by_condition(unsigned char vid, unsigned char key_id, unsigned char is_ukey, unsigned char *mac);
#endif
