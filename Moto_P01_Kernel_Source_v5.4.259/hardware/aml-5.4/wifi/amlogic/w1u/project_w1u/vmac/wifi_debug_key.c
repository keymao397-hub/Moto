#include "wifi_mac_main_reg.h"
#include "wifi_mac_if.h"
#include "wifi_debug.h"
#include "wifi_debug_key.h"

unsigned int  key_table_array[30] = {0};

/*
as the key table one world is 67 bit key_ctrl_read_word function will read 3 world once
@key_table_offset: multicast and unicast position of offset
@position:the buffer position to start write
@buf:the storage buf
*/
int wifi_mac_key_ctrl_get_hash_entry(unsigned char *mac)
{
    if (!mac) {
        AML_PRINT_LOG_ERR("mac addr err\n");
        return 0;
    }
    return (mac[5]^mac[4]^mac[3])&0x3f;
}


void wifi_mac_key_ctrl_read_word(unsigned short key_table_offset, unsigned char position, void *buf)
{

    unsigned int loop = 0;
    MAC_KEYTABLE_CMD_FIELD_T key_cmd;
    unsigned int *buffer = buf;
    //set cmd
    key_cmd.b.reg_keytable_cmd_addr = key_table_offset & CMD_ADDRESS_MASK;
    key_cmd.b.reg_keytable_cmd_type = CMD_TYPE_READ;
    key_cmd.b.reg_keytable_cmd_en = 1;

    wifi_mac_write_word(MAC_KEYTABLE_CMD, key_cmd.data);

    //wait ready
    do
    {
        key_cmd.data = wifi_mac_read_word(MAC_KEYTABLE_CMD);
        loop++;
    }
    while ((key_cmd.b.reg_keytable_cmd_ready == 0) && (loop < 10000));

    if (key_cmd.b.reg_keytable_cmd_ready == 0)
    {
        AML_PRINT_LOG_ERR("cmd not ready\n");
    }

    if (key_cmd.b.ro_keytable_cmd_fail != 0)
    {
        AML_PRINT_LOG_ERR("keycmd fail %d\n", key_cmd.b.ro_keytable_cmd_fail);
    }

    buffer[position] = wifi_mac_read_word(MAC_KEYTABLE_DATA);
    buffer[position + 1] = wifi_mac_read_word(MAC_KEYTABLE_DATA2);
    buffer[position + 2] = wifi_mac_read_word(MAC_KEYTABLE_DATA3);

}

void  wifi_mac_key_ctrl_clear_word(unsigned short key_table_offset, unsigned char clear)
{
    unsigned int loop = 0;
    MAC_KEYTABLE_CMD_FIELD_T key_cmd;

    if (clear & CLEAR_DATA)
    {
        wifi_mac_write_word(MAC_KEYTABLE_DATA, 0);
    }

    if (clear & CLEAR_DATA2)
    {
        wifi_mac_write_word(MAC_KEYTABLE_DATA2, 0);
    }

    if (clear & CLEAR_DATA3)
    {
        wifi_mac_write_word(MAC_KEYTABLE_DATA3, 0);
    }

    //set cmd
    key_cmd.b.reg_keytable_cmd_addr = key_table_offset & CMD_ADDRESS_MASK;
    key_cmd.b.reg_keytable_cmd_type = CMD_TYPE_WRITE;
    key_cmd.b.reg_keytable_cmd_en = 1;

    wifi_mac_write_word(MAC_KEYTABLE_CMD, key_cmd.data);

    //wait ready
    do
    {
        key_cmd.data = wifi_mac_read_word(MAC_KEYTABLE_CMD);
        loop++;
    }
    while ((key_cmd.b.reg_keytable_cmd_ready == 0) && (loop < 10000));


    if (key_cmd.b.reg_keytable_cmd_ready == 0)
    {
        AML_PRINT_LOG_ERR("cmd not ready\n");
    }

    if (key_cmd.b.ro_keytable_cmd_fail != 0)
    {
        AML_PRINT_LOG_ERR("keycmd fail %d\n", key_cmd.b.ro_keytable_cmd_fail);
    }

}

void  wifi_mac_key_ctrl_clear_hash_table(unsigned char *mac)
{
    unsigned int hash_val = 0;
    unsigned int hash_offset = 0;

    hash_val = wifi_mac_key_ctrl_get_hash_entry(mac);
    hash_offset = hash_val * UNICAST_KEY_ENTRY_WORD + MULTICAST_KEY_ENTRY_NUM * MULTICAST_KEY_ENTRY_WORD;

    wifi_mac_key_ctrl_clear_word(hash_offset,CLEAR_DATA3);
}

/*the func only read unicast key entry and hash info*/
void  wifi_mac_key_ctrl_read_key_table_by_condition(unsigned char vid, unsigned char key_id, unsigned char is_ukey, unsigned char *mac)
{
    unsigned short key_table_offset = 0;
    unsigned char hash_val = 0;
    unsigned char i = 0;
    MAC_KEYMAP_FIELD_T key_map;

    memset(key_table_array, 0, sizeof(key_table_array));
    if (!is_ukey)
    {
        key_table_offset = vid * MULTICAST_KEY_ENTRY_WORD;
        for (i = 0; i < MULTICAST_KEY_ENTRY_WORD; i++)
        {
            wifi_mac_key_ctrl_read_word(key_table_offset, i * 3, key_table_array);
            key_table_offset++;
        }
        wifi_debug_dump_data((unsigned char *)key_table_array, MULTICAST_KEY_ENTRY_WORD * 3 * 4, BYTES_NUM_PER_LINE);
    }
    else
    {
        //read hash which indicate key entry index
        hash_val = wifi_mac_key_ctrl_get_hash_entry(mac);
        key_table_offset = hash_val * UNICAST_KEY_ENTRY_WORD + MULTICAST_KEY_ENTRY_NUM * MULTICAST_KEY_ENTRY_WORD;
        wifi_mac_key_ctrl_read_word(key_table_offset, 0, key_table_array);
        AML_PRINT_LOG_INFO("hash:%d 12bytes\n", hash_val);
        wifi_debug_dump_data((unsigned char *)key_table_array, 3 * 4, BYTES_NUM_PER_LINE);

        //read key entry
        memset(key_table_array, 0, sizeof(key_table_array));
        key_map.data = wifi_mac_read_word(MAC_KEYMAP);
        key_table_offset = (vid * key_map.b.reg_keytable_offset1 + key_id) * UNICAST_KEY_ENTRY_WORD + MULTICAST_KEY_ENTRY_NUM * MULTICAST_KEY_ENTRY_WORD;
        for (i = 0; i < UNICAST_KEY_ENTRY_WORD; i++)
        {
            wifi_mac_key_ctrl_read_word(key_table_offset, i * 3, key_table_array);
            key_table_offset++;
        }
        AML_PRINT_LOG_INFO("key_index_entry:%d\n", key_id);
        wifi_debug_dump_data((unsigned char *)key_table_array, UNICAST_KEY_ENTRY_WORD * 3 * 4, BYTES_NUM_PER_LINE);

    }

}

