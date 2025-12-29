/*
 ****************************************************************************************
 *
 * Copyright (C) Amlogic 2020-2024
 *
 * Project: file read write
 *
 * Description:
 *     file read write fun
 *
 *
 ****************************************************************************************
 */
#include "wifi_hal_com.h"
#include <linux/namei.h>
#include "wifi_common.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
static int openFile(struct file **fpp, const char *path, int flag, int mode)
{
    struct file *fp;

    fp = filp_open(path, flag, mode);
    if (IS_ERR(fp)) {
        *fpp = NULL;
        return PTR_ERR(fp);
    } else {
        *fpp = fp;
        return 0;
    }
}
#endif

/*
* Close the file with the specific @param fp
* @param fp the pointer of struct file to close
* @return always 0
*/
static int closeFile(struct file *fp)
{
    filp_close(fp, NULL);
    return 0;
}

static int readFile(struct file *fp, char *buf, int len)
{
    int rlen = 0, sum = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
    if (!(fp->f_mode & FMODE_CAN_READ)) {
#else
    if (!fp->f_op || !fp->f_op->read) {
#endif
        return -EPERM;
    }

    while (sum < len) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
        rlen = kernel_read(fp, buf + sum, len - sum, &fp->f_pos);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
        rlen = __vfs_read(fp, buf + sum, len - sum, &fp->f_pos);
#else
        rlen = fp->f_op->read(fp, buf + sum, len - sum, &fp->f_pos);
#endif
#endif
        if (rlen > 0) {
            sum += rlen;
        } else if (0 != rlen) {
            return rlen;
        } else {
            break;
        }
    }

    return sum;

}

static int writeFile(struct file *fp, char *buf, int len)
{
    int wlen = 0, sum = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
    if (!(fp->f_mode & FMODE_CAN_WRITE)) {
#else
    if (!fp->f_op || !fp->f_op->write) {
#endif
        return -EPERM;
    }

    while (sum < len) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
        wlen = kernel_write(fp, buf + sum, len - sum, &fp->f_pos);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
#ifdef UBUNTU_PT_MODE
        wlen = kernel_write(fp, buf + sum, len - sum, &fp->f_pos);
#else
        wlen = vfs_write(fp, buf + sum, len - sum, &fp->f_pos);
#endif

#else
        wlen = fp->f_op->write(fp, buf + sum, len - sum, &fp->f_pos);
#endif
#endif
        if (wlen > 0) {
            sum += wlen;
        } else if (0 != wlen) {
            return wlen;
        } else {
            break;
        }
    }
    return sum;
}

/*
* Test if the specific @param path is a file and readable
* If readable, @param sz is got
* @param path the path of the file to test
* @return Linux specific error code
*/
extern struct device *vm_cfg80211_get_parent_dev(void);
int isFileReadable(const char *path, u32 *sz)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    int ret = 0;
    struct file *fp;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    mm_segment_t oldfs;
#endif
    char buf;

    fp = filp_open(path, O_RDONLY, 0);
    if (IS_ERR(fp)) {
        ret = PTR_ERR(fp);
    } else {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
        oldfs = get_fs();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
        set_fs(KERNEL_DS);
#else
        set_fs(get_ds());
#endif
#endif
        if (1 != readFile(fp, &buf, 1)) {
            ret = PTR_ERR(fp);
        }

        if (ret == 0 && sz) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
            *sz = i_size_read(fp->f_path.dentry->d_inode);
#else
            *sz = i_size_read(fp->f_dentry->d_inode);
#endif
        }
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
        set_fs(oldfs);
#endif
        filp_close(fp, NULL);
    }
    return ret;
#else
    int ret = 0;
    const struct firmware *fw = NULL;
    struct device *dev = vm_cfg80211_get_parent_dev();

    ret = request_firmware(&fw, path, dev);

    if (fw != NULL) {
        release_firmware(fw);
    }

    return ret;
#endif
}

/*
* Open the file with @param path and retrive the file content into memory starting from @param buf for @param sz at most
* @param path the path of the file to open and read
* @param buf the starting address of the buffer to store file content
* @param sz how many bytes to read at most
* @return the byte we've read, or Linux specific error code
*/
static int retrieveFromFile(const char *path, u8 *buf, u32 sz)
{

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
    int ret = -1;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    mm_segment_t oldfs;
#endif
    struct file *fp;

    if (path && buf) {
        ret = openFile(&fp, path, O_RDONLY, 0);
        if (0 == ret) {
            AML_PRINT_LOG_INFO("openFile path:%s fp=%p\n", path , fp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            oldfs = get_fs();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
            set_fs(KERNEL_DS);
#else
            set_fs(get_ds());
#endif
#endif // 5.15
            ret = readFile(fp, buf, sz);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            set_fs(oldfs);
#endif
            closeFile(fp);

            AML_PRINT_LOG_INFO("readFile, ret:%d\n", ret);
        } else {
            AML_PRINT_LOG_ERR("openFile path:%s Fail, ret:%d\n", path, ret);
        }
    } else {
        AML_PRINT_LOG_ERR("NULL pointer\n");
        ret = -EINVAL;
    }
    return ret;
#else
    return 0;
#endif
}

/*
* Open the file with @param path and write @param sz byte of data starting from @param buf into the file
* @param path the path of the file to open and write
* @param buf the starting address of the data to write into file
* @param sz how many bytes to write at most
* @return the byte we've written, or Linux specific error code
*/
static int storeToFile(const char *path, u8 *buf, u32 sz)
{
    int ret = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    mm_segment_t oldfs;
#endif
    struct file *fp;

    if (path && buf) {
        ret = openFile(&fp, path, O_CREAT | O_WRONLY, 0666);
        if (0 == ret) {
            AML_PRINT_LOG_INFO("openFile path:%s fp=%p\n", path , fp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            oldfs = get_fs();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
            set_fs(KERNEL_DS);
#else
            set_fs(get_ds());
#endif
#endif//5.15
            ret = writeFile(fp, buf, sz);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            set_fs(oldfs);
#endif
            closeFile(fp);
            AML_PRINT_LOG_INFO("writeFile, ret:%d\n", ret);
        } else {
            AML_PRINT_LOG_ERR("openFile path:%s Fail, ret:%d\n", path, ret);
        }
    } else {
        AML_PRINT_LOG_ERR("NULL pointer\n");
        ret =  -EINVAL;
    }
#endif
    return ret;
}

/*
* Test if the specific @param path is a file and readable
* @param path the path of the file to test
* @return true or false
*/
int aml_is_file_readable(const char *path)
{
    if (isFileReadable(path, NULL) == 0) {
        return true;
    } else {
        return false;
    }
}

/*
* Test if the specific @param path is a file and readable.
* If readable, @param sz is got
* @param path the path of the file to test
* @return _TRUE or _FALSE
*/
int aml_is_file_readable_with_size(const char *path, u32 *sz)
{
    if (isFileReadable(path, sz) == 0) {
        return true;
    } else {
        return false;
    }
}

/*
* Test if the specific @param path is a readable file with valid size.
* If readable, @param sz is got
* @param path the path of the file to test
* @return _TRUE or _FALSE
*/
int aml_readable_file_sz_chk(const char *path, u32 sz)
{
    u32 fsz;

    if (aml_is_file_readable_with_size(path, &fsz) == false) {
        return false;
    }

    if (fsz > sz) {
        return false;
    }

    return true;
}

/*
* Open the file with @param path and retrive the file content into memory starting from @param buf for @param sz at most
* @param path the path of the file to open and read
* @param buf the starting address of the buffer to store file content
* @param sz how many bytes to read at most
* @return the byte we've read
*/
int aml_retrieve_from_file(const char *path, u8 *buf, u32 sz)
{
    int ret = retrieveFromFile(path, buf, sz);
    return ret >= 0 ? ret : 0;
}

/*
* Open the file with @param path and write @param sz byte of data starting from @param buf into the file
* @param path the path of the file to open and write
* @param buf the starting address of the data to write into file
* @param sz how many bytes to write at most
* @return the byte we've written
*/
int aml_store_to_file(const char *path, u8 *buf, u32 sz)
{
    int ret = storeToFile(path, buf, sz);
    return ret >= 0 ? ret : 0;
}

extern char * capture_path;
bool isFirstWrtFwlog = true;
int storeFwlogToFile(u8 *buf, u32 sz)
{
    int ret = 0;
    unsigned int file_mode;
    ktime_t curTime = ktime_get();
    char fp_path[64] = {'\0'};
    static char time_record_buff[100] = {0};
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)) || defined (LINUX_PLATFORM)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
    mm_segment_t oldfs;
#endif
    struct file *fp;

    uint64_t remain = 0;
    uint64_t div_value_s = 0;
    uint64_t div_value_ms = 0;

    div_value_s = div_u64_rem(curTime, 1000000000, &remain);
    div_value_ms = div64_u64(remain, 1000);

    sprintf(time_record_buff, "\n<%lld.%lld>:\n", div_value_s, div_value_ms); //record system time eg:<xx.xx s>

    if (buf) {
        if (isFirstWrtFwlog) {
            file_mode = O_CREAT | O_WRONLY | O_TRUNC;
            isFirstWrtFwlog = false;
        } else {
            file_mode = O_CREAT | O_WRONLY | O_APPEND;
        }

        sprintf(fp_path,"%s/fw_trace.log", capture_path);

        ret = openFile(&fp, fp_path, file_mode, 0666);
        if (0 == ret) {
            AML_PRINT_LOG_INFO("openFile path:%s fp=%p\n", fp_path, fp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            oldfs = get_fs();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
            set_fs(KERNEL_DS);
#else
            set_fs(get_ds());
#endif
#endif//5.15
            ret += writeFile(fp, time_record_buff, strlen(time_record_buff));
            ret += writeFile(fp, buf, sz);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
            set_fs(oldfs);
#endif
            closeFile(fp);
            AML_PRINT_LOG_INFO("writeFile, ret:%d\n", ret);
        } else {
            AML_PRINT_LOG_ERR("openFile path:%s Fail, ret:%d\n", fp_path, ret);
        }
    } else {
        AML_PRINT_LOG_ERR("NULL pointer\n");
        ret =  -EINVAL;
    }
#endif
    return ret;
}

/**
* IsHexDigit -
*
* Return TRUE if chTmp is represent for hex digit
* FALSE otherwise.
*/
bool aml_char_is_hex_digit(char chTmp)
{
    if ((chTmp >= '0' && chTmp <= '9') ||
        (chTmp >= 'a' && chTmp <= 'f') ||
        (chTmp >= 'A' && chTmp <= 'F')) {
        return true;
    } else {
        return false;
    }
}


u32 aml_read_macaddr_from_file(const char *path, u8 *buf)
{
    u32 i;
    u8 temp[3];
    u32 ret = false;

    u8 file_data[FILE_DATA_LEN];
    u32 read_size;
    u8 addr[ETH_ALEN];

    read_size = aml_retrieve_from_file(path, file_data, FILE_DATA_LEN);
    if (read_size != FILE_DATA_LEN) {
        AML_PRINT_LOG_INFO("read from %s fail\n", path);
        goto exit;
    }

    temp[2] = 0; /* end of string '\0' */

    for (i = 0 ; i < ETH_ALEN ; i++) {
        if (aml_char_is_hex_digit(file_data[PARSE_DIGIT_BASE + i * 3]) == false
            || aml_char_is_hex_digit(file_data[PARSE_DIGIT_BASE + i * 3 + 1]) == false) {
            AML_PRINT_LOG_INFO("invalid 8-bit hex format for address offset:%u\n", i);
            goto exit;
        }

        if (i < ETH_ALEN - 1 && file_data[PARSE_DIGIT_BASE + i * 3 + 2] != ':') {
            AML_PRINT_LOG_INFO("invalid separator after address offset:%u\n", i);
            goto exit;
        }

        temp[0] = file_data[PARSE_DIGIT_BASE + i * 3];
        temp[1] = file_data[PARSE_DIGIT_BASE + i * 3 + 1];
        if (sscanf(temp, "%hhx", &addr[i]) != 1) {
            AML_PRINT_LOG_INFO("sscanf fail for address offset:0x%03x\n", i);
            goto exit;
        }
    }
    memset(buf, '\0', ETH_ALEN);
    memcpy(buf, addr, ETH_ALEN);

    ret = true;

exit:
    return ret;
}
