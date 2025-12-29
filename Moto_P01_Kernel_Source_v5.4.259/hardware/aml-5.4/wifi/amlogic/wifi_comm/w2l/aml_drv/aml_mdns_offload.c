/**
****************************************************************************************
*
* @file aml_tcp_ack.c
*
* Copyright (C) Amlogic, Inc. All rights reserved (2022-2023).
*
* @brief tcp ack.
*
****************************************************************************************
*/

#include "aml_mdns_offload.h"

static u32_boolean setOffloadState(u32_boolean enabled)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
    return 0;
}

static void resetAll()
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
}

static int addProtocolResponses(char *networkInterface,
    mdnsProtocolData *offloadData)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
    return -1;
}

static void removeProtocolResponses(int recordKey)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
}

static int getAndResetHitCounter(int recordKey)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
    return -1;
}

static int getAndResetMissCounter()
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
    return -1;
}

static u32_boolean addToPassthroughList(char *networkInterface,
    char *qname)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
    return 0;
}

static void removeFromPassthroughList(char *networkInterface,
    char *qname)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
}

static void setPassthroughBehavior(char *networkInterface,
    passthroughBehavior behavior)
{
    AML_PRINT(AML_DBG_MODULES_MDNS, "enter: %s\n", __func__);
}

ANDROID_MDNS_OFFLOAD_VENDOR_IMPL = {
    .setOffloadState = setOffloadState,
    .resetAll = resetAll,
    .addProtocolResponses = addProtocolResponses,
    .removeProtocolResponses = removeProtocolResponses,
    .getAndResetHitCounter = getAndResetHitCounter,
    .getAndResetMissCounter = getAndResetMissCounter,
    .addToPassthroughList = addToPassthroughList,
    .removeFromPassthroughList = removeFromPassthroughList,
    .setPassthroughBehavior = setPassthroughBehavior,
};

