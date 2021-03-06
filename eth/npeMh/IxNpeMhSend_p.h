/**
 * @file IxNpeMhSend_p.h
 *
 * @author Intel Corporation
 * @date 18 Jan 2002
 *
 * @brief This file contains the private API for the Send module.
 *
 * -- Intel Copyright Notice --
 *
 * @par
 * INTEL CONFIDENTIAL
 *
 * @par
 * Copyright 2002 Intel Corporation All Rights Reserved.
 *
 * @par
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or its suppliers or licensors.  Title to the Material remains with
 * Intel Corporation or its suppliers and licensors.  The Material
 * contains trade secrets and proprietary and confidential information of
 * Intel or its suppliers and licensors.  The Material is protected by
 * worldwide copyright and trade secret laws and treaty provisions. No
 * part of the Material may be used, copied, reproduced, modified,
 * published, uploaded, posted, transmitted, distributed, or disclosed in
 * any way without Intel's prior express written permission.
 *
 * @par
 * No license under any patent, copyright, trade secret or other
 * intellectual property right is granted to or conferred upon you by
 * disclosure or delivery of the Materials, either expressly, by
 * implication, inducement, estoppel or otherwise.  Any license under
 * such intellectual property rights must be express and approved by
 * Intel in writing.
 *
 * @par
 * For further details, please see the file README.TXT distributed with
 * this software.
 * -- End Intel Copyright Notice --
*/

/**
 * @defgroup IxNpeMhSend_p IxNpeMhSend_p
 *
 * @brief The private API for the Send module.
 * 
 * @{
 */

#ifndef IXNPEMHSEND_P_H
#define IXNPEMHSEND_P_H

#include "IxNpeMh.h"
#include "IxTypes.h"

/*
 * #defines for function return types, etc.
 */

/*
 * Prototypes for interface functions.
 */

/**
 * @fn IX_STATUS ixNpeMhSendMessageSend (
           IxNpeMhNpeId npeId,
           IxNpeMhMessage message,
           UINT32 maxSendRetries)
 *
 * @brief This function writes a message to the specified NPE's inFIFO,
 * and must be used when the message being sent does not solicit a response
 * from the NPE.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to send the message
 * to.
 * @param IxNpeMhMessage message (in) - The message to send.
 * @param UINT32 maxSendRetries (in) - Max num. of retries to perform
 * if the NPE's inFIFO is full.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhSendMessageSend (
    IxNpeMhNpeId npeId,
    IxNpeMhMessage message,
    UINT32 maxSendRetries);

/**
 * @fn IX_STATUS ixNpeMhSendMessageWithResponseSend (
           IxNpeMhNpeId npeId,
           IxNpeMhMessage message,
           IxNpeMhMessageId solicitedMessageId,
           IxNpeMhCallback solicitedCallback,
           UINT32 maxSendRetries)
 *
 * @brief This function writes a message to the specified NPE's inFIFO,
 * and must be used when the message being sent solicits a response from
 * the NPE.  The ID of the solicited response must be specified so that it
 * can be recognised, and a callback provided to pass the response back to
 * the client.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to send the message
 * to.
 * @param IxNpeMhMessage message (in) - The message to send.
 * @param IxNpeMhMessageId solicitedMessageId (in) - The ID of the
 * solicited response.
 * @param IxNpeMhCallback solicitedCallback (in) - The callback to pass the
 * solicited response back to the client.
 * @param UINT32 maxSendRetries (in) - Max num. of retries to perform
 * if the NPE's inFIFO is full.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhSendMessageWithResponseSend (
    IxNpeMhNpeId npeId,
    IxNpeMhMessage message,
    IxNpeMhMessageId solicitedMessageId,
    IxNpeMhCallback solicitedCallback,
    UINT32 maxSendRetries);

/**
 * @fn void ixNpeMhSendShow (
           IxNpeMhNpeId npeId)
 *
 * @brief This function will display the current state of the Send module.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to display state
 * information for.
 *
 * @return No return value.
 */

void ixNpeMhSendShow (
    IxNpeMhNpeId npeId);

/**
 * @fn void ixNpeMhSendShowReset (
           IxNpeMhNpeId npeId)
 *
 * @brief This function will reset the current state of the Send module.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to reset state
 * information for.
 *
 * @return No return value.
 */

void ixNpeMhSendShowReset (
    IxNpeMhNpeId npeId);

#endif /* IXNPEMHSEND_P_H */

/**
 * @} defgroup IxNpeMhSend_p
 */
