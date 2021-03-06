/**
 * @file IxNpeMh.h
 *
 * @date 14 Dec 2001
 *
 * @brief This file contains the public API for the IXP4xx NPE Message
 * Handler component.
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
 * @defgroup IxNpeMh IXP4xx NPE Message Handler (IxNpeMh) API
 *
 * @brief The public API for the IXP4xx NPE Message Handler component.
 * 
 * @{
 */

#ifndef IXNPEMH_H
#define IXNPEMH_H

#include "IxTypes.h"

/*
 * #defines for function return types, etc.
 */

#define IX_NPEMH_MIN_MESSAGE_ID (0x00) /**< minimum valid message ID */
#define IX_NPEMH_MAX_MESSAGE_ID (0xFF) /**< maximum valid message ID */

#define IX_NPEMH_SEND_RETRIES_DEFAULT (3) /**< default msg send retries */

/**
 * @enum IxNpeMhNpeId
 *
 * @brief The ID of a particular NPE.
 */

typedef enum
{
    IX_NPEMH_NPEID_NPEA = 0, /**< ID for NPE-A */
    IX_NPEMH_NPEID_NPEB,     /**< ID for NPE-B */
    IX_NPEMH_NPEID_NPEC,     /**< ID for NPE-C */
    IX_NPEMH_NUM_NPES        /**< Number of NPEs */
} IxNpeMhNpeId;

/**
 * @enum IxNpeMhNpeInterrupts
 *
 * @brief Indicator specifying whether or not NPE interrupts should drive
 * receiving of messages from the NPEs.
 */

typedef enum
{
    IX_NPEMH_NPEINTERRUPTS_NO = 0, /**< Don't use NPE interrupts */
    IX_NPEMH_NPEINTERRUPTS_YES     /**< Do use NPE interrupts */
} IxNpeMhNpeInterrupts;

/**
 * @brief The 2-word message structure to send to and receive from the
 * NPEs.
 */

typedef struct
{
    UINT32 data[2]; /**< the actual data of the message */
} IxNpeMhMessage;

/** message ID */
typedef UINT32 IxNpeMhMessageId;

/**
 * @typedef IxNpeMhCallback
 *
 * @brief This prototype shows the format of a message callback function.
 *
 * This prototype shows the format of a message callback function.  The
 * message callback will be passed the message to be handled and will also
 * be told from which NPE the message was received.  The message callback
 * will either be registered by ixNpeMhUnsolicitedCallbackRegister() or
 * passed as a parameter to ixNpeMhMessageWithResponseSend().  It will be
 * called from within an ISR triggered by the NPE's "outFIFO not empty"
 * interrupt (see ixNpeMhInitialize()).  The parameters passed are the ID
 * of the NPE that the message was received from, and the message to be
 * handled.<P><B>Re-entrancy:</B> This function is only a prototype, and
 * will be implemented by the client.  It does not need to be re-entrant.
 */

typedef void (*IxNpeMhCallback) (IxNpeMhNpeId, IxNpeMhMessage);

/*
 * Prototypes for interface functions.
 */

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhInitialize (
           IxNpeMhNpeInterrupts npeInterrupts)
 *
 * @brief This function will initialise the IxNpeMh component.
 *
 * @param IxNpeMhNpeInterrupts npeInterrupts (in) - This parameter
 * dictates whether or not the IxNpeMh component will service NPE "outFIFO
 * not empty" interrupts to trigger receiving and processing of messages
 * from the NPEs.  If not then the client must use ixNpeMhMessagesReceive()
 * to control message receiving and processing.
 *
 * This function will initialise the IxNpeMh component.  It should only be
 * called once, prior to using the IxNpeMh component.  The following
 * actions will be performed by this function:<OL><LI>Initialization of
 * internal data structures (e.g. solicited and unsolicited callback
 * tables).</LI><LI>Configuration of the interface with the NPEs (e.g.
 * enabling of NPE "outFIFO not empty" interrupts).</LI><LI>Registration of
 * ISRs that will receive and handle messages when the NPEs' "outFIFO not
 * empty" interrupts fire (if npeInterrupts equals
 * IX_NPEMH_NPEINTERRUPTS_YES).</LI></OL>
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhInitialize (
    IxNpeMhNpeInterrupts npeInterrupts);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhUnsolicitedCallbackRegister (
           IxNpeMhNpeId npeId,
           IxNpeMhMessageId messageId,
           IxNpeMhCallback unsolicitedCallback)
 *
 * @brief This function will register an unsolicited callback for a
 * particular NPE and message ID.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE whose messages
 * the unsolicited callback will handle.
 * @param IxNpeMhMessageId messageId (in) - The ID of the messages the
 * unsolicited callback will handle.
 * @param IxNpeMhCallback unsolicitedCallback (in) - The unsolicited
 * callback function.  A value of NULL will deregister any previously
 * registered callback for this NPE and message ID.
 *
 * This function will register an unsolicited message callback for a
 * particular NPE and message ID.<P>If an unsolicited callback is already
 * registered for the specified NPE and message ID then the callback will
 * be overwritten.  Only one client will be responsible for handling a
 * particular message ID associated with a NPE.  Registering a NULL
 * unsolicited callback will deregister any previously registered
 * callback.<P>The callback function will be called from an ISR that will
 * be triggered by the NPE's "outFIFO not empty" interrupt (see
 * ixNpeMhInitialize()) to handle any unsolicited messages of the specific
 * message ID received from the NPE.  Unsolicited messages will be handled
 * in the order they are received.<P>If no unsolicited callback can be
 * found for a received message then it is assumed that the message is
 * solicited.<P>If more than one client may be interested in a particular
 * unsolicited message then the suggested strategy is to register a
 * callback for the message that can itself distribute the message to
 * multiple clients as necessary.<P>See also
 * ixNpeMhUnsolicitedCallbackForRangeRegister().<P><B>Re-entrancy:</B> This
 * function will be callable from any thread at any time.  IxOsServices
 * will be used for any necessary resource protection.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhUnsolicitedCallbackRegister (
    IxNpeMhNpeId npeId,
    IxNpeMhMessageId messageId,
    IxNpeMhCallback unsolicitedCallback);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhUnsolicitedCallbackForRangeRegister (
           IxNpeMhNpeId npeId,
           IxNpeMhMessageId minMessageId,
           IxNpeMhMessageId maxMessageId,
           IxNpeMhCallback unsolicitedCallback)
 *
 * @brief This function will register an unsolicited callback for a
 * particular NPE and range of message IDs.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE whose messages the
 * unsolicited callback will handle.
 * @param IxNpeMhMessageId minMessageId (in) - The minimum message ID in
 * the range of message IDs the unsolicited callback will handle.
 * @param IxNpeMhMessageId maxMessageId (in) - The maximum message ID in
 * the range of message IDs the unsolicited callback will handle.
 * @param IxNpeMhCallback unsolicitedCallback (in) - The unsolicited
 * callback function.  A value of NULL will deregister any previously
 * registered callback(s) for this NPE and range of message IDs.
 *
 * This function will register an unsolicited callback for a particular NPE
 * and range of message IDs.  It is a convenience function that is
 * effectively the same as calling ixNpeMhUnsolicitedCallbackRegister() for
 * each ID in the specified range.  See
 * ixNpeMhUnsolicitedCallbackRegister() for more
 * information.<P><B>Re-entrancy:</B> This function will be callable from
 * any thread at any time.  IxOsServices will be used for any necessary
 * resource protection.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhUnsolicitedCallbackForRangeRegister (
    IxNpeMhNpeId npeId,
    IxNpeMhMessageId minMessageId,
    IxNpeMhMessageId maxMessageId,
    IxNpeMhCallback unsolicitedCallback);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhMessageSend (
           IxNpeMhNpeId npeId,
           IxNpeMhMessage message,
           UINT32 maxSendRetries)
 *
 * @brief This function will send a message to a particular NPE.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to send the message
 * to.
 * @param IxNpeMhMessage message (in) - The message to send.
 * @param UINT32 maxSendRetries (in) - Max num. of retries to perform
 * if the NPE's inFIFO is full.
 *
 * This function will send a message to a particular NPE.  It will be the
 * client's responsibility to ensure that the message is properly formed.
 * The return status will signify to the client if the message was
 * successfully sent or not.<P>If the message is sent to the NPE then this
 * function will return a status of success.  Note that this will only mean
 * the message has been placed in the NPE's inFIFO.  There will be no way
 * of knowing that the NPE has actually read the message, but once in the
 * incoming message queue it will be safe to assume that the NPE will
 * process it.
 * <P>The inFIFO may fill up sometimes if the Xscale is sending messages
 * faster than the NPE can handle them. This forces us to retry attempts 
 * to send the message until the NPE services the inFIFO. The client should
 * specify a ceiling value for the number of retries suitable to their
 * needs. IX_NPEMH_SEND_RETRIES_DEFAULT can be used as a default value for
 * the <i>maxSendRetries</i> parameter for this function. Each retry
 * exceeding this default number will incur a blocking delay of 1 microsecond,
 * to avoid consuming too much AHB bus bandwidth while performing retries.
 * <P>Note this function <B>must</B> only be used for messages.
 * that do not solicit responses. If the message being sent will solicit a
 * response then the ixNpeMhMessageWithResponseSend() function <B>must</B>
 * be used to ensure that the response is correctly
 * handled.<P><B>Re-entrancy:</B> This function will be callable from any
 * thread at any time.  IxOsServices will be used for any necessary
 * resource protection.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhMessageSend (
    IxNpeMhNpeId npeId,
    IxNpeMhMessage message,
    UINT32 maxSendRetries);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhMessageWithResponseSend (
           IxNpeMhNpeId npeId,
           IxNpeMhMessage message,
           IxNpeMhMessageId solicitedMessageId,
           IxNpeMhCallback solicitedCallback,
           UINT32 maxSendRetries)
 *
 * @brief This function is equivalent to the ixNpeMhMessageSend() function,
 * but must be used when the message being sent will solicited a response.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to send the message
 * to.
 * @param IxNpeMhMessage message (in) - The message to send.
 * @param IxNpeMhMessageId solicitedMessageId (in) - The ID of the
 * solicited response message.
 * @param IxNpeMhCallback solicitedCallback (in) - The function to use to
 * pass the response message back to the client.  A value of NULL will
 * cause the response message to be discarded.
 * @param UINT32 maxSendRetries (in) - Max num. of retries to perform
 * if the NPE's inFIFO is full.
 *
 * This function is equivalent to the ixNpeMhMessageSend() function, but
 * must be used when the message being sent will solicited a
 * response.<P>The client must specify the ID of the solicited response
 * message to allow the response to be recognised when it is received.  The
 * client must also specify a callback function to handle the received
 * response.  The IxNpeMh component will not offer the facility to send a
 * message to a NPE and receive a response within the same context.<P>Note
 * if the client is not interested in the response, specifying a NULL
 * callback will cause the response message to be discarded.<P>The
 * solicited callback will be stored and called some time later from an ISR
 * that will be triggered by the NPE's "outFIFO not empty" interrupt (see
 * ixNpeMhInitialize()) to handle the response message corresponding to the
 * message sent.  Response messages will be handled in the order they are
 * received.<P>
 * <P>The inFIFO may fill up sometimes if the Xscale is sending messages
 * faster than the NPE can handle them. This forces us to retry attempts 
 * to send the message until the NPE services the inFIFO. The client should
 * specify a ceiling value for the number of retries suitable to their
 * needs. IX_NPEMH_SEND_RETRIES_DEFAULT can be used as a default value for
 * the <i>maxSendRetries</i> parameter for this function. Each retry
 * exceeding this default number will incur a blocking delay of 1 microsecond,
 * to avoid consuming too much AHB bus bandwidth while performing retries.
 * <P><B>Re-entrancy:</B> This function will be callable from any
 * thread at any time.  IxOsServices will be used for any necessary
 * resource protection.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhMessageWithResponseSend (
    IxNpeMhNpeId npeId,
    IxNpeMhMessage message,
    IxNpeMhMessageId solicitedMessageId,
    IxNpeMhCallback solicitedCallback,
    UINT32 maxSendRetries);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhMessagesReceive (
           IxNpeMhNpeId npeId)
 *
 * @brief This function will receive messages from a particular NPE and
 * pass each message to the client via a solicited callback (for solicited
 * messages) or an unsolicited callback (for unsolicited messages).
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to receive and
 * process messages from.
 *
 * This function will receive messages from a particular NPE and pass each
 * message to the client via a solicited callback (for solicited messages)
 * or an unsolicited callback (for unsolicited messages).<P>If the IxNpeMh
 * component is initialised to service NPE "outFIFO not empty" interrupts
 * (see ixNpeMhInitialize()) then there is no need to call this function.
 * This function is only provided as an alternative mechanism to control
 * the receiving and processing of messages from the NPEs.<P>Note this
 * function cannot be called from within an ISR as it will use resource
 * protection mechanisms.<P><B>Re-entrancy:</B> This function will be
 * callable from any thread at any time.  IxOsServices will be used for any
 * necessary resource protection.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhMessagesReceive (
    IxNpeMhNpeId npeId);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhShow (
           IxNpeMhNpeId npeId)
 *
 * @brief This function will display the current state of the IxNpeMh
 * component.<P><B>Re-entrancy:</B> This function will be callable from
 * any thread at any time.  However, no resource protection will be used
 * so as not to impact system performance.  As this function is only
 * reading statistical information then this is acceptable.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to display state
 * information for.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhShow (
    IxNpeMhNpeId npeId);

/**
 * @ingroup IxNpeMh
 *
 * @fn IX_STATUS ixNpeMhShowReset (
           IxNpeMhNpeId npeId)
 *
 * @brief This function will reset the current state of the IxNpeMh
 * component.<P><B>Re-entrancy:</B> This function will be callable from
 * any thread at any time.  However, no resource protection will be used
 * so as not to impact system performance.  As this function is only
 * writing statistical information then this is acceptable.
 *
 * @param IxNpeMhNpeId npeId (in) - The ID of the NPE to reset state
 * information for.
 *
 * @return The function returns a status indicating success or failure.
 */

IX_STATUS ixNpeMhShowReset (
    IxNpeMhNpeId npeId);

#endif /* IXNPEMH_H */

/**
 * @} defgroup IxNpeMh
 */
