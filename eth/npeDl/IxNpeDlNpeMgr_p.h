/**
 * @file IxNpeDlNpeMgr_p.h
 *
 * @author Intel Corporation
 * @date 14 December 2001
 * @brief This file contains the private API for the NpeMgr module.
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
 * @defgroup IxNpeDlNpeMgr_p IxNpeDlNpeMgr_p
 *
 * @brief The private API for the IxNpeDl NpeMgr module
 * 
 * @{
 */

#ifndef IXNPEDLNPEMGR_P_H
#define IXNPEDLNPEMGR_P_H


/*
 * Put the user defined include files required.
 */
#include "IxNpeDl.h"
#include "IxTypes.h"


/*
 * Function Prototypes
 */

/**
 * @fn IX_STATUS ixNpeDlNpeMgrVersionLoad (IxNpeDlNpeId npeId,
                                           UINT32 *versionCodePtr,
                                           BOOL verify)
 * 
 * @brief Loads a version of microcode onto an NPE
 *
 * @param IxNpeDlNpeId [in] npeId     - Id of target NPE
 * @param UINT32* [in] versionCodePtr - pointer to version code in image to be
 *                                      downloaded
 * @param BOOL [in] verify            - if TRUE, verify each word written to
 *                                      NPE memory.
 * 
 * This function loads a version containing blocks of microcode onto a
 * particular NPE. If the <i>verify</i> option is ON, NpeDl will read back each
 * word written and verify that it was written successfully
 * 
 * @pre
 *     - The NPE should be stopped beforehand
 *
 * @post
 *     - The NPE Instruction Pipeline may be flushed clean
 *
 * @return
 *     - IX_SUCCESS if the download was successful
 *     - IX_FAIL otherwise
 */ 
IX_STATUS
ixNpeDlNpeMgrVersionLoad (IxNpeDlNpeId npeId, UINT32 *versionCodePtr,
			  BOOL verify);


/**
 * @fn IX_STATUS ixNpeDlNpeMgrNpeReset (IxNpeDlNpeId npeId)
 * 
 * @brief sets a NPE to RESET state
 *
 * @param IxNpeDlNpeId [in] npeId - id of target NPE
 * 
 * This function performs a soft NPE reset by writing reset values to the
 * Configuration Bus Execution Control registers, the Execution Context Stack
 * registers, the Physical Register file, and the Context Store registers for 
 * each context number. It also clears inFIFO, outFIFO and Watchpoint FIFO.
 * It does not reset NPE Co-processors.
 * 
 * @pre
 *     - The NPE should be stopped beforehand
 *
 * @post
 *     - NPE NextProgram Counter (NextPC) will be set to a fixed initial value,
 *       such as 0.  This should be explicitly set by downloading State
 *       Information before starting NPE Execution.
 *     - The NPE Instruction Pipeline will be in a clean state.
 *
 * @return
 *     - IX_SUCCESS if the operation was successful
 *     - IX_FAIL otherwise
 */ 
IX_STATUS
ixNpeDlNpeMgrNpeReset (IxNpeDlNpeId npeId);


/**
 * @fn IX_STATUS ixNpeDlNpeMgrNpeStart (IxNpeDlNpeId npeId)
 * 
 * @brief Starts NPE Execution
 *
 * @param IxNpeDlNpeId [in] npeId - Id of target NPE
 * 
 * Ensures only background Execution Stack Level is Active, clears instruction
 * pipeline, and starts Execution on a NPE by sending a Start NPE command to
 * the NPE. Checks the execution status of the NPE to verify that it is
 * running.
 * 
 * @pre
 *     - The NPE should be stopped beforehand.
 *     - Note that this function does not set the NPE Next Program Counter 
 *       (NextPC), so it should be set beforehand if required by downloading 
 *       appropriate State Information.
 *
 * @post
 *
 * @return
 *     - IX_SUCCESS if the operation was successful
 *     - IX_FAIL otherwise
 */ 
IX_STATUS
ixNpeDlNpeMgrNpeStart (IxNpeDlNpeId npeId);


/**
 * @fn IX_STATUS ixNpeDlNpeMgrNpeStop (IxNpeDlNpeId npeId)
 * 
 * @brief Halts NPE Execution
 *
 * @param IxNpeDlNpeId [in] npeId - id of target NPE
 * 
 * Stops execution on an NPE by sending a Stop NPE command to the NPE.
 * Checks the execution status of the NPE to verify that it has stopped.
 *
 * @pre
 *
 * @post
 *
 * @return 
 *     - IX_SUCCESS if the operation was successful
 *     - IX_FAIL otherwise
 */ 
IX_STATUS
ixNpeDlNpeMgrNpeStop (IxNpeDlNpeId npeId);


/**
 * @fn void ixNpeDlNpeMgrStatsShow (void)
 *
 * @brief This function will display statistics of the IxNpeDl NpeMgr module
 *
 * @return none
 */
void
ixNpeDlNpeMgrStatsShow (void);


/**
 * @fn void ixNpeDlNpeMgrStatsReset (void)
 *
 * @brief This function will reset the statistics of the IxNpeDl NpeMgr module
 *
 * @return none
 */
void
ixNpeDlNpeMgrStatsReset (void);


#endif /* IXNPEDLIMAGEMGR_P_H */

/**
 * @} defgroup IxNpeDlNpeMgr_p
 */
