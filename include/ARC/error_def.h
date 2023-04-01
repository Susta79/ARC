/*****************************************************************************
 * error_def.h
 *****************************************************************************
 *
 * Project:     ARC - AutomationsRobotic Robot Controller
 * Description: Macro definitions for return and error codes.
 *
 *****************************************************************************/
/** @file error_def.h
 * Macro definitions for return and error codes.
 *
 * The API distinguishes between two types of codes to signify different
 * failure modes. Return codes (MVCode_t) merely encode the reason for a
 * failure. Most of the Communicator API functions return an MVCode_t. The
 * caller of those functions can react to the return code appropriately based
 * on the context of the call.
 *
 * Error codes on the other hand are emitted when a failure does not result
 * as an immediate reaction to an API function call. Depending on the
 * component where the error occurred, the error code will be either be
 * delivered through an Error event (if it stems from the vision system or
 * vision application executing on the former) or it will be handled as an
 * exception inside of the Communicator where it is logged and may cause
 * termination of the Communicator or individual plug-ins inside of it.
 *
 * Error codes form a superset of return codes as they "decorate" the return
 * code with additional information on the system component which raised the
 * error and on its severity. The return codes itself merely encodes the
 * reason and nature of the error. The return code makes up the less
 * significant byte of the two-byte error code, with the affected component
 * and severity sharing four bits each of the more significant byte:
 *
 * ```
 *    | (4 bits)   12 | (4 bits)    8 |           (8 bits)          0 |
 *     \_____________/ \_____________/ \_____________________________/
 *        Severity        Component             Return Code
 * ```
 *
 ****************************************************************************
 */

#ifndef ERROR_DEF_H
#define ERROR_DEF_H

#include <stdint.h>

/** Type alias used for return codes.
 *
 * Return codes only occupy one byte. */
typedef uint32_t ARCCode_t;

/** Type alias used for error codes.
 *
 * Error codes occupy two bytes. The less significant byte holds the return code which encodes the reason for the error.
 * The more significant byte encodes the affect component on its four least significant bits and the severity of the
 * error on the remaining four most significant bits. */
typedef uint32_t ARCError_t;


/// @name Generic return codes
/// @{

/** A generic error has occurred. */
#define ARC_CODE_UNKNOWN                    0x00u

/** The command was processed successfully. */
#define ARC_CODE_OK                         0x01u

/** The command failed for an unknown reason. */
#define ARC_CODE_FAILED                     0x02u
/// @}

/// @name System error return codes
/// @{

/** A severe error occurred that caused the internal state of the system to become inconsistent. A restart of the
 *  system is required. */
#define ARC_CODE_INTERNAL_ERROR             0x10u

/** The command is not supported. */
#define ARC_CODE_NOT_IMPLEMENTED            0x13u

/** The command could not be executed within the specified timeout interval. */
#define ARC_CODE_TIMEOUT                    0x14u

/** The connection to the vision system has been lost. */
#define ARC_CODE_CONNECTION_LOST            0x15u

/** A file access failed. Perhaps the file does not exist or you have insufficient permissions to access it. */
#define ARC_CODE_FILE_ACCESS                0x18u
/// @}

/// @name Inverse kinematic return codes
/// @{

/** The requested pose cannot be reached with the requested robot configuration. */
#define ARC_CODE_POSE_NOT_REACHABLE        0x30u

/** The requested action is not allowed in the current state of the vision system. */
#define ARC_CODE_J2_TOO_CLOSE              0x31u

/** The requested action will not finish as expected because it was superseded by another action which takes precedence. */
//#define ARC_CODE_ACTION_SUPERSEDED          0x32u

/** The application that is specified in the recipe file could not be loaded. */
//#define ARC_CODE_PREP_RECIPE                0x33u

/** The application could not be unloaded; the system failed to restore the Initialized state. */
//#define ARC_CODE_UNPREP_RECIPE              0x34u

/** The status of the requested command is unknown and will not be set in the future. */
//#define ARC_CODE_BROKEN_PROMISE             0x35u

/** The dataflow of the application that is specified in the requested recipe file has blocks or races. */
//#define ARC_CODE_DATAFLOW                   0x36u

/** The image acquisition timed out. Perhaps a hardware triggered camera did not receive the trigger signal 
* within the specified "grab_timeout" interval or the connection to a camera was lost during acquisition. */
//#define ARC_CODE_ACQ_TIMEOUT                0x37u

/** The image acquisition partially timed out. In a multi-camera setup, the acquisition
* by some hardware-triggered cameras succeeded while the acquisition by other cameras timed out
* and it cannot be ensured that the image buffers of all cameras are in a consistent state. */
//#define ARC_CODE_ACQ_CONFLICTING_TIMEOUT    0x38u

/** The image acquisition failed. */
//#define ARC_CODE_ACQ_ERROR                  0x39u
/// @}

/// @name Plug-in management return codes
/// @{

/** The Communicator could not be started because no plug-ins have been specified. */
#define ARC_CODE_NO_PLUGINS_SPECIFIED       0x50u

/** A plug-in could not be loaded because either the corresponding dynamic library could not be opened or it does not export
 *  the API functions to identify itself. */
#define ARC_CODE_PLUGIN_LOAD_FAILED         0x51u

/** A plug-in has been loaded with a capability that it does not support. */
#define ARC_CODE_CAPABILITY_NOT_SUPPORTED   0x52u

/** A plug-in does not implement one of the required plug-in API functions. */
#define ARC_CODE_API_NOT_IMPLEMENTED        0x53u

/** A plug-in API function could not be called because the plug-in is not registered properly. */
#define ARC_CODE_PLUGIN_NOT_REGISTERED      0x54u

/** A plug-in API function could not be called because it is only applicable when the plug-in is either currently running or
 *  not yet running.  */
#define ARC_CODE_PLUGIN_RUNNING_STATE       0x55u

/** A plug-in could not be instantiated. */
#define ARC_CODE_PLUGIN_INSTANTIATION       0x56u

/// @}

/// @name Plug-in configuration return codes
/// @{

/** The plug-in configuration file could not be parsed. */
#define ARC_CODE_INV_CONFIG_FILE            0x60u

/** The value stored in a plug-in configuration file is invalid. */
#define ARC_CODE_INV_CONFIG_VALUE           0x61u

/** The (de-)serialization of a plug-in's configuration failed. */
#define ARC_CODE_CONFIG_SERIALIZATION       0x62u

/** An existing user parameter has been declared for a second time. */
#define ARC_CODE_USERPARAM_REDECL           0x63u
/// @}

/// @name Unknown entity return codes
/// @{

/** The specified action type is unknown. */
#define ARC_CODE_UNK_ACTION                 0x70u

/** The specified event type is unknown. */
#define ARC_CODE_UNK_EVENT                  0x71u

/** The specified parameter key is unknown. */
#define ARC_CODE_UNK_PARAM                  0x72u

/** The specified value type is unknown. */
#define ARC_CODE_UNK_VALUE_TYPE             0x73u

/** The specified data type is unknown. */
#define ARC_CODE_UNK_DATA_TYPE              0x74u

/** The specified recipe ID is unknown. */
#define ARC_CODE_UNK_RECIPE                 0x75u

/** The specified result ID is unknown. The corresponding result may have been dropped from the result store's ring buffer
 *  and is no longer available. */
#define ARC_CODE_UNK_RESULT                 0x76u

/** The specified filter key is unknown. */
#define ARC_CODE_UNK_FILTER                 0x77u

/** The specified user parameter is unknown, i.e., it has not been exposed by adding it to the configuration description. */
#define ARC_CODE_UNK_USERPARAM              0x78u
/// @}

/// @name Invalid argument & parameter return codes
/// @{

/** The specified parameter key does not apply to the queried object. */
#define ARC_CODE_INV_PARAM                  0x90u

/** The specified value is invalid in this context. */
#define ARC_CODE_INV_VALUE                  0x91u

/** The specified value type is invalid; check the documentation to ascertain whether scalars or arrays are supported in this
 *  context, respectively. */
#define ARC_CODE_INV_VALUE_TYPE             0x92u

/** The specified handle to the entity that is being manipulated is invalid; ensure that the entity is initialized
 *  beforehand. */
#define ARC_CODE_INV_HANDLE                 0x93u

/** The object referenced by the specified handle is invalid. */
#define ARC_CODE_INV_OBJECT                 0x94u

/** The provided recipe input values have been rejected by the application, despite their conformity to the recipe
 *  interface. */
#define ARC_CODE_INV_RECIPE_INPUT           0x95u

/** The provided job parameter tuple does not conform to the recipe interface, e.g., because the number of tuple elements
 *  does not match the number of recipe values or the conversion of the tuple elements to the data type of the respective
 *  recipe value is lossy. */
#define ARC_CODE_INV_JOB_PARAMS             0x96u

/** The result output values of the application do not match the expected recipe interface. */
#define ARC_CODE_INV_RECIPE_OUTPUT          0x97u

/** The provided filter value is invalid for the specified filter key. */
#define ARC_CODE_INV_FILTER                 0x98u

/** No user data could be retrieved for this plug-in because none has been registered. */
#define ARC_CODE_INV_USER_DATA              0x99u

/** The specified data type is invalid; check the documentation to see which data types are allowed in this context. */
#define ARC_CODE_INV_DATA_TYPE              0x9Au

/** The provided contraint value is invalid for the specified constraint type. */
#define ARC_CODE_INV_CONSTRAINT             0x9Bu

/** A plug-in could either not be registered or accessed by the plug-in manager because its ID is invalid. */
#define ARC_CODE_INV_PLUGIN_ID              0x9Cu
/// @}

/// @name Data manipulation return codes
/// @{

/** The provided array value is empty. */
#define ARC_CODE_VALUE_IS_EMPTY             0xB0u

/** The specified list is empty. */
#define ARC_CODE_LIST_IS_EMPTY              0xB1u

/** The value manipulation function which has been called does not match the type of the specified value. */
#define ARC_CODE_VALUE_TYPE_MISMATCH        0xB2u

/** The value manipulation function which has been called does not match the data type of the specified value. */
#define ARC_CODE_DATA_TYPE_MISMATCH         0xB3u

/** The requested array value element is out of range for the specified value. */
#define ARC_CODE_OUT_OF_RANGE               0xB4u

/** The parameter value to be retrieved has not been set. */
#define ARC_CODE_UNSET_PARAM                0xB5u
/// @}

/** @name Error severity
 *  Error codes have an implied severity. It is encoded in the four most significant bits of the 16-bit error code.
 */
/// @{
#define ARC_SEVERITY_NONE          0x0000u
#define ARC_SEVERITY_CRITICAL      0x1000u ///< Critical errors necessitate a restart of the affected component.
#define ARC_SEVERITY_ERROR         0x2000u ///< Regular errors indicate that some operation failed recoverably.
#define ARC_SEVERITY_WARNING       0x4000u ///< Warnings indicate that an operation was carried out but may have been anomalous.
#define ARC_SEVERITY_MASK          0xF000u ///< Bit mask to extract the severity from an error code.
/**
 * Function-like macro which applies a bit mask to an error code to extract its severity.
 */
#define ARC_ERROR_SEVERITY(error)  ((error) & ARC_SEVERITY_MASK)
/**
 * Function-like macro to change the severity of a given error code whilst preserving other information.
 */
#define ARC_SEVERITY_OVERRIDE(error, sev) (ARC_ERROR_CODE(error) | ARC_ERROR_COMPONENT(error) | ARC_ERROR_SEVERITY(sev))
/// @}

/** @name Error affected component
 *  Error codes have an implied affected component. It is encoded in the 5th to 8th most significant bits of the 16-bit
 *  error code.
 */
/// @{
#define ARC_COMPONENT_GENERAL      0x0000u
#define ARC_COMPONENT_SYSTEM       0x0100u ///< System errors pertain to the vision system and its state machine.
#define ARC_COMPONENT_KINEMATIC    0x0200u ///< Kinematic errors pertain to the kinematic of the robot (FK & IK).
#define ARC_COMPONENT_COMMUNICATOR 0x0400u ///< Communicator errors pertain to the framework for loading and running plug-ins.
//#define ARC_COMPONENT_PLUGIN       0x0800u ///< Plug-in errors are plug-in specific.
#define ARC_COMPONENT_MASK         0x0F00u ///< Bit mask to extract the component from an error code.
/**
 * Function-like macro which applies a bit mask to an error code to extract its affected component.
 */
#define ARC_ERROR_COMPONENT(error) ((error) & ARC_COMPONENT_MASK)
/// @}

/** @name Error return code
 *  Error codes "decorate" one of the ARC_CODE_* return codes with additional information on the severity and the affected
 *  component. The original return code is encoded in the eight least significant bits of the 16-bit error code.
 */
/// @{
#define ARC_ERROR_MASK             0x00FFu ///< Bit mask to extract the return code from a decorated error code.
/**
 * Function-like macro which applies a bit mask to extract the return code from a decorated error code.
 */
#define ARC_ERROR_CODE(error)      ((error) &ARC)
/**
 * Function-like macro which can be used to test if a return code indicates success.
 */
#define ARC_IsCodeOk(code)    (ARC(code)) == ARC_CODE_OK)
/**
 * Function-like macro which can be used to test if a return code indicates failure.
 */
#define ARC_IsCodeError(code) (ARC(code)) != ARC_CODE_OK)
/// @}


/** @name Vision System Error Codes
 *  These error codes are emitted in the form of Error events by the vision system.
 *
 *  Critical errors will stop the vision system from responding and require user intervention. The vision system will
 *  typically transition to the Error state when a critical error is emitted if it is still able to do so.
 *
 *  Regular errors will typically not transition to the Error state but it is possible that an automatic transition to
 *  another state takes place in reaction to such an error or that a transition which would generally be expected as a
 *  result of the offending command did not take place.
 *
 *  Warnings are emitted to inform the user about certain anomalies in the execution of a command or action which were
 *  however not severe enough to cause it to fail as a whole. The vision system will typically execute the same state
 *  transitions which would generally be expected for nominal executions.
 */
/// @{
#define ARC_CRIT_SYS_INTERNAL_ERROR           (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_SYSTEM | ARC_CODE_INTERNAL_ERROR)
#define ARC_CRIT_SYS_OUT_OF_MEM               (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_SYSTEM | ARC_CODE_OUT_OF_MEM)
#define ARC_CRIT_SYS_INV_LICENSE              (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_SYSTEM | ARC_CODE_INV_LICENSE)
#define ARC_CRIT_SYS_UNPREP_RECIPE            (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_SYSTEM | ARC_CODE_UNPREP_RECIPE)

#define ARC_ERR_SYS_ACCESS_DENIED             (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_ACCESS_DENIED)
#define ARC_ERR_SYS_BROKEN_PROMISE            (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_BROKEN_PROMISE)
#define ARC_ERR_SYS_PREP_RECIPE               (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_PREP_RECIPE)
#define ARC_ERR_SYS_NOT_IMPLEMENTED           (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_NOT_IMPLEMENTED)
#define ARC_ERR_SYS_ACQ_DEV_LOST              (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_ACQ_DEV_LOST)
#define ARC_ERR_SYS_ACQ_CONFLICTING_TIMEOUT   (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_ACQ_CONFLICTING_TIMEOUT)
#define ARC_ERR_SYS_ACQ_ERROR                 (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_ACQ_ERROR)
#define ARC_ERR_SYS_IO_DEV_LOST               (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_IO_DEV_LOST)
#define ARC_ERR_SYS_UNK_RECIPE                (ARC_SEVERITY_ERROR | ARC_COMPONENT_SYSTEM | ARC_CODE_UNK_RECIPE)

#define ARC_WARN_SYS_UNK_RESULT               (ARC_SEVERITY_WARNING | ARC_COMPONENT_SYSTEM | ARC_CODE_UNK_RESULT)
#define ARC_WARN_SYS_ACTION_REJECTED          (ARC_SEVERITY_WARNING | ARC_COMPONENT_SYSTEM | ARC_CODE_ACTION_REJECTED)
#define ARC_WARN_SYS_ACTION_SUPERSEDED        (ARC_SEVERITY_WARNING | ARC_COMPONENT_SYSTEM | ARC_CODE_ACTION_SUPERSEDED)
#define ARC_WARN_SYS_ACQ_TIMEOUT              (ARC_SEVERITY_WARNING | ARC_COMPONENT_SYSTEM | ARC_CODE_ACQ_TIMEOUT)
/// @}

/** @name Application Error Codes
 *  These error codes are also emitted in the form of Error events by the vision system. In contrast to the above vision
 *  system errors, they indicate that the problem concerns the vision application which is executed as part of the recipe.
 */
/// @{
//#define ARC_CRIT_APP_OUT_OF_MEM               (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_APPLICATION | ARC_CODE_OUT_OF_MEM)

#define ARC_ERR_KIN__POSE_NOT_REACHABLE        (ARC_SEVERITY_ERROR | ARC_COMPONENT_KINEMATIC | ARC_CODE_POSE_NOT_REACHABLE)
#define ARC_ERR_APP_J2_TOO_CLOSE               (ARC_SEVERITY_ERROR | ARC_COMPONENT_KINEMATIC | ARC_CODE_J2_TOO_CLOSE)
//#define ARC_ERR_APP_INV_RECIPE_OUTPUT         (ARC_SEVERITY_ERROR | ARC_COMPONENT_APPLICATION | ARC_CODE_INV_RECIPE_OUTPUT)

//#define ARC_WARN_APP_INV_JOB_PARAMS           (ARC_SEVERITY_WARNING | ARC_COMPONENT_APPLICATION | ARC_CODE_INV_JOB_PARAMS)
/// @}

/** @name Communicator Error Codes
 *  These error codes are emitted by the Communicator and printed to its log.
 *
 *  Critical Communicator errors will typically lead to the termination of the Communicator and all running plug-ins. It is
 *  dependent upon the specific error whether or not an attempt will be made to stop and close the plug-ins.
 *
 *  Regular Communicator errors may only produce an error log entry or cause a specific plug-in to not be started while
 *  leaving all other plug-ins unaffected.
 */
/// @{
#define ARC_CRIT_COMM_OUT_OF_MEM              (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_OUT_OF_MEM)
#define ARC_CRIT_COMM_CONNECTION_LOST         (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_CONNECTION_LOST)
#define ARC_CRIT_COMM_NO_PLUGINS_SPECIFIED    (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_NO_PLUGINS_SPECIFIED)

#define ARC_ERR_COMM_ACCESS_DENIED            (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_ACCESS_DENIED)
#define ARC_ERR_COMM_TIMEOUT                  (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_TIMEOUT)
#define ARC_ERR_COMM_PLUGIN_LOAD_FAILED       (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_PLUGIN_LOAD_FAILED)
#define ARC_ERR_COMM_CAPABILITY_NOT_SUPPORTED (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_CAPABILITY_NOT_SUPPORTED)
#define ARC_ERR_COMM_API_NOT_IMPLEMENTED      (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_API_NOT_IMPLEMENTED)
#define ARC_ERR_COMM_PLUGIN_NOT_REGISTERED    (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_PLUGIN_NOT_REGISTERED)
#define ARC_ERR_COMM_PLUGIN_RUNNING_STATE     (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_PLUGIN_RUNNING_STATE)
#define ARC_ERR_COMM_INV_CONFIG_FILE          (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_INV_CONFIG_FILE)
#define ARC_ERR_COMM_INV_CONFIG_VALUE         (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_INV_CONFIG_VALUE)
#define ARC_ERR_COMM_CONFIG_SERIALIZATION     (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_CONFIG_SERIALIZATION)
#define ARC_ERR_COMM_FILE_ACCESS              (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_FILE_ACCESS)
#define ARC_ERR_COMM_INV_PLUGIN_ID            (ARC_SEVERITY_ERROR | ARC_COMPONENT_COMMUNICATOR | ARC_CODE_INV_PLUGIN_ID)
/// @}

/** @name Plug-in Error Codes
 *  Plug-in error codes are emitted by the Communicator and printed to its log when a plug-in returns a return code from one
 *  of its API functions which is different from ARC_CODE_OK. The return code is "decorated" with severity "ERROR" and
 *  component "PLUGIN" to form the error code. The respective return codes are plug-in-specific.
 */
/// @{
#define ARC_CRIT_PLUGIN_GENERIC               (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_PLUGIN | ARC_CODE_UNKNOWN)
#define ARC_CRIT_PLUGIN_OUT_OF_MEM            (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_PLUGIN | ARC_CODE_OUT_OF_MEM)
#define ARC_CRIT_PLUGIN_INV_LICENSE           (ARC_SEVERITY_CRITICAL | ARC_COMPONENT_PLUGIN | ARC_CODE_INV_LICENSE)

#define ARC_ERR_PLUGIN_GENERIC                (ARC_SEVERITY_ERROR | ARC_COMPONENT_PLUGIN | ARC_CODE_UNKNOWN)
#define ARC_ERR_PLUGIN_INSTANTIATION          (ARC_SEVERITY_ERROR | ARC_COMPONENT_PLUGIN | ARC_CODE_PLUGIN_INSTANTIATION)
/// @}

#endif // ERROR_DEF_H
