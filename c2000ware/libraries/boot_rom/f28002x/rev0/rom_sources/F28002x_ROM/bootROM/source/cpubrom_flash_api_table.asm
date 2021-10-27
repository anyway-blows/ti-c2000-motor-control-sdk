;;###########################################################################
;;
;; FILE:    rom_flash_api_table.asm
;;
;; TITLE:   Defines the FLash ROM API tables
;;
;;###########################################################################
;; $TI Release:  $
;; $Release Date:  $
;;###########################################################################


    .def    _rom_flash_ApiTable_start
    .def    __romApi_Fapi_getLibraryInfo
    .def    __romApi_Fapi_initializeAPI
    .def    __romApi_Fapi_setActiveFlashBank
    .def    __romApi_Fapi_issueAsyncCommandWithAddress
    .def    __romApi_Fapi_doBlankCheck
    .def    __romApi_Fapi_issueProgrammingCommand
    .def    __romApi_Fapi_issueProgrammingCommandForEccAddresses
    .def    __romApi_Fapi_doVerify
    .def    __romApi_Fapi_doVerifyBy16bits
    .def    __romApi_Fapi_issueAsyncCommand
    .def    __romApi_Fapi_issueFsmSuspendCommand
    .def    __romApi_Fapi_checkFsmForReady
    .def    __romApi_Fapi_getFsmStatus
    .def    __romApi_Fapi_calculateEcc
    .def    __romApi_Fapi_calculateFletcherChecksum
    .def    __romApi_Fapi_isAddressEcc
    .def    __romApi_Fapi_remapEccAddress
    .def    __romApi_Fapi_flushPipeline
    .def    __romApi_Fapi_calculatePsa
    .def    __romApi_Fapi_doPsaVerify
    .def    _rom_flash_ApiTable_end


    .ref    Fapi_getLibraryInfo
    .ref    Fapi_initializeAPI
    .ref    Fapi_setActiveFlashBank
    .ref    Fapi_issueAsyncCommandWithAddress
    .ref    Fapi_doBlankCheck
    .ref    Fapi_issueProgrammingCommand
    .ref    Fapi_issueProgrammingCommandForEccAddresses
    .ref    Fapi_doVerify
    .ref    Fapi_doVerifyBy16bits
    .ref    Fapi_issueAsyncCommand
    .ref    Fapi_issueFsmSuspendCommand
    .ref    Fapi_checkFsmForReady
    .ref    Fapi_getFsmStatus
    .ref    Fapi_calculateEcc
    .ref    Fapi_calculateFletcherChecksum
    .ref    Fapi_isAddressEcc
    .ref    Fapi_remapEccAddress
    .ref    Fapi_flushPipeline
    .ref    Fapi_calculatePsa
    .ref    Fapi_doPsaVerify


    .sect ".romFlashApiTable"
_rom_flash_ApiTable_start:
__romApi_Fapi_getLibraryInfo:
    LB Fapi_getLibraryInfo
__romApi_Fapi_initializeAPI:
    LB Fapi_initializeAPI
__romApi_Fapi_setActiveFlashBank:
    LB Fapi_setActiveFlashBank
__romApi_Fapi_issueAsyncCommandWithAddress:
    LB Fapi_issueAsyncCommandWithAddress
__romApi_Fapi_doBlankCheck:
    LB Fapi_doBlankCheck
__romApi_Fapi_issueProgrammingCommand:
    LB Fapi_issueProgrammingCommand
__romApi_Fapi_issueProgrammingCommandForEccAddresses:
    LB Fapi_issueProgrammingCommandForEccAddresses
__romApi_Fapi_doVerify:
    LB Fapi_doVerify
__romApi_Fapi_doVerifyBy16bits:
    LB Fapi_doVerifyBy16bits
__romApi_Fapi_issueAsyncCommand:
    LB Fapi_issueAsyncCommand
__romApi_Fapi_issueFsmSuspendCommand:
    LB Fapi_issueFsmSuspendCommand
__romApi_Fapi_checkFsmForReady:
    LB Fapi_checkFsmForReady
__romApi_Fapi_getFsmStatus:
    LB Fapi_getFsmStatus
__romApi_Fapi_calculateEcc:
    LB Fapi_calculateEcc
__romApi_Fapi_calculateFletcherChecksum:
    LB Fapi_calculateFletcherChecksum
__romApi_Fapi_isAddressEcc:
    LB Fapi_isAddressEcc
__romApi_Fapi_remapEccAddress:
    LB Fapi_remapEccAddress
__romApi_Fapi_flushPipeline:
    LB Fapi_flushPipeline
__romApi_Fapi_calculatePsa:
    LB Fapi_calculatePsa
__romApi_Fapi_doPsaVerify:
    LB Fapi_doPsaVerify
_rom_flash_ApiTable_end:


