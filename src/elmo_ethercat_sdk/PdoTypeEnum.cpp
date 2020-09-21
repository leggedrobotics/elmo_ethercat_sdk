#include "elmo_ethercat_sdk/PdoTypeEnum.hpp"

std::ostream& operator<<(std::ostream& os, const elmo::TxPdoTypeEnum& txPdoTypeEnum){
    switch(txPdoTypeEnum){
        case elmo::TxPdoTypeEnum::NA:
            os << "NA";
            break;
        case elmo::TxPdoTypeEnum::TxPdoStandard:
            os << "TxPdoStandard";
            break;
        case elmo::TxPdoTypeEnum::TxPdoCST:
            os << "TxPdoCST";
            break;
        default:
            break;
    }
    return os;
}
std::ostream& operator<<(std::ostream& os, const elmo::RxPdoTypeEnum& rxPdoTypeEnum){
    switch(rxPdoTypeEnum){
        case elmo::RxPdoTypeEnum::NA:
            os << "NA";
            break;
        case elmo::RxPdoTypeEnum::RxPdoStandard:
            os << "RxPdoStandard";
            break;
        case elmo::RxPdoTypeEnum::RxPdoCST:
            os << "RxPdoCST";
            break;
        default:
            break;
    }
    return os;
}
