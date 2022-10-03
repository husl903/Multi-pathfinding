#ifndef STONESNGEMS_UTIL_H
#define STONESNGEMS_UTIL_H

#include <string>

#include "definitions.h"

namespace stonesngems {
namespace util {

Board parse_board_str(const std::string &board_str);

}    // namespace util
}    // namespace stonesngems

#endif    // STONESNGEMS_UTIL_H