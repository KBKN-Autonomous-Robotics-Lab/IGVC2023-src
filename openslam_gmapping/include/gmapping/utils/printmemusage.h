#ifndef PRINTMEMUSAGE_H
#define PRINTMEMUSAGE_H
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <fstream>
#include <gmapping/utils/utils_export.h>
#include <iostream>
#include <string>

namespace GMapping {
void UTILS_EXPORT printmemusage();
};

#endif
