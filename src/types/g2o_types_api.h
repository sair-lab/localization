#ifndef G2O_TYPES_API_H
#define G2O_TYPES_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef types_sba_EXPORTS
#define G2O_TYPES_API __declspec(dllexport)
#else
#define G2O_TYPES_API __declspec(dllimport)
#endif
#else
#define G2O_TYPES_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_TYPES_API
#endif

#endif // G2O_TYPES_SBA_API_H
