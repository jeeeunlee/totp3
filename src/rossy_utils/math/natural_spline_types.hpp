#ifndef NATURAL_SPLINES_TYPES_H
#define NATURAL_SPLINES_TYPES_H

#include <iostream>

// natural : ZEROACC for cubic, ZEROJERK for quartic
enum SplineType{ NATURAL, // cubic, quartic
                SAMEJERK, // [not-a-knot] cubic
                SAMEACC, // cubic
                ZEROVEL, // cubic
                OPT // quartic
};
#endif