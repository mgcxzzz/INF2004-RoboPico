/* The shared source must retain its old behaviour unless explicitly opted in. */
#define JUNCTION_ANGLE_GATE_ONLY 0
#include "test_junction_policy.c"
_Static_assert(B_SIDE_AHEAD_MM == 92, "Non-barcode migration geometry must retain SDK default");
_Static_assert(BARCODE_CONFIRM_GATE == 0, "Non-barcode profiles retain the legacy approach gate");
