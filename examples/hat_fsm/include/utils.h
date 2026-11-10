#ifndef UTILS_H
#define UTILS_H

/// @brief A shorthand function to implement a repeated value range comparison
/// @param value Pointer to the value to be evaluated
/// @param low The low non-reachable boundary for the value
/// @param high The high non-reachable boundary for the value
/// @return true (1) if the value is between the boundary values, otherwise false (0)
static inline int inBetween(const float *value, float low, float high) {
    if (*value > low && *value < high) return 1;
    else return 0;
}

#endif