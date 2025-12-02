#ifndef GLOBEART_SRC_GLOBE_TESTING_MACROS_HPP_
#define GLOBEART_SRC_GLOBE_TESTING_MACROS_HPP_

#include <cstdlib>
#include <gtest/gtest.h>

#define REQUIRE_EXPENSIVE() \
    do { \
        const char* env_var = std::getenv("EXPENSIVE"); \
        if (!env_var || env_var[0] == '\0') { \
            GTEST_SKIP() << "Skipping expensive test (set EXPENSIVE=1 to run)"; \
        } \
    } while (0)

#endif //GLOBEART_SRC_GLOBE_TESTING_MACROS_HPP_
