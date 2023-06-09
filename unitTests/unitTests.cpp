#include <iostream>
#include <string>

// Macro for unit testing.
#define UNIT_TEST(test, obtained, expected) \
({ \
    if ((obtained) == (expected)) { \
        std::cout << "[PASSED] " << (test) << "\n"; \
 \
    } else { \
        std::cout << "[FAILED] " << (test) << "\nExpected: " << (expected) << ". Obtained: " << (obtained) << ".\n"; \
    } \
\
    (obtained) != (expected); \
})

// Macro for raycast unit tests.
#define RAYCAST_TEST(test, obtained, expected, dist, expectedDist) \
({ \
    if ((obtained) == (expected) && ZMath::compare((dist), (expectedDist))) { \
        std::cout << "[PASSED] " << (test) << "\n"; \
        \
    } else if ((obtained) != (expected)) { \
        std::cout << "[FAILED] " << (test) << "\nExpected: " << (expected) << ". Obtained: " << (obtained) << ".\n"; \
        \
    } else { \
        std::cout << "[FAILED] " << (test) << "\nExpected Distance: " << (expectedDist) << ". Obtained: " << (dist) << ".\n"; \
        \
    } \
    \
    (obtained) != (expected) || !ZMath::compare((dist), (expectedDist)); \
})

// Run a function of unit tests.
bool testCases(std::string const &test, bool (*func)()) {
    std::cout << "================== " << test << " Tests. ==================\n\n";

    if (func()) {
        std::cout << "\n================ [FAILED] " << test << ". ================\n\n";
        return 1;
    }

    std::cout << "\n================ [PASSED] " << test << ". ================\n\n";
    return 0;
};
