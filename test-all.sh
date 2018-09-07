set -e

VAGRIND_FLAGS="--quiet --leak-check=full --error-exitcode=12"
CXX_TEST_FLAGS="-Wall -Wextra -g -std=c++11 -DBNS_DEBUG -DEXIT_ON_ASSERT"

$CXX $CXX_TEST_FLAGS -O0 test_main.cpp -o test_main.out

valgrind $VAGRIND_FLAGS ./test_main.out

$CXX $CXX_TEST_FLAGS -O2 test_main.cpp -o test_main_opt.out

valgrind $VAGRIND_FLAGS ./test_main_opt.out




