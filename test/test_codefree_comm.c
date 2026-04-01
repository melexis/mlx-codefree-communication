#include "unity.h"
#include "codefree_comm.h"
#include "mock_codefree_comm_external.h"

void dummy_app_callback(CodefreeCommReadMessage_t* msg) {
    /* Do nothing for now */
}

void setUp(void) {
}

void tearDown(void) {
}

/* Test 1: Initialization */
void test_codefree_comm_init_should_ReturnTrue_WhenDriverInitSucceeds(void) {
    codefree_comm_ext_initDriver_ExpectAndReturn(true);
    bool result = codefree_comm_init(dummy_app_callback);

    TEST_ASSERT_TRUE(result);
}