#include <stdio.h>
#include <string.h>
#include "unity.h"

void app_main(void)
{
    printf("Total test count: %d\n", unity_get_test_count());

    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();

    unity_run_menu();
}
