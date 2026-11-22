#pragma once

void test_enter_critical();
void test_exit_critical();
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
