// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright(c) 2026 Intel Corporation.
 */

#include <zephyr/fatal.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <xtensa_internal.h>

#define USER_STACK_SIZE 2048

static struct k_thread fault_thread;
static K_THREAD_STACK_DEFINE(fault_stack, USER_STACK_SIZE);
K_SEM_DEFINE(after_fault, 0, 1);

/**
 * @brief Raise a user-mode fatal reason from base-firmware code, outside any LLEXT.
 */
static __noinline void fault_in_base_firmware(unsigned int reason)
{
	xtensa_arch_except(reason);
}

static void user_fault(void *reason_arg, void *unused1, void *unused2)
{
	unsigned int reason = (uintptr_t)reason_arg;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	if (reason == K_ERR_CPU_EXCEPTION)
		__asm__ volatile("ill" ::: "memory");
	else
		fault_in_base_firmware(reason);

	k_sem_give(&after_fault);
}

/**
 * @brief Verify that the production fatal policy aborts a user thread without an expected-fault hook.
 */
static void check_user_fault(unsigned int reason)
{
	int ret;

	k_sem_reset(&after_fault);
	k_thread_create(&fault_thread, fault_stack, USER_STACK_SIZE, user_fault,
			(void *)(uintptr_t)reason, NULL, NULL, -1, K_USER, K_FOREVER);
	k_thread_access_grant(&fault_thread, &after_fault);
	k_thread_start(&fault_thread);

	ret = k_thread_join(&fault_thread, K_SECONDS(2));
	if (ret)
		k_thread_abort(&fault_thread);
	zassert_equal(ret, 0, "faulting user thread did not terminate: %d", ret);
	zassert_equal(k_sem_count_get(&after_fault), 0, "faulting instruction was resumed");
	zassert_false(k_is_user_context(), "supervisor test coordinator did not continue");
}

ZTEST(sof_boot, user_fatal_cpu_exception)
{
	check_user_fault(K_ERR_CPU_EXCEPTION);
}

ZTEST(sof_boot, user_fatal_oops_in_base_firmware)
{
	check_user_fault(K_ERR_KERNEL_OOPS);
}

ZTEST(sof_boot, user_fatal_stack_check_in_base_firmware)
{
	check_user_fault(K_ERR_STACK_CHK_FAIL);
}

ZTEST(sof_boot, user_fatal_rejects_unavailable_context)
{
	struct arch_esf invalid_frame = { 0 };

	zassert_false(xtensa_exception_is_user(NULL), "accepted unavailable frame");
	zassert_false(xtensa_exception_is_user(&invalid_frame),
		      "accepted non-exception supervisor context");
}
