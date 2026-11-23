// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2021 Intel Corporation. All rights reserved.
//
// Author: Guennadi Liakhovetski <guennadi.liakhovetski@linux.intel.com>

#include <sof/list.h>
#include <rtos/spinlock.h>
#include <rtos/symbol.h>
#include <sof/audio/component.h>
#include <rtos/interrupt.h>
#include <sof/lib/memory.h>
#include <sof/schedule/dp_schedule.h>
#include <sof/schedule/ll_schedule_domain.h>
#include <sof/schedule/zephyr_ll.h>
#include <sof/schedule/schedule.h>
#include <rtos/task.h>
#include <sof/lib/perf_cnt.h>
#include <zephyr/kernel.h>
#include <ipc4/base_fw.h>
#include <sof/debug/telemetry/telemetry.h>

LOG_MODULE_DECLARE(ll_schedule, CONFIG_SOF_LOG_LEVEL);

/* Kernel part of the Zephyr LL scheduler: the scheduler initialization, the task
 * semaphore syscalls and the access grants to the scheduler objects. The rest is in
 * src/schedule/zephyr_ll.c
 */

#if CONFIG_SOF_USERSPACE_LL
/*
 * Kernel-only copy of zephyr_ll_locks[]. That array is writable by user
 * threads, so kernel code granting access to the locks must use this copy,
 * otherwise a user thread could make it grant access to any kernel object.
 */
static struct k_mutex *zephyr_ll_klocks[CONFIG_CORE_COUNT];

/* Kernel-only copy of the per-core scheduler pointers, for fault recovery */
static struct zephyr_ll *zephyr_ll_ksch[CONFIG_CORE_COUNT];

/* Upper bound on the temporary run list walked by the fault handler */
#define ZEPHYR_LL_FAULT_MAX_TASKS	256
#endif

#if CONFIG_DYNAMIC_OBJECTS
static struct list_item zephyr_ll_task_sem_list = LIST_INIT(zephyr_ll_task_sem_list);

struct zephyr_ll_task_sem {
	struct task *task;
	struct k_sem *sem;
	struct list_item list;
};

/*
 * The syscall validators pass in pdata that they read once from user memory
 * and validated, so these helpers must not read task->priv_data again.
 */
static int zephyr_ll_task_sem_alloc_pdata(struct task *task, struct zephyr_ll_pdata *pdata)
{
	struct zephyr_ll_task_sem *ts = rmalloc(SOF_MEM_FLAG_COHERENT, sizeof(*ts));

	if (!ts)
		return -ENOMEM;

	ts->sem = k_object_alloc(K_OBJ_SEM);
	if (!ts->sem) {
		rfree(ts);
		return -ENOMEM;
	}

	k_sem_init(ts->sem, 0, 1);

#if CONFIG_SOF_USERSPACE_LL
	/*
	 * The per-task semaphore is signalled from zephyr_ll_task_done(),
	 * which runs in the (unprivileged) LL scheduler thread when a task is
	 * freed while it is still running. k_object_alloc() only grants access
	 * to the calling thread (the IPC handler that creates the task), so the
	 * LL thread must be granted access explicitly, otherwise its
	 * k_sem_give() traps with a userspace permission fault.
	 *
	 * Resolve the LL thread from kernel-only per-core state keyed by the
	 * task's target core; never traverse the user-accessible scheduler or
	 * domain objects from privileged context.
	 */
	struct k_thread *ll_tid = zephyr_domain_thread_tid_for_core(task->core);

	if (ll_tid)
		k_thread_access_grant(ll_tid, ts->sem);
#endif /* CONFIG_SOF_USERSPACE_LL */

	ts->task = task;
	pdata->sem_p = ts->sem;
	/* List is protected by IPC serialization */
	list_item_append(&ts->list, &zephyr_ll_task_sem_list);

	return 0;
}

int z_impl_zephyr_ll_task_sem_alloc(struct task *task)
{
	return zephyr_ll_task_sem_alloc_pdata(task, task->priv_data);
}

static int zephyr_ll_task_sem_free_pdata(struct task *task, struct zephyr_ll_pdata *pdata)
{
	struct list_item *list;
	struct zephyr_ll_task_sem *ts;
	bool found = false;

	/* List is protected by IPC serialization */
	list_for_item(list, &zephyr_ll_task_sem_list) {
		ts = container_of(list, struct zephyr_ll_task_sem, list);
		if (ts->task == task) {
			found = true;
			break;
		}
	}

	if (!found)
		return -ENOENT;

	if (pdata->sem_p != ts->sem)
		return -EINVAL;

	list_item_del(list);
	k_object_free(ts->sem);
	rfree(ts);

	return 0;
}

int z_impl_zephyr_ll_task_sem_free(struct task *task)
{
	return zephyr_ll_task_sem_free_pdata(task, task->priv_data);
}

#ifdef CONFIG_USERSPACE
#include <zephyr/internal/syscall_handler.h>
#include <sof/userspace/syscall_caller.h>
/*
 * Read task->priv_data exactly once. The task lives in user memory and a
 * thread on another core can change the pointer after it has been checked.
 *
 * @return validated pdata pointer, or NULL if the task has none
 */
static struct zephyr_ll_pdata *zephyr_ll_task_pdata_vrfy(struct task *task)
{
	struct zephyr_ll_pdata *pdata;

	K_OOPS(K_SYSCALL_MEMORY_WRITE(task, sizeof(*task)));
	K_OOPS(k_usermode_from_copy(&pdata, &task->priv_data, sizeof(pdata)));
	if (pdata)
		K_OOPS(K_SYSCALL_MEMORY_WRITE(pdata, sizeof(*pdata)));

	return pdata;
}

static inline int z_vrfy_zephyr_ll_task_sem_alloc(struct task *task)
{
	struct zephyr_ll_pdata *pdata;

	SOF_SYSCALL_CALLER_CHECK(SOF_SYSUSER_INFRA);

	if (!task)
		return -EINVAL;
	pdata = zephyr_ll_task_pdata_vrfy(task);
	if (!pdata)
		return -EINVAL;

	return zephyr_ll_task_sem_alloc_pdata(task, pdata);
}
#include <zephyr/syscalls/zephyr_ll_task_sem_alloc_mrsh.c>

static inline int z_vrfy_zephyr_ll_task_sem_free(struct task *task)
{
	struct zephyr_ll_pdata *pdata;

	SOF_SYSCALL_CALLER_CHECK(SOF_SYSUSER_INFRA);

	if (!task)
		return -EINVAL;
	pdata = zephyr_ll_task_pdata_vrfy(task);
	if (!pdata)
		return -EINVAL;

	return zephyr_ll_task_sem_free_pdata(task, pdata);
}
#include <zephyr/syscalls/zephyr_ll_task_sem_free_mrsh.c>
#endif /* CONFIG_USERSPACE */
#endif /* CONFIG_DYNAMIC_OBJECTS */

#if CONFIG_SOF_USERSPACE_LL
struct k_thread *zephyr_ll_init_context(void *data, struct task *task)
{
	struct zephyr_ll *sch = data;
	struct zephyr_ll_pdata *pdata = task->priv_data;
	int ret;

	/*
	 * Use domain_thread_init() for privileged setup (thread creation,
	 * timer, access grants). domain_register() is now bookkeeping only
	 * and will be called later from user context when scheduling tasks.
	 */
	ret = domain_thread_init(sch->ll_domain, task);
	if (ret < 0) {
		tr_err(&ll_tr, "cannot init_context %d", ret);
		return NULL;
	}

	assert(!k_is_user_context());
	k_thread_access_grant(zephyr_domain_thread_tid(sch->ll_domain), sch->lock, pdata->sem_p);

	tr_dbg(&ll_tr, "granting access to lock %p for thread %p", sch->lock,
	       zephyr_domain_thread_tid(sch->ll_domain));
	tr_dbg(&ll_tr, "granting access to domain lock %p for thread %p", &sch->ll_domain->lock,
	       zephyr_domain_thread_tid(sch->ll_domain));

	return zephyr_domain_thread_tid(sch->ll_domain);
}

void user_ll_grant_access(struct k_thread *thread, int core)
{
	assert(core < CONFIG_CORE_COUNT && zephyr_ll_klocks[core] != NULL);

	k_thread_access_grant(thread, zephyr_ll_klocks[core]);
}

/*
 * Repair the scheduler state of a core whose LL thread died in the middle of
 * a tick, while holding the core's LL lock.
 *
 * The task being run is quarantined: marked faulted, so that it can never be
 * scheduled again, and cancelled, so that the next tick removes it from the
 * scheduler. The tasks already run in the tick are moved from the temporary
 * list back to the head of the task list, in front of the tasks not run yet,
 * which restores the priority order.
 *
 * The scheduler data is writable by the dying thread, so sch->running is
 * only trusted if it is on the temporary list, and the list walk is bounded.
 *
 * @return 0 on success, -EINVAL if the scheduler state cannot be trusted.
 */
static int zephyr_ll_fault_repair(struct zephyr_ll *sch)
{
	struct task *running = sch->running;
	struct list_item *first = sch->tasks.next;
	struct list_item *list, *tmp;
	unsigned int n = 0;
	bool found = false;

	list_for_item(list, &sch->run_list) {
		if (++n > ZEPHYR_LL_FAULT_MAX_TASKS) {
			tr_err(&ll_tr, "core %u: LL run list corrupted", sch->core);
			return -EINVAL;
		}

		if (container_of(list, struct task, list) == running)
			found = true;
	}

	if (running && found) {
		struct zephyr_ll_pdata *pdata = running->priv_data;

		if (pdata) {
			pdata->faulted = true;
			/* back on the main list below, no longer being run */
			pdata->run = false;
		}
		running->state = SOF_TASK_STATE_CANCEL;

		tr_err(&ll_tr, "core %u: task %p (%pU) faulted, cancelled",
		       sch->core, running, running->uid);
	}

	sch->running = NULL;

	list_for_item_safe(list, tmp, &sch->run_list) {
		list_item_del(list);
		/* insert in front of the first task not run in this tick */
		list_item_append(list, first);
	}

	return 0;
}

/*
 * Release all LL scheduler locks held by the faulting thread.
 *
 * Zephyr does not release mutexes when a thread is aborted, so every later
 * user_ll_lock_sched() on that core, e.g. from the IPC user thread
 * triggering a pipeline, would pend forever.
 *
 * The current thread is the mutex owner, so the regular unlock path is used
 * to hand the lock over to the first waiter. Rescheduling is deferred until
 * the exception returns.
 *
 * Uses the kernel-only zephyr_ll_klocks[] copy: zephyr_ll_locks[] is
 * user-writable and must not be trusted here.
 */
static int zephyr_ll_release_locks(struct k_thread *thread)
{
	int released = 0;
	int core;

	for (core = 0; core < CONFIG_CORE_COUNT; core++) {
		struct k_mutex *m = zephyr_ll_klocks[core];

		if (!m || m->owner != thread)
			continue;

		/* before unlocking: the new owner counts depth from 0 */
		zephyr_ll_lock_shadow_reset(core);

		/* the mutex is recursive, drop every level */
		while (m->owner == thread)
			if (z_impl_k_mutex_unlock(m))
				break;

		released++;
	}

	return released;
}

/**
 * @brief Recover the LL scheduler from a fault of the current user thread.
 *
 * Must only be called from k_sys_fatal_error_handler(), before
 * z_fatal_error() aborts _current.
 *
 * If the faulting thread is a core's LL thread, the scheduler state it was
 * modifying under the LL lock is repaired first, quarantining the task being
 * run, and a restart of the LL thread is requested. It is done later from a
 * work queue, after the thread has been aborted. Then all LL locks held by
 * the thread are released, which lets their waiters, e.g. the IPC user
 * thread, proceed.
 *
 * @return Number of LL scheduler locks released.
 */
int zephyr_ll_fault_recover_current(void)
{
	struct k_thread *thread = k_current_get();
	int core = zephyr_domain_thread_core(thread);
	bool restart = core >= 0;

	if (restart && zephyr_ll_ksch[core] && zephyr_ll_klocks[core] &&
	    zephyr_ll_klocks[core]->owner == thread &&
	    zephyr_ll_fault_repair(zephyr_ll_ksch[core]) < 0) {
		/* a new LL thread would run on corrupted lists */
		tr_err(&ll_tr, "core %d: LL scheduler not recoverable", core);
		restart = false;
	}

	/* before releasing the locks, the new owner may use the task lists */
	int released = zephyr_ll_release_locks(thread);

	if (restart)
		zephyr_domain_thread_restart(core);

	return released;
}
#endif /* CONFIG_SOF_USERSPACE_LL */

#if CONFIG_SOF_BOOT_TEST_STANDALONE || CONFIG_LIBRARY
static void zephyr_ll_scheduler_free(void *data, uint32_t flags)
{
	struct zephyr_ll *sch = data;

	zephyr_ll_assert_core(sch);

#if CONFIG_SOF_USERSPACE_LL
	zephyr_ll_locks[sch->core] = NULL;
	zephyr_ll_klocks[sch->core] = NULL;
	zephyr_ll_ksch[sch->core] = NULL;
	k_object_free(sch->lock);
#endif

	if (sch->n_tasks)
		tr_err(&ll_tr, "%u tasks are still active!",
		       sch->n_tasks);

#if CONFIG_SOF_USERSPACE_LL
	domain_thread_free(sch->ll_domain, sch->n_tasks);
#endif
	zephyr_ll_free(sch);
}
#endif /* CONFIG_SOF_BOOT_TEST_STANDALONE || CONFIG_LIBRARY */

static const struct scheduler_ops zephyr_ll_ops = {
	.schedule_task		= zephyr_ll_task_schedule,
	.schedule_task_before	= zephyr_ll_task_schedule_before,
	.schedule_task_after	= zephyr_ll_task_schedule_after,
	.schedule_task_free	= zephyr_ll_task_free,
	.schedule_task_cancel	= zephyr_ll_task_cancel,
#if CONFIG_SOF_BOOT_TEST_STANDALONE || CONFIG_LIBRARY
	.scheduler_free		= zephyr_ll_scheduler_free,
#endif
#if CONFIG_SOF_USERSPACE_LL
	.scheduler_init_context	= zephyr_ll_init_context,
#endif
};

/* TODO: low-power mode clock support */
/* Runs on each core during initialisation with the same domain argument */
__cold int zephyr_ll_scheduler_init(struct ll_schedule_domain *domain)
{
	struct zephyr_ll *sch;
	int core = cpu_get_id();

	assert_can_be_cold();

	tr_dbg(&ll_tr, "init on core %d", core);

	/* initialize per-core scheduler private data */
#if CONFIG_SOF_USERSPACE_LL
	sch = zephyr_ll_karea_alloc(ZEPHYR_LL_KAREA_RW, SOF_MEM_FLAG_COHERENT, sizeof(*sch), 0);
#else
	sch = sof_heap_alloc(sof_sys_heap_get(), SOF_MEM_FLAG_KERNEL | SOF_MEM_FLAG_COHERENT,
			     sizeof(*sch), 0);
#endif
	if (!sch) {
		tr_err(&ll_tr, "allocation failed");
		return -ENOMEM;
	}

	memset(sch, 0, sizeof(*sch));

	list_init(&sch->tasks);
	list_init(&sch->run_list);
	sch->ll_domain = domain;
	sch->core = core;
	sch->n_tasks = 0;

#if CONFIG_SOF_USERSPACE_LL
	/* Allocate mutex dynamically for userspace access */
	sch->lock = k_object_alloc(K_OBJ_MUTEX);
	if (!sch->lock) {
		tr_err(&ll_tr, "mutex allocation failed");
		zephyr_ll_free(sch);
		return -ENOMEM;
	}
	assert(core < CONFIG_CORE_COUNT && zephyr_ll_locks[core] == NULL);
	zephyr_ll_locks[core] = sch->lock;
	zephyr_ll_klocks[core] = sch->lock;
	zephyr_ll_ksch[core] = sch;
	k_mutex_init(sch->lock);

	tr_dbg(&ll_tr, "ll-scheduler init done, sch %p sch->lock %p", sch, sch->lock);
#endif

	scheduler_init(domain->type, &zephyr_ll_ops, sch);

	return 0;
}
