// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright(c) 2025 Intel Corporation. All rights reserved.
 *
 * Author: Marcin Szkudlinski
 */

#include <rtos/task.h>
#include <rtos/userspace_helper.h>

#include <sof/audio/module_adapter/module/generic.h>
#include <sof/audio/component.h>
#include <sof/common.h>
#include <sof/ipc/common.h>
#include <sof/ipc/msg.h>
#include <sof/lib/mailbox.h>
#include <sof/list.h>
#include <sof/kernel/llext_manager.h>
#include <sof/objpool.h>
#include <sof/schedule/dp_schedule.h>
#include <sof/schedule/schedule.h>
#include <sof/schedule/ll_schedule_domain.h>

#include <zephyr/app_memory/mem_domain.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <sof/schedule/zephyr_dp_schedule.h>

LOG_MODULE_DECLARE(dp_schedule, CONFIG_SOF_LOG_LEVEL);
extern struct tr_ctx dp_tr;

/* Kernel part of the application variant of the DP scheduler: the DP thread creation,
 * its memory domain, the kernel-only record of its kernel objects and the
 * scheduler_dp_internal_free() system call. The DP thread function and the IPC
 * forwarding to it are in src/schedule/zephyr_dp_schedule_application.c
 */

static struct objpool_head dp_mdom_head = {.list = LIST_INIT(dp_mdom_head.list)};
/*
 * Protects dp_mdom_head. objpool_alloc() and objpool_free() update the pool
 * without locking. DP task creation and the module delete path are serialized
 * by IPC processing, but the DP fault recovery work frees domains outside of
 * it, concurrently with a DP module being created on another core.
 */
static K_MUTEX_DEFINE(dp_mdom_lock);

static struct k_mem_domain *dp_mdom_alloc(void)
{
	struct k_mem_domain *mdom;

	k_mutex_lock(&dp_mdom_lock, K_FOREVER);
	mdom = objpool_alloc(&dp_mdom_head, sizeof(*mdom), SOF_MEM_FLAG_COHERENT);
	k_mutex_unlock(&dp_mdom_lock);

	return mdom;
}

static void dp_mdom_free(struct k_mem_domain *mdom)
{
	k_mutex_lock(&dp_mdom_lock, K_FOREVER);
	objpool_free(&dp_mdom_head, mdom);
	k_mutex_unlock(&dp_mdom_lock);
}

/* Synchronization semaphore for the scheduler thread to wait for DP startup */
#define DP_SYNC_INIT(i, _)	Z_SEM_INITIALIZER(dp_sync[i], 0, 1)
#define DP_SYNC_INIT_LIST	LISTIFY(CONFIG_CORE_COUNT, DP_SYNC_INIT, (,))
STRUCT_SECTION_ITERABLE_ARRAY(k_sem, dp_sync, CONFIG_CORE_COUNT) = { DP_SYNC_INIT_LIST };

/* State of a DP task record */
enum scheduler_dp_task_kobj_state {
	DP_KOBJ_ALIVE = 0,	/* the DP thread is running, 0 so zeroed records are alive */
	DP_KOBJ_FAULTED,	/* the DP thread faulted, queued for the fault recovery */
};

/*
 * Kernel-only record of the kernel objects owned by a DP task.
 *
 * struct scheduler_dp_task_memory lives in the module's heap, which is mapped
 * into the DP thread's memory domain, and is also writable by the LL and IPC
 * user threads. Freeing must therefore never use the pointers stored there:
 * scheduler_dp_internal_free() takes them from this record instead. The task
 * and module pointers are only used as lookup keys.
 */
struct scheduler_dp_task_kobj {
	struct list_item list;
	struct task *task;
	struct processing_module *mod;
	struct comp_dev *dev;		/* mod->dev at creation, mod is writable by user threads */
	const struct comp_driver *orig_drv; /* mod->dev->drv before it was pointed at the copy */
	struct k_event *event;
	struct k_thread *thread;
	k_thread_stack_t *stack;	/* the DP thread stack, pdata->p_stack is user-writable */
	struct ipc4_flat *flat;		/* IPC parameter area in task memory */
	void *config_data;		/* LARGE_CONFIG_GET reply buffer in task memory */
	struct k_mem_domain *mdom;
	struct k_mem_partition mpart[SOF_DP_PART_TYPE_COUNT];
	uint32_t comp_id;
	uint16_t core;
	uint32_t gen;			/* unique per record, see dp_task_kobj_take() */
	enum scheduler_dp_task_kobj_state state;
	struct list_item faulted;	/* in dp_fault_wq::faulted while DP_KOBJ_FAULTED */
};

/*
 * Per-core DP fault recovery work queue. Allocated from the kernel heap, not
 * defined statically, so that its thread isn't a kernel object and doesn't
 * use one of the few thread permission indices.
 */
struct dp_fault_wq {
	struct k_work_q q;
	struct k_work work;
	struct list_item faulted;	/* faulted DP task records, under dp_task_kobj_lock */
	uint16_t core;
};

static struct dp_fault_wq *dp_fault_wq[CONFIG_CORE_COUNT];
static K_KERNEL_STACK_ARRAY_DEFINE(dp_fault_wq_stack, CONFIG_CORE_COUNT,
				   CONFIG_ZEPHYR_DP_FAULT_WQ_STACK_SIZE);

static struct list_item dp_task_kobj_list = LIST_INIT(dp_task_kobj_list);
static struct k_spinlock dp_task_kobj_lock;
/* generation of the last added record, under dp_task_kobj_lock */
static uint32_t dp_task_kobj_gen;

static void dp_task_kobj_add(struct scheduler_dp_task_kobj *kobj)
{
	k_spinlock_key_t key = k_spin_lock(&dp_task_kobj_lock);

	kobj->gen = ++dp_task_kobj_gen;
	list_item_append(&kobj->list, &dp_task_kobj_list);
	k_spin_unlock(&dp_task_kobj_lock, key);
}

/* Find the record for @task, caller holds dp_task_kobj_lock */
static struct scheduler_dp_task_kobj *dp_task_kobj_find(const struct task *task)
{
	struct scheduler_dp_task_kobj *kobj;
	struct list_item *item;

	list_for_item(item, &dp_task_kobj_list) {
		kobj = container_of(item, struct scheduler_dp_task_kobj, list);
		if (kobj->task == task)
			return kobj;
	}

	return NULL;
}

/* Find the record of the DP thread @thread, caller holds dp_task_kobj_lock */
static struct scheduler_dp_task_kobj *dp_task_kobj_find_thread(const struct k_thread *thread)
{
	struct scheduler_dp_task_kobj *kobj;
	struct list_item *item;

	list_for_item(item, &dp_task_kobj_list) {
		kobj = container_of(item, struct scheduler_dp_task_kobj, list);
		if (kobj->thread == thread)
			return kobj;
	}

	return NULL;
}

/* Copy the record for @mod to @copy */
static bool dp_task_kobj_peek_mod(const struct processing_module *mod,
				  struct scheduler_dp_task_kobj *copy)
{
	k_spinlock_key_t key = k_spin_lock(&dp_task_kobj_lock);
	struct scheduler_dp_task_kobj *kobj;
	struct list_item *item;
	bool found = false;

	list_for_item(item, &dp_task_kobj_list) {
		kobj = container_of(item, struct scheduler_dp_task_kobj, list);
		if (kobj->mod == mod) {
			*copy = *kobj;
			found = true;
			break;
		}
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	return found;
}

/* Copy the record for @task to @copy without unlinking it */
static bool dp_task_kobj_peek(const struct task *task, struct scheduler_dp_task_kobj *copy)
{
	k_spinlock_key_t key = k_spin_lock(&dp_task_kobj_lock);
	struct scheduler_dp_task_kobj *kobj = dp_task_kobj_find(task);

	if (kobj)
		*copy = *kobj;

	k_spin_unlock(&dp_task_kobj_lock, key);

	return kobj != NULL;
}

/*
 * Find the record for @task and unlink it, so it can only be freed once.
 * If @expected is given, only take the record it was copied from, so a
 * record replaced since @expected was peeked is left alone.
 */
static struct scheduler_dp_task_kobj *
dp_task_kobj_take(const struct task *task, const struct scheduler_dp_task_kobj *expected)
{
	k_spinlock_key_t key = k_spin_lock(&dp_task_kobj_lock);
	struct scheduler_dp_task_kobj *kobj = dp_task_kobj_find(task);

	/*
	 * Compare the generation, not the object addresses: the task could
	 * have been freed and recreated at the same addresses meanwhile.
	 */
	if (kobj && expected && kobj->gen != expected->gen)
		kobj = NULL;

	if (kobj) {
		list_item_del(&kobj->list);
		/*
		 * A faulted record still queued for the fault recovery, the
		 * recovery work must not find it after it is freed. A no-op
		 * if it isn't queued: the item is then linked to itself.
		 */
		list_item_del(&kobj->faulted);
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	return kobj;
}

/*
 * Safe to call with partial successful initialisation,
 * k_mem_domain_remove_partition() then just returns -ENOENT
 */
static void scheduler_dp_domain_free(struct processing_module *pmod, struct k_mem_domain *mdom,
				     struct k_mem_partition *mpart, uint32_t comp_id)
{
	llext_manager_rm_domain(comp_id, mdom);

	k_mem_domain_remove_partition(mdom, mpart + SOF_DP_PART_HEAP);
	k_mem_domain_remove_partition(mdom, mpart + SOF_DP_PART_HEAP_CACHE);
	k_mem_domain_remove_partition(mdom, mpart + SOF_DP_PART_CFG);
	k_mem_domain_remove_partition(mdom, mpart + SOF_DP_PART_CFG_CACHE);

	if (mdom->num_partitions) {
		/* Violation: all partitions should be freed by now */
		unsigned int n = mdom->num_partitions;

		LOG_WRN("%u domain partition(s) not freed, force-freeing them all", n);

		for (unsigned int i = 0; i < arch_mem_domain_max_partitions_get() && n; i++) {
			struct k_mem_partition *dpart = &mdom->partitions[i];

			if (dpart->size) {
				k_mem_domain_remove_partition(mdom, dpart);
				n--;
			}
		}
	}

	/* drop messages still queued by the module, before the domain is reused */
	ipc_user_msg_purge(mdom);

	/* All partitions removed, the domain can be freed now */
	pmod->mdom = NULL;
	k_mem_domain_deinit(mdom);
	dp_mdom_free(mdom);
}

/*
 * Free the kernel objects of @task, using only the kernel-only record.
 *
 * @return false if @task has no record (unknown task or already freed)
 */
static bool scheduler_dp_internal_free_kobj(struct task *task,
					    const struct scheduler_dp_task_kobj *expected)
{
	struct scheduler_dp_task_kobj *kobj = dp_task_kobj_take(task, expected);

	if (!kobj)
		return false;

	/* normally already terminated by scheduler_dp_task_free(), no-op then */
	k_thread_abort(kobj->thread);
	k_object_free(kobj->event);
	k_object_free(kobj->thread);
	scheduler_dp_domain_free(kobj->mod, kobj->mdom, kobj->mpart, kobj->comp_id);

	/* mod->dev->drv points to the driver copy in the task memory freed next */
	kobj->dev->drv = kobj->orig_drv;
	mod_free(kobj->mod, container_of(task, struct scheduler_dp_task_memory, task));
	rfree(kobj);

	return true;
}

void z_impl_scheduler_dp_internal_free(struct task *task)
{
	if (!scheduler_dp_internal_free_kobj(task, NULL))
		tr_err(&dp_tr, "unknown DP task %p", task);
}

#ifdef CONFIG_THREAD_NAME
static void scheduler_dp_thread_name_set(k_tid_t thread_id, struct processing_module *mod)
{
	char name[CONFIG_THREAD_MAX_NAME_LEN];

	snprintf(name, sizeof(name), "DP:%#x", mod->dev->ipc_config.id);

	k_thread_name_set(thread_id, name);
}
#else
/* k_thread_name_set() is a no-op so skip constructing a thread name */
#define scheduler_dp_thread_name_set(x, y)
#endif

/* Called only in IPC context in kernel mode (this can change) */
int scheduler_dp_task_init(struct task **task, const struct sof_uuid_entry *uid,
			   const struct task_ops *ops, struct processing_module *mod,
			   uint16_t core, size_t stack_size, uint32_t options)
{
	k_thread_stack_t *p_stack;
	struct scheduler_dp_task_memory *task_memory;
	uint8_t *ipc_config_data = NULL;
	k_tid_t thread_id;
	uint32_t comp_id;

	int ret;

	/* must be called on the same core the task will be bound to */
	assert(cpu_get_id() == core);
	assert(!k_is_user_context());

	/*
	 * allocate memory
	 * to avoid multiple malloc operations allocate all required memory as a single structure
	 * and return pointer to task_memory->task
	 * As the structure contains zephyr kernel specific data, it must be located in
	 * shared, non cached memory
	 */
	task_memory = mod_alloc_ext(mod, SOF_MEM_FLAG_USER | SOF_MEM_FLAG_COHERENT,
				    sizeof(*task_memory), 0);
	if (!task_memory) {
		tr_err(&dp_tr, "memory alloc failed");
		return -ENOMEM;
	}

	memset(task_memory, 0, sizeof(*task_memory));

	const struct comp_driver *drv = mod->dev->drv;

	task_memory->drv = *mod->dev->drv;
	mod->dev->drv = &task_memory->drv;

	/* allocate stack - must be aligned and cached so a separate alloc */
	p_stack = user_stack_allocate(stack_size, options);
	if (!p_stack) {
		tr_err(&dp_tr, "stack alloc failed");
		ret = -ENOMEM;
		goto e_tmem;
	}

	struct task *ptask = &task_memory->task;

	/* internal SOF task init */
	ret = schedule_task_init(ptask, uid, SOF_SCHEDULE_DP, 0, ops->run, mod, core, options);
	if (ret < 0) {
		tr_err(&dp_tr, "schedule_task_init failed");
		goto e_stack;
	}

	/*
	 * task_memory, including pdata, is in the module heap, which user
	 * threads can write while this function runs. Keep everything the
	 * kernel uses here or at teardown in kobj and locals, and only mirror
	 * it into pdata for the DP thread.
	 */
	struct task_dp_pdata *pdata = &task_memory->pdata;
	/* zeroed: a new record is DP_KOBJ_ALIVE and fields added later start as NULL */
	struct scheduler_dp_task_kobj *kobj = rzalloc(SOF_MEM_FLAG_KERNEL | SOF_MEM_FLAG_COHERENT,
						      sizeof(*kobj));

	if (!kobj) {
		tr_err(&dp_tr, "task record alloc failed");
		ret = -ENOMEM;
		goto e_stack;
	}

	kobj->task = ptask;
	kobj->mod = mod;
	kobj->dev = mod->dev;
	/* the fault recovery frees task_memory, it restores mod->dev->drv from here first */
	kobj->orig_drv = drv;
	/* the fault recovery frees the stack, pdata->p_stack can't be trusted then */
	kobj->stack = p_stack;
	list_init(&kobj->faulted);
	kobj->core = core;
	comp_id = mod->dev->ipc_config.id;
	kobj->comp_id = comp_id;

	if (mod->dev->drv->adapter_ops->get_configuration) {
		ipc_config_data = mod_alloc_ext(mod, SOF_MEM_FLAG_USER, SOF_IPC_MSG_MAX_SIZE, 0);
		if (!ipc_config_data) {
			tr_err(&dp_tr, "Failed to allocate memory for get-config");
			ret = -ENOMEM;
			goto e_kobj;
		}
	}
	pdata->ipc_config_data = ipc_config_data;
	kobj->config_data = ipc_config_data;

	kobj->flat = &task_memory->flat;
	pdata->flat = kobj->flat;

	kobj->event = k_object_alloc(K_OBJ_EVENT);
	if (!kobj->event) {
		tr_err(&dp_tr, "Event object allocation failed");
		ret = -ENOMEM;
		goto e_config;
	}

	pdata->event = kobj->event;

	kobj->thread = k_object_alloc(K_OBJ_THREAD);
	if (!kobj->thread) {
		tr_err(&dp_tr, "Thread object allocation failed");
		ret = -ENOMEM;
		goto e_event;
	}
	memset(&kobj->thread->arch, 0, sizeof(kobj->thread->arch));
	pdata->thread = kobj->thread;

	/* success, fill the structures */
	pdata->p_stack = p_stack;
	pdata->mod = mod;

	/* initialize other task structures */
	ptask->ops.complete = ops->complete;
	ptask->ops.get_deadline = ops->get_deadline;
	ptask->priv_data = pdata;
	list_init(&ptask->list);

	/* create a zephyr thread for the task */
	thread_id = k_thread_create(kobj->thread, p_stack,
				    stack_size, dp_thread_fn, ptask, NULL, NULL,
				    CONFIG_DP_THREAD_PRIORITY, ptask->flags, K_FOREVER);
	pdata->thread_id = thread_id;
	scheduler_dp_thread_name_set(thread_id, mod);

	unsigned int pidx;
	size_t size;
	uintptr_t start;
	bool on_pool = false;

#ifdef CONFIG_SCHED_CPU_MASK
	/* pin the thread to specific core */
	ret = k_thread_cpu_pin(thread_id, core);
	if (ret < 0) {
		tr_err(&dp_tr, "zephyr task pin to core failed");
		goto e_thread;
	}
#endif

	k_thread_access_grant(thread_id, kobj->event, &dp_sync[core]);
	scheduler_dp_grant(thread_id, core);
#if CONFIG_SOF_USERSPACE_LL
	struct k_thread *thread_ipc = ipc_thread_user(core);

	k_thread_access_grant(thread_ipc, kobj->event, thread_id, p_stack, &dp_sync[core]);
	scheduler_dp_grant(thread_ipc, core);
	/* the IPC user thread frees DP tasks, see pipeline_comp_dp_task_free() */
	scheduler_dp_free_lock_grant(thread_ipc, core);
	scheduler_dp_grant(zephyr_ll_domain_thread(), core);
#endif

	struct k_mem_domain *mdom = dp_mdom_alloc();

	if (!mdom) {
		tr_err(&dp_tr, "objpool allocation failed");
		ret = -ENOMEM;
		goto e_thread;
	}

	on_pool = true;

	mod->mdom = mdom;
	kobj->mdom = mdom;

	if (!mdom->arch.ptables) {
		ret = k_mem_domain_init(mdom, 0, NULL);
		if (ret < 0)
			goto e_thread;
	}

	/* Module heap partition */
	mod_heap_info(mod, &size, &start);
	kobj->mpart[SOF_DP_PART_HEAP] = (struct k_mem_partition){
		.start = (uintptr_t)sys_cache_uncached_ptr_get((void *)start),
		.size = size,
		.attr = K_MEM_PARTITION_P_RW_U_RW,
	};
	kobj->mpart[SOF_DP_PART_HEAP_CACHE] = (struct k_mem_partition){
		.start = start,
		.size = size,
		.attr = K_MEM_PARTITION_P_RW_U_RW | XTENSA_MMU_CACHED_WB,
	};
	/* Host mailbox partition for additional IPC parameters: read-only */
	kobj->mpart[SOF_DP_PART_CFG] = (struct k_mem_partition){
		.start = (uintptr_t)sys_cache_uncached_ptr_get((void *)MAILBOX_HOSTBOX_BASE),
		.size = MAILBOX_HOSTBOX_SIZE,
		.attr = K_MEM_PARTITION_P_RO_U_RO,
	};
	kobj->mpart[SOF_DP_PART_CFG_CACHE] = (struct k_mem_partition){
		.start = (uintptr_t)MAILBOX_HOSTBOX_BASE,
		.size = MAILBOX_HOSTBOX_SIZE,
		.attr = K_MEM_PARTITION_P_RO_U_RO | XTENSA_MMU_CACHED_WB,
	};

	for (pidx = 0; pidx < SOF_DP_PART_TYPE_COUNT; pidx++) {
		ret = k_mem_domain_add_partition(mdom, kobj->mpart + pidx);
		if (ret < 0)
			goto e_dom;
	}

	memcpy(pdata->mpart, kobj->mpart, sizeof(pdata->mpart));

	ret = llext_manager_add_domain(comp_id, mdom);
	if (ret < 0) {
		tr_err(&dp_tr, "failed to add LLEXT to domain %d", ret);
		goto e_dom;
	}

	/*
	 * Keep this call last, able to fail, otherwise domain will be removed
	 * before its thread
	 */
	ret = k_mem_domain_add_thread(mdom, thread_id);
	if (ret < 0) {
		tr_err(&dp_tr, "failed to add thread to domain %d", ret);
		goto e_dom;
	}

	/* publish the record before the thread can run */
	dp_task_kobj_add(kobj);

	/* start the thread, it should immediately stop at the semaphore */
	k_event_init(kobj->event);
	k_thread_start(thread_id);

	/* ptask points into task_memory, so only publish it once it cannot be freed */
	*task = ptask;

	return 0;

e_dom:
	scheduler_dp_domain_free(mod, mdom, kobj->mpart, comp_id);
	on_pool = false;
e_thread:
	if (on_pool)
		dp_mdom_free(mdom);
	k_thread_abort(thread_id);
	k_object_free(kobj->thread);
e_event:
	k_object_free(kobj->event);
e_config:
	mod_free(mod, ipc_config_data);
e_kobj:
	rfree(kobj);
e_stack:
	user_stack_free(p_stack);
e_tmem:
	/* the copy lives in task_memory, so stop pointing at it before freeing */
	mod->dev->drv = drv;
	mod_free(mod, task_memory);
	return ret;
}

/*
 * Get the objects needed to send an IPC to the DP thread of @mod from the
 * kernel-only record. Returns -EINVAL if @mod has no DP task, -ENODEV if its
 * DP thread has faulted.
 */
int scheduler_dp_task_ipc_params(const struct processing_module *mod, struct k_event **event,
				 struct ipc4_flat **flat, const void **config_data,
				 unsigned int *core)
{
	struct scheduler_dp_task_kobj kobj;

	if (!dp_task_kobj_peek_mod(mod, &kobj))
		return -EINVAL;

	if (kobj.state != DP_KOBJ_ALIVE)
		return -ENODEV;

	*event = kobj.event;
	*flat = kobj.flat;
	*config_data = kobj.config_data;
	*core = kobj.core;

	return 0;
}

int z_impl_scheduler_dp_kick(const struct scheduler_dp_kick *kicks, uint32_t count)
{
	struct scheduler_dp_task_kobj *kobj;
	unsigned int core = cpu_get_id();
	uint32_t now = k_cycle_get_32();
	k_spinlock_key_t key;
	int skipped = 0;
	uint32_t i;

	/* the record cannot be freed while dp_task_kobj_lock is held */
	key = k_spin_lock(&dp_task_kobj_lock);

	for (i = 0; i < count; i++) {
		kobj = dp_task_kobj_find(kicks[i].task);
		/* a faulted DP thread is dead, its objects are about to be freed */
		if (!kobj || kobj->core != core || kobj->state != DP_KOBJ_ALIVE) {
			skipped++;
			continue;
		}

		/*
		 * Overflows are OK. The deadline may be in the past, Zephyr
		 * still schedules the thread with the earliest deadline first.
		 */
		if (kicks[i].flags & SCHEDULER_DP_KICK_DEADLINE)
			k_thread_absolute_deadline_set(kobj->thread, now + kicks[i].deadline);

		if (kicks[i].flags & SCHEDULER_DP_KICK_PROCESS)
			k_event_post(kobj->event, DP_TASK_EVENT_PROCESS);
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	return skipped;
}

unsigned int scheduler_dp_task_count(unsigned int core)
{
	k_spinlock_key_t key = k_spin_lock(&dp_task_kobj_lock);
	struct scheduler_dp_task_kobj *kobj;
	struct list_item *item;
	unsigned int count = 0;

	/*
	 * Task memory is only freed after its record is unlinked, so it can be
	 * read here. Only the state value is used, no pointers from it: a
	 * forged state only changes whether that task is counted.
	 */
	list_for_item(item, &dp_task_kobj_list) {
		kobj = container_of(item, struct scheduler_dp_task_kobj, list);
		if (kobj->core == core && kobj->state == DP_KOBJ_ALIVE &&
		    task_is_active(kobj->task))
			count++;
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	return count;
}

/*
 * DP fault recovery
 *
 * When a DP thread faults, k_sys_fatal_error_handler() calls
 * scheduler_dp_thread_faulted() and z_fatal_error() then aborts the thread.
 * scheduler_dp_thread_faulted() only marks the record of the thread faulted and
 * queues it on the recovery work queue of its core. The work then runs the task
 * part of the module delete path, see scheduler_dp_task_free(), on the dead
 * thread: it removes the task from the DP scheduler, joins the thread, frees
 * its stack, its kernel objects, its memory domain, the task memory and the
 * record. The module itself stays until the host deletes it. Its IPCs then
 * don't reach a DP thread any more, see scheduler_dp_thread_ipc().
 */

/* A faulted DP thread is aborted before the fatal error handler returns */
#define DP_FAULT_JOIN_TIMEOUT_MS	10

/* DP scheduler data of the calling core, kernel context */
static struct scheduler_dp_data *dp_fault_sch_get(void)
{
#if CONFIG_SOF_USERSPACE_LL
	/* the kernel-only copy, user threads can write the published one */
	struct schedulers **schedulers = arch_user_schedulers_get();

	return schedulers ? scheduler_list_get_data(*schedulers, SOF_SCHEDULE_DP) : NULL;
#else
	return scheduler_get_data(SOF_SCHEDULE_DP);
#endif
}

/*
 * Whether @node can be a neighbour of the task of the @faulted record in the
 * DP task list of @dp_sch: the list head, or the list item of another DP task
 * of the same core. The list items are in task memory, which user threads can
 * write, so they are checked before the kernel writes through them. Caller
 * holds dp_task_kobj_lock.
 */
static bool dp_fault_list_node_valid(struct scheduler_dp_data *dp_sch,
				     const struct scheduler_dp_task_kobj *faulted,
				     const struct list_item *node)
{
	struct scheduler_dp_task_kobj *kobj;
	struct list_item *item;

	if (node == &dp_sch->tasks)
		return true;

	list_for_item(item, &dp_task_kobj_list) {
		kobj = container_of(item, struct scheduler_dp_task_kobj, list);
		if (kobj != faulted && kobj->core == faulted->core &&
		    node == &kobj->task->list)
			return true;
	}

	return false;
}

/*
 * Remove the faulted task from the DP scheduler, like scheduler_dp_task_stop().
 *
 * Unlike scheduler_dp_task_stop(), this doesn't cancel the DP tick source when
 * the last DP task of the core is removed. That takes the LL scheduler lock,
 * while the LL thread takes dp_lock with the LL scheduler lock held when it
 * starts a DP task from a pipeline trigger: taking both here, asynchronously
 * to IPC processing, could deadlock. The tick source then keeps running a
 * dummy LL task until the next DP task of the core is stopped, scheduling it
 * again for a new DP task is harmless.
 */
static void dp_fault_task_unlink(struct scheduler_dp_task_kobj *kobj)
{
	struct scheduler_dp_data *dp_sch = dp_fault_sch_get();
	struct list_item *tlist = &kobj->task->list;
	struct list_item *next, *prev;
	unsigned int lock_key;
	k_spinlock_key_t key;

	if (!dp_sch) {
		LOG_ERR("comp %#x: no DP scheduler on core %u", kobj->comp_id, kobj->core);
		return;
	}

	lock_key = scheduler_dp_lock(kobj->core);

	key = k_spin_lock(&dp_task_kobj_lock);

	next = tlist->next;
	prev = tlist->prev;

	/* linked to itself if not scheduled */
	if (next != tlist || prev != tlist) {
		if (dp_fault_list_node_valid(dp_sch, kobj, next) &&
		    dp_fault_list_node_valid(dp_sch, kobj, prev) &&
		    next->prev == tlist && prev->next == tlist)
			list_item_del(tlist);
		else
			LOG_ERR("comp %#x: DP task list corrupted, task not removed",
				kobj->comp_id);
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	kobj->task->state = SOF_TASK_STATE_CANCEL;

	scheduler_dp_unlock(lock_key);
}

/*
 * Free the DP task of a faulted DP thread. Called with the DP task freeing
 * lock of its core held, after the record was taken off the faulted list.
 */
static void dp_fault_task_free(struct scheduler_dp_task_kobj *kobj)
{
	struct task *task = kobj->task;
	struct comp_dev *dev = kobj->dev;
	uint32_t comp_id = kobj->comp_id;
	int ret;

	/* 1. stop scheduling the task */
	dp_fault_task_unlink(kobj);

	/*
	 * 2. scheduler_dp_task_stop() sets DP_TASK_EVENT_CANCEL here to make a
	 * live DP thread exit, a dead one doesn't need it.
	 *
	 * 3. the thread was aborted before the fatal error handler returned, so
	 * this returns at once. Required before freeing the thread and its stack.
	 */
	ret = k_thread_join(kobj->thread, K_MSEC(DP_FAULT_JOIN_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("comp %#x: faulted DP thread still alive (%d), aborting", comp_id, ret);
		k_thread_abort(kobj->thread);
	}

	/* 4. the stack, from the record: pdata->p_stack is user-writable */
	ret = user_stack_free(kobj->stack);
	if (ret < 0)
		LOG_ERR("comp %#x: DP thread stack free failed %d", comp_id, ret);
	kobj->stack = NULL;

	/*
	 * 5. leave the module without a DP task, before anything the IPC thread
	 * could still use is freed. The IPC thread can preempt this work: from
	 * here on its DP thread IPCs return without using the task, and the
	 * module delete path skips the task, see scheduler_dp_thread_ipc() and
	 * pipeline_comp_dp_task_free().
	 */
	dev->task = NULL;

	/*
	 * 6. the thread and event objects, the memory domain, then the task
	 * memory, after restoring mod->dev->drv, which points to the driver copy
	 * in it, and the record.
	 */
	if (scheduler_dp_internal_free_kobj(task, NULL))
		LOG_INF("comp %#x: faulted DP task freed", comp_id);
	else
		LOG_ERR("comp %#x: faulted DP task record not found", comp_id);
}

static void dp_fault_work_fn(struct k_work *work)
{
	struct dp_fault_wq *wq = CONTAINER_OF(work, struct dp_fault_wq, work);
	struct scheduler_dp_task_kobj *kobj;
	k_spinlock_key_t key;

	do {
		/*
		 * Serialized with the module delete path, see
		 * pipeline_comp_dp_task_free(): whichever frees the task first,
		 * the other one doesn't find it any more. Take the record off
		 * the faulted list only with the lock held, so it can't be
		 * freed by the delete path meanwhile.
		 */
		scheduler_dp_free_lock(wq->core);

		key = k_spin_lock(&dp_task_kobj_lock);
		kobj = list_is_empty(&wq->faulted) ? NULL :
			list_first_item(&wq->faulted, struct scheduler_dp_task_kobj, faulted);
		if (kobj)
			list_item_del(&kobj->faulted);
		k_spin_unlock(&dp_task_kobj_lock, key);

		if (kobj)
			dp_fault_task_free(kobj);

		scheduler_dp_free_unlock(wq->core);
	} while (kobj);
}

int scheduler_dp_fault_wq_init(void)
{
	const struct k_work_queue_config cfg = {
		.name = "dp_fault",
	};
	unsigned int core = cpu_get_id();
	struct dp_fault_wq *wq;
	k_spinlock_key_t key;
	k_tid_t thread;
	int ret;

	if (dp_fault_wq[core])
		return 0;

	wq = rzalloc(SOF_MEM_FLAG_KERNEL | SOF_MEM_FLAG_COHERENT, sizeof(*wq));
	if (!wq)
		return -ENOMEM;

	wq->core = core;
	list_init(&wq->faulted);
	k_work_init(&wq->work, dp_fault_work_fn);

	/* above the DP threads, below LL, IPC and IDC */
	k_work_queue_init(&wq->q);
	k_work_queue_start(&wq->q, dp_fault_wq_stack[core],
			   K_KERNEL_STACK_SIZEOF(dp_fault_wq_stack[core]),
			   CONFIG_DP_THREAD_PRIORITY - 1, &cfg);

	/* the work must run on the faulting core, see dp_fault_sch_get() */
	thread = k_work_queue_thread_get(&wq->q);
	k_thread_suspend(thread);
#ifdef CONFIG_SCHED_CPU_MASK
	ret = k_thread_cpu_pin(thread, core);
#else
	ret = 0;
#endif
	k_thread_resume(thread);

	if (ret < 0) {
		/* leave the queue idle, DP faults on this core won't be recovered */
		LOG_ERR("DP fault work queue pin to core %u failed %d", core, ret);
		return ret;
	}

	/* publish last, scheduler_dp_thread_faulted() reads it */
	key = k_spin_lock(&dp_task_kobj_lock);
	dp_fault_wq[core] = wq;
	k_spin_unlock(&dp_task_kobj_lock, key);

	return 0;
}

bool scheduler_dp_thread_faulted(k_tid_t thread)
{
	struct scheduler_dp_task_kobj *kobj;
	struct dp_fault_wq *wq = NULL;
	bool first_fault = false;
	k_spinlock_key_t key;
	uint32_t comp_id = 0;
	bool dp_thread;

	/*
	 * Exception context on the faulting core. Other cores keep creating,
	 * deleting and kicking DP tasks, so the record list is only walked
	 * with dp_task_kobj_lock held. This core can't already hold it: the
	 * fault came from user mode, and the kernel only holds the lock with
	 * interrupts locked, outside of user code. Nothing is freed here, the
	 * thread and its stack are still in use until z_fatal_error() aborts it.
	 */
	key = k_spin_lock(&dp_task_kobj_lock);

	kobj = dp_task_kobj_find_thread(thread);
	dp_thread = kobj != NULL;
	if (kobj && kobj->state == DP_KOBJ_ALIVE) {
		first_fault = true;
		comp_id = kobj->comp_id;
		wq = dp_fault_wq[kobj->core];
		if (wq) {
			kobj->state = DP_KOBJ_FAULTED;
			list_item_append(&kobj->faulted, &wq->faulted);
		}
	}

	k_spin_unlock(&dp_task_kobj_lock, key);

	if (!first_fault)
		return dp_thread;

	if (wq) {
		/*
		 * Interrupts are locked here, so this doesn't reschedule. The
		 * queue is pinned to this core: the work only runs after the
		 * exception has returned, when the thread has been aborted.
		 */
		k_work_submit_to_queue(&wq->q, &wq->work);
		LOG_ERR("DP thread %p of comp %#x faulted, freeing its DP task", thread, comp_id);
	} else {
		/* the delete path still frees it, the DP thread IPCs just time out */
		LOG_ERR("DP thread %p of comp %#x faulted, no recovery work queue",
			thread, comp_id);
	}

	return true;
}

#if CONFIG_SOF_BOOT_TEST_STANDALONE
/* Add a DP task record for zephyr/test/userspace/test_dp_kick.c */
int scheduler_dp_test_record_add(struct task *task, struct k_event *event,
				 struct k_thread *thread, uint16_t core)
{
	struct scheduler_dp_task_kobj *kobj = rzalloc(SOF_MEM_FLAG_KERNEL | SOF_MEM_FLAG_COHERENT,
						      sizeof(*kobj));

	if (!kobj)
		return -ENOMEM;

	kobj->task = task;
	kobj->event = event;
	kobj->thread = thread;
	kobj->core = core;
	list_init(&kobj->faulted);
	dp_task_kobj_add(kobj);

	return 0;
}

void scheduler_dp_test_record_del(struct task *task)
{
	rfree(dp_task_kobj_take(task, NULL));
}
#endif

#ifdef CONFIG_USERSPACE
#include <zephyr/internal/syscall_handler.h>
#include <sof/userspace/syscall_caller.h>

/*
 * mod_free() follows the module's resource and allocator pointers in
 * supervisor mode. They are user-writable, so at least check that they
 * point to memory the caller can access.
 */
static void scheduler_dp_mod_vrfy(struct processing_module *mod)
{
	K_OOPS(K_SYSCALL_MEMORY_WRITE(mod, sizeof(*mod)));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(mod->dev, sizeof(*mod->dev)));
	K_OOPS(K_SYSCALL_MEMORY_READ(mod->dev->drv, sizeof(*mod->dev->drv)));

	struct mod_alloc_ctx *alloc = mod->priv.resources.alloc;

	K_OOPS(K_SYSCALL_MEMORY_WRITE(alloc, sizeof(*alloc)));
	if (alloc->heap) {
		size_t h_size = 0;
		uintptr_t h_start;

		mod_heap_info(mod, &h_size, &h_start);
		if (h_size)
			K_OOPS(K_SYSCALL_MEMORY_WRITE(h_start, h_size));
	}
	if (alloc->vreg) {
		size_t vr_size = 0;
		uintptr_t vr_start;

		vregion_mem_info(alloc->vreg, &vr_size, &vr_start);
		K_OOPS(K_SYSCALL_MEMORY_WRITE(vr_start, vr_size));
	}
}

void z_vrfy_scheduler_dp_internal_free(struct task *task)
{
	struct scheduler_dp_task_kobj kobj;

	SOF_SYSCALL_CALLER_CHECK(SOF_SYSUSER_INFRA);

	/*
	 * Do not read task->priv_data: the kernel objects come from the
	 * kernel-only record, and an unknown task pointer is rejected.
	 */
	K_OOPS(!dp_task_kobj_peek(task, &kobj));

	/*
	 * A DP thread holds its own thread and event objects, so it would pass
	 * the checks below for its own task. Freeing would then abort the
	 * caller after the record is unlinked, leaking everything it holds.
	 */
	K_OOPS(kobj.thread == k_current_get());

	/*
	 * The caller must have been granted the task's objects. Check before
	 * unlinking the record, so an unauthorized call leaves it intact.
	 */
	K_OOPS(K_SYSCALL_OBJ(kobj.event, K_OBJ_EVENT));
	K_OOPS(K_SYSCALL_OBJ_INIT(kobj.thread, K_OBJ_THREAD));
	scheduler_dp_mod_vrfy(kobj.mod);

	K_OOPS(!scheduler_dp_internal_free_kobj(task, &kobj));
}
#include <zephyr/syscalls/scheduler_dp_internal_free_mrsh.c>

int z_vrfy_scheduler_dp_kick(const struct scheduler_dp_kick *kicks, uint32_t count)
{
	struct scheduler_dp_kick kkicks[SCHEDULER_DP_KICK_MAX];

	/* without user-space LL, only the kernel LL scheduler kicks DP tasks */
	K_OOPS(!IS_ENABLED(CONFIG_SOF_USERSPACE_LL));
	SOF_SYSCALL_CALLER_CHECK(SOF_SYSUSER_LL);

	K_OOPS(K_SYSCALL_VERIFY(count <= ARRAY_SIZE(kkicks)));
	K_OOPS(k_usermode_from_copy(kkicks, kicks, count * sizeof(kkicks[0])));

	return z_impl_scheduler_dp_kick(kkicks, count);
}
#include <zephyr/syscalls/scheduler_dp_kick_mrsh.c>
#endif /* CONFIG_USERSPACE */
