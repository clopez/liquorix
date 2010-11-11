#ifdef CONFIG_SCHED_AUTOGROUP

unsigned int __read_mostly sysctl_sched_autogroup_enabled = 1;

struct autogroup {
	struct kref		kref;
	struct task_group	*tg;
};

static struct autogroup autogroup_default;

static void autogroup_init(struct task_struct *init_task)
{
	autogroup_default.tg = &init_task_group;
	kref_init(&autogroup_default.kref);
	init_task->autogroup = &autogroup_default;
}

static inline void autogroup_destroy(struct kref *kref)
{
	struct autogroup *ag = container_of(kref, struct autogroup, kref);

	sched_destroy_group(ag->tg);
	kfree(ag);
}

static inline void autogroup_kref_put(struct autogroup *ag)
{
	kref_put(&ag->kref, autogroup_destroy);
}

static inline struct autogroup *autogroup_kref_get(struct autogroup *ag)
{
	kref_get(&ag->kref);
	return ag;
}

static inline struct autogroup *autogroup_create(void)
{
	struct autogroup *ag = kmalloc(sizeof(*ag), GFP_KERNEL);

	if (!ag)
		goto out_fail;

	ag->tg = sched_create_group(&init_task_group);
	kref_init(&ag->kref);

	if (!(IS_ERR(ag->tg)))
		return ag;

out_fail:
	if (ag) {
		kfree(ag);
		WARN_ON(1);
	} else
		WARN_ON(1);

	return autogroup_kref_get(&autogroup_default);
}

static void autogroup_fork(struct task_struct *p)
{
	p->autogroup = autogroup_kref_get(current->autogroup);
}

static inline void
autogroup_task_group(struct task_struct *p, struct task_group **tg)
{
	int enabled = sysctl_sched_autogroup_enabled;

	enabled &= (*tg == &root_task_group);
	enabled &= (p->sched_class == &fair_sched_class);
	enabled &= (!(p->flags & PF_EXITING));

	if (enabled)
		*tg = p->autogroup->tg;
}

static void
autogroup_move_task(struct task_struct *p, struct autogroup *ag)
{
	struct autogroup *prev;
	struct rq *rq;
	unsigned long flags;

	rq = task_rq_lock(p, &flags);
	prev = p->autogroup;
	if (prev == ag) {
		task_rq_unlock(rq, &flags);
		return;
	}

	p->autogroup = autogroup_kref_get(ag);
	__sched_move_task(p, rq);
	task_rq_unlock(rq, &flags);

	autogroup_kref_put(prev);
}

void sched_autogroup_create_attach(struct task_struct *p)
{
	autogroup_move_task(p, autogroup_create());

	/*
	 * Correct freshly allocated group's refcount.
	 * Move takes a reference on destination, but
	 * create already initialized refcount to 1.
	 */
	if (p->autogroup != &autogroup_default)
		autogroup_kref_put(p->autogroup);
}
EXPORT_SYMBOL(sched_autogroup_create_attach);

void sched_autogroup_detatch(struct task_struct *p)
{
	autogroup_move_task(p, &autogroup_default);
}
EXPORT_SYMBOL(sched_autogroup_detatch);

void sched_autogroup_exit(struct task_struct *p)
{
	autogroup_kref_put(p->autogroup);
}

int sched_autogroup_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct task_struct *p, *t;
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		return ret;

	/*
	 * Exclude cgroup, task group and task create/destroy
	 * during global classification.
	 */
	cgroup_lock();
	spin_lock(&task_group_lock);
	read_lock(&tasklist_lock);

	do_each_thread(p, t) {
		sched_move_task(t);
	} while_each_thread(p, t);

	read_unlock(&tasklist_lock);
	spin_unlock(&task_group_lock);
	cgroup_unlock();

	return 0;
}

static int __init setup_autogroup(char *str)
{
	sysctl_sched_autogroup_enabled = 0;

	return 1;
}

__setup("noautogroup", setup_autogroup);
#endif
