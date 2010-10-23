#ifdef CONFIG_SCHED_AUTOGROUP
#include <linux/tty.h>

unsigned int __read_mostly sysctl_sched_autogroup_enabled = 1;

void sched_autogroup_create_tty(struct tty_struct *tty)
{
	tty->tg = sched_create_group(&init_task_group);
	if (IS_ERR(tty->tg)) {
		tty->tg = &init_task_group;
		 WARN_ON(1);
	}
}
EXPORT_SYMBOL(sched_autogroup_create_tty);

void sched_autogroup_destroy_tty(struct tty_struct *tty)
{
	if (tty->tg && tty->tg != &init_task_group)
		sched_destroy_group(tty->tg);
}
EXPORT_SYMBOL(sched_autogroup_destroy_tty);

static void
autogroup_attach_tty(struct task_struct *p, struct task_group **tg)
{
	struct tty_struct *tty = p->signal->tty;

	if (!tty)
		return;

	*tg = p->signal->tty->tg;
}

static inline void
autogroup_check_attach(struct task_struct *p, struct task_group **tg)
{
	if (!sysctl_sched_autogroup_enabled || *tg != &root_task_group ||
			p->sched_class != &fair_sched_class)
		return;

	rcu_read_lock();

	autogroup_attach_tty(p, tg);

	rcu_read_unlock();
}

int sched_autogroup_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct task_struct *p, *t;
	struct task_group *tg;
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		return ret;

	for_each_process(p) {
		tg = task_group(p);
		sched_move_task(p);
		list_for_each_entry_rcu(t, &p->thread_group, thread_group) {
			sched_move_task(t);
		}
	}

	return 0;
}

static int __init setup_autogroup(char *str)
{
	sysctl_sched_autogroup_enabled = 0;

	return 1;
}
__setup("noautogroup", setup_autogroup);
#endif
