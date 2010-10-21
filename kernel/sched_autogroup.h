#ifdef CONFIG_SCHED_AUTOGROUP
static inline void
autogroup_check_attach(struct task_struct *p, struct task_group **tg);
#else
static inline void
autogroup_check_attach(struct task_struct *p, struct task_group **tg) { }
#endif
