#ifdef CONFIG_SCHED_AUTOGROUP
static inline void
autogroup_task_group(struct task_struct *p, struct task_group **tg);
static void __sched_move_task(struct task_struct *tsk, struct rq *rq);
#else /* !CONFIG_SCHED_AUTOGROUP */
static inline void autogroup_init(struct task_struct *init_task) {  }
static inline void autogroup_fork(struct task_struct *p) { }
static inline void
autogroup_task_group(struct task_struct *p, struct task_group **tg) { }
#endif /* CONFIG_SCHED_AUTOGROUP */
