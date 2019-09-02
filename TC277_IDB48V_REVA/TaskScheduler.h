
//****************************************************************************
// @Macros
//****************************************************************************

#define SYSTEM_TICK					(100U)		// [usec]
#define	SYSTEM_TICK_TIMER_CLOCK		(100U)		// [MHz] e.g GTM_FIXED_CLOCK0 is used : 100MHz(fsys)
#define SYSTEM_TICK_TIMER_PERIOD	(SYSTEM_TICK * SYSTEM_TICK_TIMER_CLOCK)

#define EPB_operation	1

//****************************************************************************
// @Typedefs
//****************************************************************************
typedef union
{
	unsigned int	U;
	struct
	{
		unsigned int	Enable			:16;	// Task scheduler enable bit
		unsigned int	TickCount		:16;	// Software counter for system tick
		unsigned int	Tick			:16;	// Selected System tick
		unsigned int	TaskRun			:16;	// Current running task
		unsigned int	TaskMax			:16;	// Maximum repeated time task
	}B;
} TASK_CONTROL;


typedef enum
{
	TASK_100us	= 1,
	TASK_200us	= 2,
	TASK_1ms	= 10,
	TASK_5ms	= 50,
	TASK_10ms	= 100,
	TASK_20ms	= 200,
	TASK_50ms	= 500,
	TASK_100ms	= 1000,
	TASK_1s		= 10000,
	TASK_5s		= 50000
} NO_OF_TICK_COUNT_FOR_TASK;




//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************
extern 	void TaskScheduler_Initialization_Core0(unsigned int );
extern	void TaskScheduler_ActivateTask_Core0(void);
//extern	uint16 uTesflagPWM;
//extern	uint8	uTesflagMotor;



