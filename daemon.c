//Example to execute a program periodical in a daemon
#include<stdio.h>
#include<time.h>

main()
{
	struct tm *ptr;
	time_t lt;
	int i = 48;
	init_daemon();
	
	for(;;)
	{
		lt = time(NULL);
		ptr = localtime(&lt);
		if(i == ptr->tm_min)
		{
			system("clear");
			printf("Daemon running\n");
			sleep(10);
		}
	}
}

// Code to creat a daemon
init_daemon(void)		
{
	int pid;
	if(!fork())
	{
		setsid();// Disassociate the process from the tt and 
		chdir("/");//make it the process group leader
		umask(0);
//		return(0);
	}
	else
	exit(0);
}
		
