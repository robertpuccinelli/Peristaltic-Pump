

void	FPPA0 (void)
{
	.ADJUST_IC	SYSCLK=IHRC/4		//	SYSCLK=IHRC/4



	while (1)
	{

	}
}


void	Interrupt (void)
{
	pushaf;

	if (Intrq.T16)
	{
	}

	popaf;
}

