#ifndef TRUE
#define TRUE	1
#endif

//int diag_printf(char *fmt, ...);


/*****************************************
 * Convert 'a' - 'z' into 'A' - 'Z'
 *****************************************/
char    UpperChar(char ch)
{
        if ((ch >= 'a') && (ch <= 'z')){
                ch -= ('a' - 'A');
        }
        return ch;
}


/***********************************************************
 * Convert to Hex number (etc. 0 .. 9 and A .. F or a .. f)
 * return 0xff with error input
 * others return 0 .. 16
 ************************************************************/
int     IsHexNumber(char ch)
{
        ch = UpperChar(ch);

        if ((ch >= '0') && (ch <= '9'))
                return (ch - '0');

        if ((ch >= 'A') && (ch <= 'F'))
                return (ch - 'A' + 10);

        return 0xff;
}



unsigned int sd_dump_default_addr = 0x80000000;

int dump_mem(unsigned int addr, int len)
{
unsigned char   *raw,ch;
int                     row,col,rowsz;

        raw = (unsigned char *)addr;
        if (len == 0) {
                rowsz = 20;
        }
        else {
                rowsz = (len + 15)/16;
        }
        for (row=0;row<rowsz;row++)
        {
                // Address
                //sdprintf("%08X: ",(addr + row * 16));
                diag_printf("%08X: ",(addr + row * 16));

                // Show HEX
                for (col=0;col<8;col++) {
                        //sdprintf("%02X ",raw[col]);
                        diag_printf("%02X ",raw[col]);
                }
                //sdprintf("- ");
                diag_printf("- ");
                for (col=8;col<16;col++) {
                        //sdprintf("%02X ",raw[col]);
                        diag_printf("%02X ",raw[col]);
                }
                // Show ASCII
                for (col=0;col<16;col++) {
                        if ((raw[col] < 0x20) || (raw[col] > 0x7e)) {
                                ch = '.';
                        }
                        else {
                                if ((raw[col] == 0x25) || (raw[col] == 0x5c))
                                        ch = '.';
                                else
                                        ch = raw[col];
                        }
                        //sdprintf("%c",ch);
                        diag_printf("%c",ch);
                }
                raw += 16;


                // Display
                //sdprintf("\r\n");
                diag_printf("\r\n");

        }       // end of for
        //sdisp_flush();
        return TRUE;
}


unsigned long str2UL(char *str) 
{
	unsigned long	ul;
	unsigned char	ch;
	int				i;
	
	ul = 0;
	if ((str[0] == '0' && str[1] == 'X') ||
		(str[0] == '0' && str[1] == 'x'))
	{
		// Hex Mode
		for (i=2;i<strlen(str);i++)	{			
			ch = IsHexNumber(str[i]);
			if (ch == 0xff)	{ /* illegal char. */
				break;
			}			
			ul <<= 4;
			ul |= ch;
		}
	}
	else {
		// DEC Mode
		for (i=0;i<strlen(str);i++)	{
		
			ch = str[i];				
			if (ch < '0' || ch > '9')	{ /* illegal char. */
				break;
			}			
			ul *= 10;
			ul += (ch - '0');
		}			
	}
	
	return ul;
}

#define  ASCII_CR    0x0d
#define  ASCII_LF    0x0a
#define  ASCII_BS    0x08
#define  ASCII_BELL  0x07
#define  ASCII_TAB   0x09
#define	 ASCII_ESC   0x1b
#define	 ASCII_SPACE 0x20
static char ch_buf[256];
static int buf_idx = 0;
static int last_buf_idx = 0;
int kbd_proc(void)
{
    char kbd_cc = 0;

    //kbd_cc=getc();
    hal_if_diag_read_char(&kbd_cc);

    switch(kbd_cc) {
    case 0:
    	/* get nothing */
    	break;
#if 0
    case ASCII_ESC:
    	start = Timer_Get();
		while(!(kbd_cc=getc())&&!timeout(start,10));
		if(kbd_cc==0x5b)
		{
	    	start = Timer_Get();
			while(!(kbd_cc=getc())&&!timeout(start,10));
    		if(kbd_cc==0x41)
    		{
    			if(buf_idx == 0)
    			{
	    			for(i=0; i<last_buf_idx; i++)
    					hal_if_diag_write_char(ch_buf[i]);
    				buf_idx = last_buf_idx;
    			}
    		}
    	}
    	break;
#endif
	case ASCII_LF:
	case ASCII_CR:
		ch_buf[buf_idx]='\0';
		last_buf_idx = buf_idx;
		buf_idx = 0;
		hal_if_diag_write_char(ASCII_LF);
		hal_if_diag_write_char(ASCII_CR);
		return(1);
	case ASCII_TAB:
		ch_buf[buf_idx]=ASCII_SPACE;
	    if(buf_idx<sizeof(ch_buf))
		{
			buf_idx++;
			hal_if_diag_write_char(ASCII_SPACE);
		}
		else 
			hal_if_diag_write_char(ASCII_BELL);
		break;
	case ASCII_BS:
		if(buf_idx>0) {
			buf_idx--;
			hal_if_diag_write_char(ASCII_BS);
			hal_if_diag_write_char(ASCII_SPACE);
			hal_if_diag_write_char(ASCII_BS);
		}
		break;
	default:
		ch_buf[buf_idx]=kbd_cc;
	    if(buf_idx<sizeof(ch_buf))
	    {
			buf_idx++;
			hal_if_diag_write_char(kbd_cc);
		}
		else 
			hal_if_diag_write_char(ASCII_BELL);
    }
	return 0;
}


#define MAX_ARGS	20
#define MAX_ARG_LEN	20
int cmd_proc(void)
{
unsigned int	i;
char *argv[MAX_ARGS];
int	argc = 0;
static char argv_value[MAX_ARGS][MAX_ARG_LEN+1];
int arg_idx = 0;

        /* We need make 'argv' a really array of point of char. */
        for(i=0; i < MAX_ARGS; i++){
            argv[i] =  argv_value[i];
        }

	for(i=0; ch_buf[i]!='\0'; i++)
	{
		if(ch_buf[i]==' '){
			argv[argc][arg_idx]='\0';
			argc++;
			arg_idx=0;
		}
		else {
			if(arg_idx<MAX_ARG_LEN)
			{	
				argv[argc][arg_idx]=ch_buf[i];
				arg_idx++;
			}
		}
	}
	argv[argc][arg_idx]='\0';

	if(!strcmp(argv[0], "r"))
	{
		//if(str2UL(argv[1])%4)
		//	diag_printf("Address must be multiple of 4");
		//else
			diag_printf("[%08lx] = %08lx\n", str2UL(argv[1]), *((unsigned long *)str2UL(argv[1])));
	}
	else		
	if(!strcmp(argv[0], "w"))
	{	
		//if(str2UL(argv[1])%4)
		//	diag_printf("Address must be multiple of 4");
		//else
			*((unsigned long *)str2UL(argv[1])) = str2UL(argv[2]);
	}
	else
	if(!strcmp(argv[0], "d"))
	{
	unsigned long daddr;
	int dlen;

		if(strlen(argv[1]))	// address field present
			daddr = str2UL(argv[1]);
	
		else
			daddr = sd_dump_default_addr;
		
		if(strlen(argv[2]))	// len field present
		{
			dlen = str2UL(argv[2]);
			if(dlen>256)
				dlen = 256;
		}
		else
			dlen = 128;

		dump_mem(daddr, dlen);
		sd_dump_default_addr = daddr + dlen;
	}
	else if(!strcmp(argv[0], "time"))
	{
	  //	diag_printf("System Time = %ld\n", Timer_Get());
	}
	else if(!strcmp(argv[0], "download"))
	{
	  Download();
	}
	else if(!strcmp(argv[0], "assign"))
	{
	  Assign();
	}
	else if(!strcmp(argv[0], "reboot"))
	{
	  reset();
	}	
	else if(!strcmp(argv[0], "ping"))
	{
	  do_ping(argc,argv);
	}	
	else if(!strcmp(argv[0], "ip_address"))
	{

	  do_ip_addr(argc,argv);
        }
        else if(!strcmp(argv[0], "load"))
        {
            do_load(argc, argv);
        }
        else if(!strcmp(argv[0], "exec"))
        {
            do_exec_cmd(argc, argv);
        }
        else if(!strcmp(argv[0], "fcopy"))
        {
            do_flash_copy(argc, argv);
        }
	
	memset(argv, 0, sizeof(argv));
	diag_printf("<test#>");
	return TRUE;
}

