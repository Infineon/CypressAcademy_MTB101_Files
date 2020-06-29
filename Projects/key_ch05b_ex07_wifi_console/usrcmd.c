/**
 * @file usrcmd.c
 * @author CuBeatSystems
 * @author Shinichiro Nakamura
 * @copyright
 * ===============================================================
 * Natural Tiny Shell (NT-Shell) Version 0.3.1
 * ===============================================================
 * Copyright (c) 2010-2016 Shinichiro Nakamura
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */


#include "ntopt.h"
#include "ntlibc.h"
#include "ntshell.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "networkTask.h"
#include <stdlib.h>

extern ntshell_t ntshell;

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_clear(int argc, char **argv);
static int usrcmd_printargs(int argc, char **argv);
static int usrcmd_scan(int argc, char **argv);
static int usrcmd_connect(int argc, char **argv);
static int usrcmd_disconnect(int argc, char **argv);
static int usrcmd_print(int argc, char **argv);


typedef struct {
    char *cmd;
    char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
    { "clear", "Clear the screen", usrcmd_clear },
    { "printargs","print the list of arguments", usrcmd_printargs},
	{ "scan", "Start a WiFi scan an print the results - usage: scan [on|off]",usrcmd_scan},
	{ "connect", "Connect to an AP - usage: connect SSID [Password]",usrcmd_connect},
	{ "disconnect", "Disconnect from the AP",usrcmd_disconnect},
	{ "print", "Print IP address or MAC address - usage: print ip|mac",usrcmd_print},
};

int usrcmd_execute(const char *text)
{
    return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (unsigned int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    printf("%s","Unknown command found.\n");
    return 0;
}

static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (unsigned int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        printf("%s",p->cmd);
        printf("%s","\t:");
        printf("%s",p->desc);
        printf("%s","\n");
        p++;
    }
    return 0;
}


static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
        printf("%s","info sys\n");
        printf("%s","info ver\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
        printf("%s","PSoC 6 MBED Monitor\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
        printf("%s","Version 0.0.0\n");
        return 0;
    }
    printf("%s","Unknown sub command found\n");
    return -1;
}


static int usrcmd_clear(int argc, char **argv)
{
    vtsend_erase_display_home(&ntshell.vtsend);
    return 0;
}

static int usrcmd_printargs(int argc, char **argv)
{
    printf("ARGC = %d\n",argc);

    for(int i =0;i<argc;i++)
    {
        printf("argv[%d] = %s\n",i,argv[i]);
    }
    return 0;

}

static int usrcmd_scan(int argc, char **argv)
{
	static int state=0;
	if(argc==1)
	{
		state = (state ==0)?1:0;
	}
	else if(argc == 2)
	{
		if(strcmp(argv[1],"on") == 0)
		{
			state=1;
		}
		else if(strcmp(argv[1],"off") == 0)
		{
			state = 0;
		}
		else
		{
			printf("usage: scan [on|off]\n");
			return 0;
		}
	}
	else
	{
		printf("usage: scan [on|off]\n");
		return 0;
	}
	networkQueueMsg_t msg;
	msg.cmd = net_scan;
	msg.val0 = state;
	xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	return 0;
}

static int usrcmd_connect(int argc, char **argv)
{
	networkQueueMsg_t msg;

	if(argc == 2)
	{
		msg.val0 = (uint32_t)malloc(strlen(argv[1])+1);
		msg.val1 = (uint32_t)malloc(sizeof(""));
		strcpy((char *)msg.val0,argv[1]);
		strcpy((char *)msg.val1,"");
		msg.cmd = net_connect;
		xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	}
	else if(argc == 3)
	{
		msg.val0 = (uint32_t)malloc(strlen(argv[1])+1);
		msg.val1 = (uint32_t)malloc(strlen(argv[2])+1);
		strcpy((char *)msg.val0,argv[1]);
		strcpy((char *)msg.val1,argv[2]);
		msg.cmd = net_connect;
		xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	}
	else
	{
		printf("usage: connect SSID [Password]\n");
	}

	return 0;
}

static int usrcmd_disconnect(int argc, char **argv)
{
	networkQueueMsg_t msg;
	msg.cmd = net_disconnect;
	xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	return 0;
}

static int usrcmd_print(int argc, char **argv)
{
	networkQueueMsg_t msg;
	if(argc == 2 && strcmp(argv[1],"ip")==0)
	{
		msg.cmd = net_printip;
		xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	}
	else if(argc == 2 && strcmp(argv[1],"mac")==0)
	{
		msg.cmd = net_printmac;
		xQueueSend(networkQueue,(const void *)&msg,portMAX_DELAY);
	}
	else
	{
		printf("usage: print ip|mac\n");
	}
	return 0;
}
