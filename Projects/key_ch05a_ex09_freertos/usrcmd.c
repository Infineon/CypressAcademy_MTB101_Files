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
#include <stdlib.h>

#include "ntshell.h"
#include "ntlibc.h"
#include "psoc6_ntshell_port.h"

#include "FreeRTOS.h"
#include "task.h"

static ntshell_t ntshell;

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_clear(int argc, char **argv);
static int usrcmd_pargs(int argc, char **argv);
static int usrcmd_ledrate(int argc, char **argv);

uint ledrate = 100;

#ifdef configUSE_TRACE_FACILITY
#if configUSE_STATS_FORMATTING_FUNCTIONS ==1
static int usrcmd_list(int argc, char **argv);
#endif
#endif

typedef struct {
    char *cmd;
    char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
    { "clear", "Clear the screen", usrcmd_clear },
    { "pargs","print the list of arguments", usrcmd_pargs},
	{ "ledrate","Change the LED blink rate to the specified rate in ms. Usage: ledrate <rate> ", usrcmd_ledrate},
#ifdef configUSE_TRACE_FACILITY 
#if configUSE_STATS_FORMATTING_FUNCTIONS ==1
    { "tasks","print the list of RTOS Tasks", usrcmd_list},
#endif
#endif
};


void usrcmd_task()
{

  setvbuf(stdin, NULL, _IONBF, 0);
  printf("Started user command task with NT Shell\n");
  ntshell_init(
	       &ntshell,
	       ntshell_read,
	       ntshell_write,
	       ntshell_callback,
	       (void *)&ntshell);
  ntshell_set_prompt(&ntshell, "AnyCloud> ");
  vtsend_erase_display(&ntshell.vtsend);
  ntshell_execute(&ntshell);
}

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

static int usrcmd_pargs(int argc, char **argv)
{
    printf("ARGC = %d\n",argc);

    for(int i =0;i<argc;i++)
    {
        printf("argv[%d] = %s\n",i,argv[i]);
    }
    return 0;

}

static int usrcmd_ledrate(int argc, char **argv)
{
	if(argc > 1)
	{
		ledrate = atoi(argv[1]);
	}
	else
	{
		printf("You must provide an LED blink rate in ms\n");
	}
	return 0;
}

#ifdef configUSE_TRACE_FACILITY
#if configUSE_STATS_FORMATTING_FUNCTIONS ==1
static int usrcmd_list(int argc,char **argv)
{
    // 40 bytes/task + some margin
    char buff[40*10 + 100];

    vTaskList( buff );
    printf("Name          State Priority   Stack  Num\n");
    printf("------------------------------------------\n");
    printf("%s",buff);

    printf("‘B’ – Blocked\n‘R’ – Ready\n‘D’ – Deleted (waiting clean up)\n‘S’ – Suspended, or Blocked without a timeout\n");
    printf("Stack = bytes free at highwater\n");
    return 0;
}
#endif
#endif
