#ifndef __CLI_H
#define __CLI_H

typedef void (*cmd_handler_t)(char *args);

typedef struct {
    const char *name;
    cmd_handler_t handler;
} command_t;

void process_command(char *cmd);

#endif
