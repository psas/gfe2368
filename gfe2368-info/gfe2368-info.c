

/*
 * gfe2368-info.c
 */

#include <stdint.h>
#include <string.h>

#include "lpc23xx.h"
#include "lpc23xx-uart.h"
#include "printf-lpc.h"

#include "gfe2368-info.h"

gfe2368_info_s __attribute__ ((unused)) brd ;

void info_init() {

#ifdef BOARD_ONE
    printf_lpc(UART0, "board_one\n");
    char *comment = "board one";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_TWO
    char *comment = "board two";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_THREE
    char *comment = "board three";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_FOUR
    char *comment = "board four";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_FIVE
    char *comment = "board five";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_SIX
    char *comment = "board six";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_SEVEN
    char *comment = "board seven";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#elif BOARD_EIGHT
    char *comment = "board eight";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';

#else
    char *comment = "unknown";
    strncpy(brd.boardtag, comment, strlen(comment));
    brd.boardtag[strlen(comment)] = '\0';
#endif

}

/*
 * infoquery_board
 * must call info_init first
 */
const char*    infoquery_gfe_boardtag(void) {
//    printf_lpc(UART0,"Return of boardtag: %s\n", brd.boardtag);
    return((const char*) brd.boardtag);
}

