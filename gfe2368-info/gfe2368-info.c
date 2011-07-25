

/*
 * gfe2368-info.c
 */

#include <stdint.h>
#include "lpc23xx.h"

#include "gfe2368-info.h"

#ifdef BOARD_ONE
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_one" };
#elif BOARD_TWO
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_two" };
#elif BOARD_THREE
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_three" };
#elif BOARD_FOUR
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_four" };
#elif BOARD_FIVE
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_five" };
#elif BOARD_SIX
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_six" };
#elif BOARD_SEVEN
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_seven" };
#elif BOARD_EIGHT
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_eight" };
#elif BOARD_NINE
   gfe2368_info brd __attribute__ ((unused)) = { .boardtag = "board_nine" };
#else
   gfe2368_info_s brd __attribute__ ((unused)) = { .boardtag = "undefined" };
#endif

/*
 * infoquery_board
 */
char*    infoquery_boardtag(void) {
    return(brd.boardtag);
}

