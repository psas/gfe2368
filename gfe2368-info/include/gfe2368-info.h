

/*
 * gfe2368-info.h
 */

typedef struct gfe2368_info {
    char*	boardtag ;
} gfe2368_info_s ;

extern gfe2368_info_s brd;

char*   infoquery_boardtag(void) ;

