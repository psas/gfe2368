

/*
 * gfe2368-info.h
 */

#define MAX_BOARDTAG  20

typedef struct gfe2368_info {
    char	boardtag[MAX_BOARDTAG] ;
} gfe2368_info_s ;

extern gfe2368_info_s brd;

void          info_init();
const char*   infoquery_gfe_boardtag(void) ;

