#ifndef TMCL_H
#define TMCL_H

	#include "tmc/helpers/API_Header.h"

    void txTest(uint8 ch);
    void dispInt(int value);
    void dispString(char *pstr);



	void tmcl_init();
	void tmcl_process();
	void tmcl_boot();

#endif /* TMCL_H */
