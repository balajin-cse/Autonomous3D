#ifndef _HOTDEC_LQR_lSH_H_
#define _HOTDEC_LQR_lSh_H_


#include "HotDec_Structs.h"


class HotDec_Controller {
public:
	HotDec_Controller();
	~HotDec_Controller();

	void update(const hotdec_state_sp_t state_sp, const hotdec_plant_state_t state, hotdec_thrust_sp_t & thrust_sp);

private:


};

#endif