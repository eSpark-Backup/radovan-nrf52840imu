#ifdef __cplusplus 
#ifndef MAINCPP
#define MAINCPP

//#include "MPU9250.h"

class MainCpp
{   

    public:
       MainCpp() {}
       void setup();
       void loop();
   
    
};

#endif
#else
struct MainCpp* create_MainCpp();
void MainCpp_setup(struct MainCpp* pMainCpp); // wrapper function
void MainCpp_loop(struct MainCpp* pMainCpp); // wrapper function
#endif