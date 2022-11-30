#include "main_cpp.hpp" 
//MPU9250 mpu;
char printData[50];


void MainCpp::setup()
{
float acc_resolution {0.f}; 
  //// calibrate anytime you want to
  //  NRF_LOG_INFO("Accel Gyro calibration will start in 5sec.\r\n");
  //  NRF_LOG_INFO("Please leave the device still on the flat plane.\r\n");
  //  mpu.verbose(true);
  //  nrf_delay_ms(5000);
  //  mpu.calibrateAccelGyro();

  //  NRF_LOG_INFO("Mag calibration will start in 5sec.\r\n");
  //  NRF_LOG_INFO("Please Wave device in a figure eight until done.\r\n");
  //  nrf_delay_ms(5000);
  //  mpu.calibrateMag();

  //  mpu.verbose(false);
}

void MainCpp::loop()
{
    //if (mpu.update()) {
    //   uint16_t len;
    //   len+=sprintf(&printData[len],"%0.2,f0.2f,0.2f,0.2f,0.2f,0.2f,", mpu.getAccX(),mpu.getAccY(),mpu.getAccZ(),mpu.getYaw(),mpu.getPitch(),mpu.getRoll());
    //}
}

extern "C" MainCpp * create_MainCpp()
{
    return new MainCpp;
}

extern "C" void MainCpp_setup(MainCpp* pMainCpp) // wrapper function
{
    return pMainCpp->setup();
}

extern "C" void MainCpp_loop(MainCpp* pMainCpp) // wrapper function
{
    return pMainCpp->loop();
}