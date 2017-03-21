/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "ble/BLE.h"
#include "math.h"

DigitalOut led1(LED1, 1);
Ticker     ticker;
Serial pc(USBTX,USBRX);
void periodicCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */
}


void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) { //advertises all the present values written inside this function like rssi adress etc

  //  printf("Adv peerAddr: [%02x %02x %02x %02x %02x %02x] rssi %d, ScanResp: %u, AdvType: %u\r\n",
   //      params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0],
       //    params->rssi, params->isScanResponse, params->type);
     
               if(params->peerAddr[2]==0x3A && params->peerAddr[1]==0x0B && params->peerAddr[0]==0x28)
               
       {
             pc.printf("BeaconID peerAddr:[%02x %02x %02x %02x %02x %02x] rssi= %d \r\n", params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0], params->rssi);
             
             //for (unsigned index = 0; index < params->advertisingDataLen; index++) {
                 int txPower=params->advertisingData[29];
                 int rssi = params->rssi;
                 
                 
                 
                 txPower=-52;
                 float rssiq=rssi;
                 pc.printf("txpower rssi %d \r\n",txPower,params->rssi);
                 float dist;
                 float a=txPower - rssiq;
                 float b=36;
                 float c =a/b;
                 dist = pow(10,c);
                 pc.printf("distance= %f",dist);  
                
                
             
 
  

    printf("\r\n");
    
  
   } 
        
 
 /* DUMP_ADV_DATA */

}
/**
 * This function is called when the ble initialization process has failed
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }
 
    ble.gap().setScanParams(1 /* scan interval */, 1000 /* scan window */);
    ble.gap().startScan(advertisementCallback);
}

int main(void)
{
    ticker.attach(periodicCallback, 1);

    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);

    while (true) {
        ble.waitForEvent();
    }
}
