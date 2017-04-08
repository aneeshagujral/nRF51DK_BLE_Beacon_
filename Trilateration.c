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
#include "Matrix.h" 
#include "MatrixMath.h"
#include <arm_math.h>
extern "C" void mbed_reset();
Matrix A(3,3);
Matrix B(3,1);
Matrix X(3,1);
DigitalOut led1(LED1, 1);
Ticker     ticker;
Serial pc(USBTX,USBRX);
DigitalOut reset(p1);
float dist,dist1,dist2;
double r1,r2,r3,q1,q2,q3,w1,w2,w3;
double x1=0,y11=0,x2=0,y2=1,x3=1,y3=1;
int u1,u2,u3;
void periodicCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */
}


void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) { //advertises all the present values written inside this function like rssi adress etc

  //  printf("Adv peerAddr: [%02x %02x %02x %02x %02x %02x] rssi %d, ScanResp: %u, AdvType: %u\r\n",
   //      params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0],
       //    params->rssi, params->isScanResponse, params->type);
               if(params->peerAddr[2]==0x01 && params->peerAddr[1]==0xD4 && params->peerAddr[0]==0x1D)
               {
                 //  pc.printf("1 beacons \n");
                  // pc.printf("BeaconID peerAddr:[%02x %02x %02x %02x %02x %02x] rssi= %d \r\n", params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0], params->rssi);
             
             //for (unsigned index = 0; index < params->advertisingDataLen; index++) {
                    int txPower1=params->advertisingData[29];
                    int rssi1 = params->rssi;
                 
                 
                 
                    txPower1=-50;
                    float rssiq=rssi1;
                    //pc.printf("txpower rssi %d \r\n",txPower1,params->rssi);
                    
                    float d=txPower1 - rssiq;
                    float b=36;
                    float e =d/b;
                    
                    dist = pow(10,e);
                    
                    pc.printf("distance beacon1= %f \r\n",dist);
                    u1=2;
                }
                   else if(params->peerAddr[2]==0x4E && params->peerAddr[1]==0xAC && params->peerAddr[0]==0x91)
                   {
                       //pc.printf("2 beacons\n");
                       
                        int txPower2=params->advertisingData[29];
                        int rssi2 = params->rssi;
                        
                        txPower2=-50;
                        float rssiq=rssi2;
                       // pc.printf("txpower rssi %d \r\n",txPower2,params->rssi);
                        
                        float a=txPower2 - rssiq;
                        float b=36;
                        float c =a/b;
                        
                        dist1 = pow(10,c);
                        
                        pc.printf("distance beacon2= %f \r\n",dist1);
                        u2=2;
                    }
                       else if(params->peerAddr[2]==0xB7 && params->peerAddr[1]==0x82 && params->peerAddr[0]==0xB4)
                       {
                           
                           //pc.printf("3 beacons\n");
                           //pc.printf("BeaconID peerAddr:[%02x %02x %02x %02x %02x %02x] rssi= %d \r\n", params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0], params->rssi);
             
             //for (unsigned index = 0; index < params->advertisingDataLen; index++) {
                            int txPower3=params->advertisingData[29];
                            int rssi3 = params->rssi;
                 
                 
                 
                            txPower3=-50;
                            float rssiq=rssi3;
                            //pc.printf("txpower rssi %d \r\n",txPower3,params->rssi);
                            
                            float z=txPower3 - rssiq;
                            float b=36;
                            float p =z/b;
                 
                            dist2 = pow(10,p);
                            pc.printf("distance beacon 3= %f \r\n",dist2);
                            
                            u3=2;}
                           
                        else if(u1==2&&u2==2&u3==2)     //as we need 3beacons for scanning so when taken data from 3 beacons then it will do trilateration
                        {
                            pc.printf("done\r\n");
                            
                            u1=0;
                            u2=0;
                            u3=0;
                            r1=pow(dist,2);
                            r2=pow(dist1,2);
                            r3=pow(dist2,2);
                            q1=pow(x1,2);  //x1 square
                            pc.printf("here %f",q1);
                            q2=pow(x2,2);  //x2 square
                            //pc.printf("q2 %f",q2);
                            q3=pow(x3,2);  //x3 square
                            //pc.printf("q3 %f",q3);
                            w1=pow(y11,2); //y1 square
                            //pc.printf("w1 %f",w1);
                            w2=pow(y2,2);  //y2 square
                            //pc.printf("w2 %f",w2);
                            w3=pow(y3,2);  //y3 square
                            
                            B << r1-q1-w1      //B matrix is known
                              << r2-q2-w2
                              << r3-q3-w3;
                            
                            A << 1<< -2*x1 << -2*y11
                              << 1<< -2*x2 << -2*y2
                              << 1 << -2*x3<< -2*y3;        //A matrix is known
                  
            
                            Matrix myInv = MatrixMath::Inv( A );  
                
                            X=myInv*B;
                            pc.printf("Robot\r\n");
                            X.print();
                            NVIC_SystemReset();    //System reset is done as it gets stuck calculating the matrix           
                            
                        }
               
               
               
       
              
                 
                                                  
               
                
             
                        else
                    {
  
                pc.printf("Still detecting");
                
                } 
                        
                
   } 
        
 
 /* DUMP_ADV_DATA */



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
