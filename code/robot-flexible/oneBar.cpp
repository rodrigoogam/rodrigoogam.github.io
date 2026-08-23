#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <826api.h>  //Copy to /usr/include
//install driver make and the install
#include<vector>
#include <fstream>
#include <algorithm>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <eigen3/Eigen/Dense>

#include <zmq.hpp>

#include "shared_data.hpp"
#include <fcntl.h>
#include <sys/mman.h>

// for print error message
#include <string.h>
#include <errno.h>


using namespace std;
using namespace std::chrono;
using namespace zmq;
using namespace cv;
using namespace Eigen;
//Controller 1

const double pi = 3.14159265358979323846;
double Amplitud = 25;
double period = 6;
double tf = 60;
double h = 0.008;
int steps = tf/h;
int cc = 875;
double Cal = 0;
vector<double> q1(steps);
vector<double> q1p(steps);
vector<double> q1pF(steps);
vector<double> u1(steps);
vector<double> crl(steps);
vector<double> e1(steps);
vector<double> e2(steps);
vector<double> eHat(steps);
vector<double> de(steps);
vector<double> deAcc(steps);
vector<double> rew(steps);
vector<double> e1p(steps);
vector<double> ie1(steps);
vector<double> r1(steps);
vector<double> r1t(steps);
vector<double> accx(steps);
vector<double> accy(steps);
vector<double> accz(steps);
vector<double> accxF(steps);
vector<double> accxFT(steps);
vector<double> accxD(steps);
vector<double> curr(steps);
vector<double> currF(steps);
vector<double> Cacc(cc);
vector<double> CaccF(cc);
vector<double> eAcc(steps);
vector<double> accxDF(steps);
vector<double> accyF(steps);
vector<double> acczF(steps);
vector<double> x(steps);
vector<double> y(steps);
vector<double> yPred(steps);
vector<double> t1(steps);
vector<double> t2(steps);
vector<double> xD(steps);
vector<double> yD(steps);
vector<double> ex(steps);
vector<double> ey(steps);

vector<double> t(steps);

vector<int> c(steps);
double Integral1 = 0;
double Integral2 = 0;
int k = 0;
string modo = "";

vector<double> posi(2);
vector<double> p = {0,0};
vector<double> x_pos(steps);
vector<double> y_pos(steps);
vector <double> a = {1,-1.9822,0.9824};
vector <double> b = {1.0e-4*0.3913,1.0e-4*0.7826,1.0e-4*0.3913};
vector <double> s = {0,0};
const char* name = "/my_shared_mem";
int fd = shm_open(name, O_RDONLY, 0666);
auto* shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ, MAP_SHARED, fd, 0));

class LowPassFilter {
    private:
        double alpha;
        double prev_output;
    
    public:
        // Constructor: alpha in (0, 1). Closer to 1 = less smoothing.
        LowPassFilter(double alpha_init) : alpha(alpha_init), prev_output(0.0) {}
    
        double filter(double input) {
            double output = alpha * input + (1.0 - alpha) * prev_output;
            prev_output = output;
            return output;
        }
    
        void reset(double initial_output = 0.0) {
            prev_output = initial_output;
        }
    };


// Configuration constants.
#define TSETTLE 7 // Settling delay after switching AIN (adjust as necessary).
#define SLOTFLAGS 0xFFFF // Timeslot flags: use all 16 timeslots.
int adcbuf[16]; // Sample buffer -- always set size=16 (even if fewer samples needed)
// Configure all timeslots: 1 slot per AIN; constant settling time for all slots.

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array

//Error handling
#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Counter utility functions used by the demos.

#define PWM_MODE  (S826_CM_K_50MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_BP_BOTH | S826_CM_OM_PRELOAD)

//#define PWM_MODE 0x01682020

uint board      = 0;                        // change this if you want to use other than board number 0
int errcode     = S826_ERR_OK;	

static int PwmGeneratorStart(uint board, uint counter, uint ontime, uint offtime)
{
    int errcode;
    X826( S826_CounterStateWrite(board, counter, 0)                         );  // halt counter channel if it's running
    X826( S826_CounterModeWrite(board, counter, PWM_MODE)                   );  // configure counter as pwm generator
    X826( S826_CounterSnapshotConfigWrite(board, counter, 0, S826_BITWRITE) );  // don't need counter snapshots -- we're just outputting pwm signal
    X826( S826_CounterPreloadWrite(board, counter, 0, offtime)              );  // program pwm on-time in microseconds
    X826( S826_CounterPreloadWrite(board, counter, 1, ontime)               );  // program pwm off-time in microseconds
    X826( S826_CounterStateWrite(board, counter, 1)                         );  // start pwm generator
    return errcode;
}

static int RouteCounterOutput(uint board, uint ctr, uint dio)
{
uint data[2]; // dio routing mask
if ((dio >= S826_NUM_DIO) || (ctr >= S826_NUM_COUNT))
return S826_ERR_VALUE; // bad channel number
if ((dio & 7) != ctr)
return S826_ERR_VALUE; // counter output can't be routed to dio
// Route counter output to DIO pin:
S826_SafeWrenWrite(board, S826_SAFEN_SWE); // Enable writes to DIO signal router.
S826_DioOutputSourceRead(board, data); // Route counter output to DIO
data[dio > 23] |= (1 << (dio % 24)); // without altering other routes.
S826_DioOutputSourceWrite(board, data);
return S826_SafeWrenWrite(board, S826_SAFEN_SWD); // Disable writes to DIO signal router.
}

static int JamCounts(uint board, uint ctr, uint value){
    int errcode;
    X826( S826_CounterStateWrite(board, ctr, 0));
    X826(S826_CounterPreloadWrite(board, ctr, 0, value)); // Write value to preload0 register.
    X826(S826_CounterPreload(board, ctr, 1, 0)); // Copy preload0 to counter.
    X826( S826_CounterStateWrite(board, ctr, 1));
    return errcode;
}

static int setupEncoder(uint board, uint ctr){
    int errcode;
    X826(S826_CounterModeWrite(board, ctr, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface.
    X826(JamCounts(board, ctr, 0));
    X826(S826_CounterStateWrite(board, ctr, 1));
    return errcode;
}


double getEncoder(uint board, uint ctr){
    uint counts; // encoder counts when the snapshot was captured
    uint timestamp; // time the snapshot was captured
    S826_CounterSnapshot(board, ctr); // Trigger snapshot on counter 0.
    S826_CounterSnapshotRead(board, ctr, &counts, &timestamp, NULL,0);
    int pulsos =(int) counts;
    double grados = (pulsos)/(73185.0/360.0);
    return grados;
}

double getC(uint board, uint ctr){
    uint counts; // encoder counts when the snapshot was captured
    uint timestamp; // time the snapshot was captured
    S826_CounterSnapshot(board, ctr); // Trigger snapshot on counter 0.
    S826_CounterSnapshotRead(board, ctr, &counts, &timestamp, NULL,0);
    int pulsos =(int) counts;
    return pulsos;
}


void stop(){
    uint maskA[] = {48, 0}; // bitmask for DIOs 4-5
    S826_DioOutputWrite(0, maskA, S826_BITCLR); // Enciende DIOS 4-5;
}


int setPWM(uint board, uint ctr,int duty,int sat){
    if (sat == 1000) sat = 999;
    duty = duty >= sat ? sat:duty;  //saturate duty cycle positive 100%
    duty = duty <= -sat ? -sat:duty; //saturate duty cycle to negative 100%
    if(ctr ==2){
        if (duty<0){
            uint maskA[] = {16, 0}; // bitmask for DIO 4
            uint maskB[] = {32, 0}; // bitmask for DIO 5
            S826_DioOutputWrite(board, maskA, S826_BITCLR); // enciende DIO 4;
            S826_DioOutputWrite(board, maskB, S826_BITSET); // Apaga DIO 5;
        }
        else if (duty>0){
            uint maskA[] = {32, 0}; // bitmask for DIOs 5
            uint maskB[] = {16, 0}; // bitmask for DIOs 4
            S826_DioOutputWrite(board, maskA, S826_BITCLR); // enciende DIO 5;
            S826_DioOutputWrite(board, maskB, S826_BITSET); // Apaga DIO 4;
        }
        else{
            uint maskA[] = {48, 0}; // bitmask for DIOs 4-5
            S826_DioOutputWrite(board, maskA, S826_BITSET); // apaga DIOS 4-5;
        }
    }else if(ctr ==3){
        if (duty<0){
            uint maskA[] = {64, 0}; // bitmask for DIOs 6
            uint maskB[] = {128, 0}; // bitmask for DIOs 7
            S826_DioOutputWrite(board, maskA, S826_BITCLR); // enciende DIO 6;
            S826_DioOutputWrite(board, maskB, S826_BITSET); // Apaga DIO 7;
        }
        else if (duty>0){
            uint maskA[] = {128, 0}; // bitmask for DIOs 7
            uint maskB[] = {64, 0}; // bitmask for DIOs 6
            S826_DioOutputWrite(board, maskA, S826_BITCLR); // enciende DIO 7;
            S826_DioOutputWrite(board, maskB, S826_BITSET); // Apaga DIO 6;
        }
        else{
            uint maskA[] = {192, 0}; // bitmask for DIOs 6-7
            S826_DioOutputWrite(board, maskA, S826_BITSET); // apaga DIOS 6-7;
        }
    }
    int dc = abs(duty*2.5);
    PwmGeneratorStart(board,ctr,dc,2500-dc);
    return duty;
}


double toVoltage(int adc){
    return adc*5.0/32767;
}

double toCurrent(int adc){
    return (toVoltage(adc)-2.5)*-2;
}

double toMotorVoltage(int adc){
    return toVoltage(adc)*24/2.3;
}

double normali(double x_1){
    return fmod(abs((x_1 + pi)),(2 * pi)) - pi;
}


double filter(double un, double fill){
    double salida = (10*un-10*fill)*0.003333+fill;
    return salida; 
}

double filter2(double un, double fill){
    double salida = (2*un-2*fill)*0.003333+fill;
    return salida; 
}

void filterACC(){
    accxF[k] = b[0]*accx[k]+b[1]*accx[k-1]+b[2]*accx[k-2]-a[1]*accxF[k-1]-a[2]*accxF[k-2];
    accyF[k] = b[0]*accy[k]+b[1]*accy[k-1]+b[2]*accy[k-2]-a[1]*accyF[k-1]-a[2]*accyF[k-2];
    acczF[k] = b[0]*accz[k]+b[1]*accz[k-1]+b[2]*accz[k-2]-a[1]*acczF[k-1]-a[2]*acczF[k-2];
    accxDF[k] = b[0]*accxD[k]+b[1]*accxD[k-1]+b[2]*accxD[k-2]-a[1]*accxDF[k-1]-a[2]*accxDF[k-2];
}

void filterCur(){
    currF[k] = b[0]*curr[k]+b[1]*curr[k-1]+b[2]*curr[k-2]-a[1]*currF[k-1]-a[2]*currF[k-2];
}

vector<double> calculatePower(vector<double> v1,vector<double> v2,vector<double> i){
    int vsize = v1.size();
    vector<double> power;
    for (int n = 0; n<vsize;n++){
        double voltage = v1[n]-v2[n];
        double p = abs(voltage*i[n]);
        power.push_back(p);
    }
    return power;
}

void zeros(){

    uint dios[] = {0xFFFFFFFF,0x00000000};
    uint data[] = {0x00000000,0x00000000};
    S826_SafeWrenWrite(0, S826_SAFEN_SWE);
    S826_DioOutputSourceWrite(0, data);
    S826_DioOutputWrite(0, dios, S826_BITWRITE); // Turn off all dios
    S826_SafeWrenWrite(1, S826_SAFEN_SWE);
    S826_DioOutputSourceWrite(1, data);
    S826_DioOutputWrite(1, dios, S826_BITWRITE); // Turn off all dios
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


double PID1(){
    double P = 100;
    double I = 0;
    double D = 0;
    double control;
    yPred[k] = atan2(y_pos[k],x_pos[k])*180/pi-atan2(y_pos[0],x_pos[0])*180/pi;
    Integral1 += e1[k];
    if(k==0){
        control = P*e1[k]+I*h*Integral1;
    }
    else{
        control = P*e1[k]+I*h*Integral1+D/h*(e1[k]-e1[k-1]);
    }
    return control;
}




vector<double> traj5(double ini,double fin,double time){
    VectorXd b {{ini,fin,0,0,0,0}};
    MatrixXd A {{1,0,0,0,0,0},
    {1,time,pow(time,2),pow(time,3),pow(time,4),pow(time,5)},
    {0,1,0,0,0,0},
    {0,1,2*time,3*pow(time,2),4*pow(time,3),5*pow(time,4)},
    {0,0,2,0,0,0},
    {0,0,2,6*time,12*pow(time,2),20*pow(time,3)}};
    VectorXd x= A.inverse()*b;
    int s = int(time);
    vector<double> pos(s);
    for(int j=0;j<time;j++){
        pos[j] = x(0)+x(1)*j+x(2)*pow(j,2)+x(3)*pow(j,3)+x(4)*pow(j,4)+x(5)*pow(j,5);
    }
    return pos;
}
vector<double> concatena(vector<double> t1, vector<double> t2){
    int vsize = t1.size();
    vector<double> t3;
    for (int n = 0; n<(2*vsize);n++){
        if(n<vsize){
        t3.push_back(t1[n]);
        }else{
        t3.push_back(t2[n%vsize]);
        }
    }
    return t3;
}
void generaReferencia(){
    for(int i = 0; i<steps; i++){
        double ti = (i)*h;
        r1[i] = 30; // SetPoint
        accxD[i]=0;
        xD[i] = 0.815*cos((r1[i]+68.8)*pi/180); 
        yD[i] = 0.815*sin((r1[i]+68.8)*pi/180);
        }
}

void generatePRBS(){
    double amplitud1 = 0;
    int ancho1 = 0;
    double amplitud2 = 0;
    int ancho2 = 0;
    int max = 100;
    for(int i = 0; i< steps; i++){
    if(ancho1<=0){
        amplitud1 = 2*max*((rand() % 10000) / 10000.0)-max;
        ancho1 = (rand() % 300*10000) / 10000.0;
    }
    if(ancho2<=0){
        amplitud2 = 2*max*((rand() % 10000) / 10000.0)-max;
        ancho2 = (rand() % 300*10000) / 10000.0;
    }
    r1[i]= amplitud1;
    ancho1--;
    ancho2--;
    }
}

void calibrateAcc(){
    uint remaining = SLOTFLAGS; // timeslots that have not yet been read
    uint slotlist = remaining; // Read all available remaining timeslots.
    for(int j=0;j<cc;j++){ 
        do {
        int errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 0); // note: tmax=0
        remaining &= ~slotlist;
        // OPTIONAL: NEWLY ARRIVED SAMPLES MAY BE PROCESSED WHILE WAITING FOR REMAINING SAMPLES
        } while (errcode == S826_ERR_NOTREADY);
        if (errcode != S826_ERR_OK){
            cout<<"error"<<endl;
        }
        Cacc[j] = (toVoltage(adcbuf[0]&0xFFFF)*-9.81/0.3);
        CaccF[j] = b[0]*Cacc[j]+b[1]*Cacc[j-1]+b[2]*Cacc[j-2]-a[1]*CaccF[j-1]-a[2]*CaccF[j-2];

        this_thread::sleep_for(chrono::milliseconds(8));
        posi[0] = shared->xcoord;
        posi[1] = shared->ycoord; 
    }
    double suma = 0;
    for(int s = cc-2; s>cc-200; s--){
        suma+=CaccF[s];
    }
    Cal = ((suma/200)-0.45);
    cout<<Cal<<endl;
}

void plot() {
    ofstream myfile("./data.csv");
    myfile  << "q1[n]"<<","<<"u1[n]"<<","<<"r1[n]"<<","
            << "accx[n]"<<","<<"accy[n]"<<","<<"accz[n]"<<","
            << "accxF[n]"<<","<<"accyF[n]"<<","<<"acczF[n]"<<","
            << "accxD[n]"<<","<<"accxDF[n]"<<","<<"eAcc[n]"<<","
            << "Cacc[n]"<<","<<"CaccF[n]"<<","<< "x[n]"<<","<<"y[n]"<<","
            << "xD[n]"<<","<<"yD[n]"<<","<<"crl[n]"<<","<<"curr[n]"<<","
            << "r1t[n]"<<","<< "yPred[n]"<<","
            <<"t"<<endl;

    int vsize = q1.size();
    for (int n = 0; n<vsize;n++){
        myfile  <<q1[n]<<","<<u1[n]<<","<<r1[n]<<","
                <<accx[n]<<","<<accy[n]<<","<<accz[n]<<","
                <<accxFT[n]<<","<<accyF[n]<<","<<acczF[n]<<","
                <<accxD[n]<<","<<accxDF[n]<<","<<eAcc[n]<<","
                <<Cacc[n]<<","<<CaccF[n]<<","<<x_pos[n]<<","<<y_pos[n]<<","
                <<xD[n]<<","<<yD[n]<<","<<crl[n]<<","<<currF[n]<<","
                << r1t[n]<<","<< yPred[n]<<","
                <<t[n]<<endl;
    }

    myfile.close();
    system("python3 /home/ubuntu/Desktop/VigaFlexible/OneLinkControlPID/control/plots.py");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.

int main(int argc, char **argv){
    const char* name = "/my_shared_mem";
    int fd = shm_open(name, O_RDONLY, 0666);
    auto* shared = static_cast<SharedData*>(mmap(NULL, sizeof(SharedData), PROT_READ, MAP_SHARED, fd, 0));
    close(fd);

    board      = 0;                        // change this if you want to use other than board number 0
    errcode     = S826_ERR_OK;	
    int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    if (argc > 1)
        board = atoi(argv[1]);

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        printf("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            }
        }
    } else  {
        // Execute the demo functions. Uncomment any functions you want to run.
         // Configure counter0 as PWM.
        cout<<"Iniciando"<<endl;
        X826(setupEncoder(0,0));
        X826(PwmGeneratorStart(0,2,0,0));
        X826(RouteCounterOutput(0,2,2));

        X826(setupEncoder(0,1));
        X826(PwmGeneratorStart(0,3,0,0));
        X826(RouteCounterOutput(0,3,11));
        //generatePRBS();
        //ADC Setup
        for (int i = 0; i < 16; i++) S826_AdcSlotConfigWrite(board, i, i, TSETTLE, S826_ADC_GAIN_2);
        // Configure adc system and start it running.
        S826_AdcSlotlistWrite(board, SLOTFLAGS, S826_BITWRITE); // Enable all 16 timeslots.
        //S826_AdcTrigModeWrite(board, 0); // Select free-running mode.
        S826_AdcEnableWrite(board, 1); // Start conversions.
        context_t context;
        socket_t socket (context, ZMQ_REP);
        socket.bind("tcp://127.0.0.1:5555");
        calibrateAcc();
        generaReferencia();
        auto start = high_resolution_clock::now();

        double gio;

        while(true){
            x_pos[k] = shared->xcoord-posi[0]+0.295;
            y_pos[k] = shared->ycoord-posi[1]+0.76;
            //STEP
                double p1 = getEncoder(0,0); //Lee el encoder
                q1[k] = p1; //Actualiza el vector de posciones
                e1[k] = (r1[k]-q1[k]); //Error
                if(k<2){
                    ie1[k] = 0;
                    gio =1000;
                
                }else{
                    ie1[k] = ie1[k-1]+h*e1[k];
                    gio=0;
                }
                
                
                
                crl[k] = 0;//std::stod(control); 
                u1[k] = setPWM(0,2,gio,gio);//Mandar a llamar el PID,recive el PID,satura el PWM
        
                t[k] = h*k; //Tiempo transcurrido
            uint remaining = SLOTFLAGS; // timeslots that have not yet been read
            do {
            uint slotlist = remaining; // Read all available remaining timeslots.
            int errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 0); // note: tmax=0
            remaining &= ~slotlist;
            // OPTIONAL: NEWLY ARRIVED SAMPLES MAY BE PROCESSED WHILE WAITING FOR REMAINING SAMPLES
            } while (errcode == S826_ERR_NOTREADY);
            if (errcode != S826_ERR_OK){
                cout<<"error"<<endl;
                break;
            }
                accx[k] = (toVoltage(adcbuf[0]&0xFFFF)*-9.81/0.3); //Filtros de aceleración
                accy[k] = (toVoltage(adcbuf[1]&0xFFFF)*9.81/0.3);//Filtros de aceleración
                accz[k] = (toVoltage(adcbuf[2]&0xFFFF)*9.81/0.3);//Filtros de aceleración
                curr[k] = toVoltage(adcbuf[3]&0xFFFF); //Filtro de la corriente
                if(k>=2){
                    filterACC();
                }else{
                    cout<<CaccF[cc-2+k]<<endl;
                    accxF[k] = CaccF[cc-2+k];

                    accxDF[0] = b[0]*accxD[0];
                    accxDF[1] = b[0]*accxD[1]+b[1]*accxD[0]+b[2]*0-a[1]*accxDF[0]-a[2]*0;

                }
                accxFT[k]=accxF[k]-Cal;
                curr[k] = curr[k]-2.55;
                filterCur();
                k++;

            if(k>=steps) break; //Condición del final 
            this_thread::sleep_for(chrono::microseconds(8000));
        }
        shm_unlink("/my_shared_mem");

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end-start);
        cout<< duration.count()<<endl;
        cout<<"Finalizando"<<endl;
        X826(JamCounts(0, 0, 0));
        X826(JamCounts(0, 1, 0));
        zeros();
    }

    switch (errcode)
    {
    case S826_ERR_OK:           break;
    case S826_ERR_BOARD:        printf("Illegal board number"); break;
    case S826_ERR_VALUE:        printf("Illegal argument"); break;
    case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
    case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
    case S826_ERR_DRIVER:       printf("Driver call failed"); break;
    case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
    case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;
    case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
    case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
    case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
    default:                    printf("Unknown error"); break;
    }
    S826_SystemClose();
    plot();
    return 0;
}
