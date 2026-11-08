// Format: returnType functionName();


double getLiftHeight();
bool liftDown();

// void cascadeToHeight(double height){
//   int direction;
//     if(getLiftHeight()<height){
//       direction =1;
//     }else{
//       direction =-1;
//     }
//     while(fabs(getLiftHeight()-height)){
//         cascade.spin(fwd,12*direction,volt);
//     }
// }

extern double alliance;
extern double neutral;
extern double midfield;

extern double cup;
extern double pin;

// extern void cascadeToHeight(double height);
// extern void cascadeToHeight(double pinCount, double cupCount, std::string goal, double buffer = 2);
// extern void cascadePlusHeight(double height);

void exampleAuton();
void exampleAuton2();
void tunePid();
void autonOne();
void liftPID_tuner();
void fullLiftTest();
void brainD();
void NA_1pin();
void A_1pin();
void qual1();
void skills();

void qual1_lift();

void qual1_wrist();

void printText(const char* text);
void release();

double clamp(double value, double min, double max);
//blue
void BA_2pin();
void BA_3pin();
void BAL_3pin();
//red
