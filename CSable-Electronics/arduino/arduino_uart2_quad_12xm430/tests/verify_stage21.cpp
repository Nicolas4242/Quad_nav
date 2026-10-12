// Host-only synthetic fixtures. Never use these values as robot calibration.
#include <cassert>
#include <cmath>
#include <cctype>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <tuple>
#include <functional>
#include <algorithm>
using std::isnan;
using std::isinf;
struct __FlashStringHelper {};
#define F(s) reinterpret_cast<const __FlashStringHelper*>(s)
const int LED_BUILTIN=13, OUTPUT=1, LOW=0, HIGH=1, UNIT_RAW=0, OP_POSITION=3;
#include "actuator.h"
using namespace DYNAMIXEL;
uint64_t clockUs=0;
unsigned long millis(){return (uint32_t)(clockUs/1000);}
unsigned long micros(){return (uint32_t)clockUs;}
void delay(unsigned long t){clockUs+=t*1000;}
void pinMode(int,int){}
void digitalWrite(int,int){}
struct FakeSerial {
  std::string output;
  std::deque<char> input;
  void begin(unsigned long){}
  void feed(const std::string &s){for(char c:s)input.push_back(c);}
  int available(){return (int)input.size();}
  char read(){char c=input.front();input.pop_front();return c;}
  void print(const __FlashStringHelper* x){output+=(const char*)x;}
  void print(uint8_t x){output+=std::to_string(x);}
  template<class T> void print(T x){std::ostringstream s;s<<x;output+=s.str();}
  template<class T> void print(T x,int){print(x);}
  template<class T> void println(T x){print(x);println();}
  template<class T> void println(T x,int p){print(x,p);println();}
  void println(){output+='\n';}
} Serial, Serial2;
struct FakeWire {void begin(){} void setClock(int){}} Wire;
namespace imu {template<int N> struct Vector {double x(){return 0;}double y(){return 0;}double z(){return 0;}};}
struct Adafruit_BNO055 {
  enum {VECTOR_ACCELEROMETER,VECTOR_GYROSCOPE};
  Adafruit_BNO055(int,int){} bool begin(){return true;} void setExtCrystalUse(bool){}
  imu::Vector<3> getVector(int){return {};}
};
using namespace ControlTableItem;
struct Event {int id,item;int32_t value;};
struct SerialPortHandler {};
const int D2A_LIB_ERROR_NULLPTR_PORT_HANDLER=1;
struct Dynamixel2Arduino {
  SerialPortHandler port, *p_dxl_port_=nullptr;
  void* getPort(){return &port;}
  void setLastLibErrCode(int e){libError=e;}
  int32_t readControlTableItem(uint16_t model,uint8_t item,uint8_t id,uint32_t timeout);
  std::vector<Event> transactions;
  std::function<void(int,int)> beforeRead;
  bool shortRead=false,lostWriteAck=false;
  uint32_t snapshotUs=1000;
  std::map<std::pair<int,int>,int32_t> regs;
  std::vector<Event> writes;
  int libError=0,packetError=0;
  int failReadId=-1,failReadItem=-1,failWriteId=-1,failWriteItem=-1;
  bool packetFailure=false,followGoal=true;
  int torqueJumpId=-1; int32_t torqueJump=0;
  std::function<void(Event)> afterWrite;
  Dynamixel2Arduino(FakeSerial&,int){}
  void begin(unsigned long){}
  bool setPortProtocolVersion(float v){assert(v==1.0f||v==2.0f);return true;}
  bool ping(int id){assert(id>=0&&id<=252);return true;}
  int getLastLibErrCode(){return libError;}
  int getLastStatusPacketError(){return packetError;}
  int32_t readControlTableItem(uint8_t item,uint8_t id){return readControlTableItem(XM430_W350,item,id,100);}
  int itemAt(int address){
    if(address==60)return STARTUP_CONFIGURATION;
    if(address==98)return BUS_WATCHDOG;
    for(int item=0;item<ControlTableItem::LAST_DUMMY_ITEM;item++){
      auto info=getControlTableItemInfo(XM430_W350,(uint8_t)item);
      if(info.addr_length&&info.addr==address)return item;
    }
    return -1;
  }
  int32_t read(uint8_t id,uint16_t address,uint16_t length,uint8_t* out,uint16_t cap,uint32_t timeout){
    assert(id<=252&&cap>=length);libError=packetError=0;
    int item=itemAt(address);
    transactions.push_back({id,item,-999});
    if(beforeRead)beforeRead(id,length==72?-72:item);
    clockUs+=(length==72?snapshotUs:100);
    if(id==failReadId&&(item==failReadItem||(length==72&&failReadItem==-72))){
      clockUs+=timeout*1000;
      if(packetFailure)packetError=4;else libError=1;return 0;
    }
    memset(out,0,length);
    if(length==72){
      out[0]=(uint8_t)regs[{id,TORQUE_ENABLE}];out[6]=(uint8_t)regs[{id,HARDWARE_ERROR_STATUS}];
      out[58]=(uint8_t)regs[{id,MOVING}];
      uint32_t value=(uint32_t)regs[{id,PRESENT_POSITION}];
      for(int j=0;j<4;j++)out[68+j]=(uint8_t)(value>>(8*j));
    }else{
      assert(item>=0);uint32_t value=(uint32_t)regs[{id,item}];
      for(int j=0;j<length;j++)out[j]=(uint8_t)(value>>(8*j));
    }
    return shortRead?length-1:length;
  }
  bool write(uint8_t id,uint16_t address,const uint8_t* bytes,uint16_t length,uint32_t){
    int32_t value=0;memcpy(&value,bytes,length);
    return writeControlTableItem((uint8_t)itemAt(address),id,value);
  }
  bool writeControlTableItem(uint8_t item,uint8_t id,int32_t value){
    assert(id<=252); clockUs+=100;libError=packetError=0;writes.push_back({id,item,value});transactions.push_back({id,item,value});
    if(id==failWriteId && item==failWriteItem){libError=1;return false;}
    regs[{id,item}]=value;
    if(item==GOAL_POSITION&&regs[{id,TORQUE_ENABLE}]&&followGoal)regs[{id,PRESENT_POSITION}]=value;
    if(item==TORQUE_ENABLE&&value==1&&id==torqueJumpId)regs[{id,PRESENT_POSITION}]+=torqueJump;
    if(afterWrite)afterWrite({id,item,value});
    if(lostWriteAck){libError=1;return false;}
    return true;
  }
  bool torqueOff(uint8_t id){return writeControlTableItem(TORQUE_ENABLE,id,0);}
  bool torqueOn(uint8_t id){return writeControlTableItem(TORQUE_ENABLE,id,1);}
  bool setOperatingMode(uint8_t id,uint8_t mode){return writeControlTableItem(OPERATING_MODE,id,mode);}
  bool setGoalPosition(uint8_t id,float raw,int){return writeControlTableItem(GOAL_POSITION,id,(int32_t)raw);}
  float getPresentPosition(uint8_t id,int){return (float)readControlTableItem(PRESENT_POSITION,id);}
};
#include "library_decode.inc"
namespace stage1 {
#include "stage1.inc"
}
namespace stage2 {
#include "stage2.inc"
}
int checks=0;
void check(bool ok,const char* label){checks++;if(!ok){std::cerr<<"FAIL: "<<label<<"\n"<<Serial2.output;std::exit(1);}}
bool anyMotion(const Dynamixel2Arduino &d){
  for(auto e:d.writes)if(e.item==GOAL_POSITION||(e.item==TORQUE_ENABLE&&e.value==1))return true;
  return false;
}
void resetRuntime(){
  stage2::dxl=Dynamixel2Arduino(Serial,2);Serial2=FakeSerial();clockUs=0;
  stage2::legStartupReady=false;stage2::faultMotorIndex=-1;stage2::nextZMonitor=0;stage2::nextZMonitorMs=0;
  for(auto &t:stage2::zLastGoodMs)t=0;
  stage2::mode=stage2::MODE_STANDING;stage2::hardwareReady=false;
  stage2::torqueEnabled=false;stage2::zCalibrationReady=false;stage2::zAxisReady=false;
  stage2::torqueOffUnconfirmed=true;stage2::cmdLen=0;
}
void configureMock(){
  resetRuntime();stage2::MOTION_COMMISSIONING_VERIFIED=true;
  // Synthetic data in this temporary test executable only. NOT robot calibration.
  for(int z=0;z<4;z++){
    auto &m=stage2::motors[stage2::Z_INDICES[z]];
    m.id=101+z;m.direction=1;m.offsetRad=stage2::PI_F;m.tolerance=2;
    m.type=stage2::MOTOR_XM430_W350_R;m.protocol=2.0f;m.maxRaw=4095;m.maxDegree=360.0f;
    stage2::zCalibration[z]={true,0.0f,1000,3000,10,5,3,0,0,45,100,100,20};
    auto &r=stage2::dxl.regs;int id=m.id;
    r[{id,MODEL_NUMBER}]=XM430_W350;r[{id,FIRMWARE_VERSION}]=45;r[{id,OPERATING_MODE}]=3;
    r[{id,MIN_POSITION_LIMIT}]=1000;r[{id,MAX_POSITION_LIMIT}]=3000;
    r[{id,PWM_LIMIT}]=100;r[{id,CURRENT_LIMIT}]=100;r[{id,VELOCITY_LIMIT}]=20;
    r[{id,STATUS_RETURN_LEVEL}]=2;r[{id,SECONDARY_ID}]=255;r[{id,PRESENT_POSITION}]=2048;
  }
  assert(stage2::validateZCalibration());
  float q[2];assert(stage2::inverseKinematics(stage2::START_X,stage2::START_Y,q));
  for(int i=0;i<8;i++){
    stage2::legStartup[i]={true,0,4095,10,2,0,0};
    int id=stage2::motors[i].id;auto &r=stage2::dxl.regs;
    r[{id,MODEL_NUMBER}]=XM430_W350;r[{id,FIRMWARE_VERSION}]=45;
    r[{id,OPERATING_MODE}]=3;r[{id,MIN_POSITION_LIMIT}]=0;r[{id,MAX_POSITION_LIMIT}]=4095;
    r[{id,STATUS_RETURN_LEVEL}]=2;r[{id,SECONDARY_ID}]=255;
    r[{id,PRESENT_POSITION}]=stage2::jointRadToMotorRaw(i,q[i%2]);
  }
  assert(stage2::validateLegStartup());
}
void stopCommand(){char stop[]="STOP";stage2::handleCommand(stop);}
void failureCleanup(){stage2::mode=stage2::MODE_STOPPED;stage2::safeCleanup();}
void testDefault(){
  resetRuntime();stage2::setup();
  check(!anyMotion(stage2::dxl),"default setup emits no torque-on or goal commands");
  check(stage2::mode==stage2::MODE_STOPPED&&!stage2::hardwareReady&&!stage2::zAxisReady,"default boot stopped");
  check(stage2::torqueOffUnconfirmed,"unset IDs do not claim torque off");
  for(const char *text:{"WALK","STAND","TOGGLE","START"}){
    char cmd[20];strcpy(cmd,text);stage2::handleCommand(cmd);stage2::loop();
    check(stage2::mode==stage2::MODE_STOPPED&&!anyMotion(stage2::dxl),"commands cannot bypass default gate");
  }
  check(!stage2::setupOneMotor(0)&&!stage2::holdZAxis()&&!stage2::smoothStandupBlocking(),"direct startup helpers gated");
  stage2::commandMotorModelRad(0,0);stage2::commandMotorModelRad(stage2::FL_Z,0);
  check(!anyMotion(stage2::dxl),"command helper cannot bypass gate");
  for(auto &c:stage2::zCalibration)c.verified=true;
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"verified flags alone cannot authorize unset Z values");
}
void invalidConfig(const std::function<void()> &mutate){
  configureMock();mutate();check(!stage2::setupAllMotors(),"invalid calibration blocks setup");
  check(!anyMotion(stage2::dxl),"invalid calibration sends no motion commands");
}
void testCalibration(){
  for(int z=0;z<4;z++)invalidConfig([=]{stage2::zCalibration[z].verified=false;});
  for(int id:{253,254,255,1,102})invalidConfig([=]{stage2::motors[8].id=id;});
  invalidConfig([]{stage2::motors[8].direction=0;});
  invalidConfig([]{stage2::motors[8].offsetRad=NAN;});
  invalidConfig([]{stage2::motors[8].protocol=1.0f;});
  invalidConfig([]{stage2::zCalibration[0].neutralJointRad=NAN;});
  invalidConfig([]{stage2::zCalibration[0].neutralJointRad=1000;});
  invalidConfig([]{stage2::zCalibration[0].minRaw=-1;});
  invalidConfig([]{stage2::zCalibration[0].maxRaw=4096;});
  invalidConfig([]{stage2::zCalibration[0].minRaw=2500;});
  invalidConfig([]{stage2::zCalibration[0].minRaw=3000;});
  invalidConfig([]{stage2::motors[8].tolerance=-1;});
  invalidConfig([]{stage2::motors[8].tolerance=11;});
  invalidConfig([]{stage2::zCalibration[0].startupToleranceRaw=-1;});
  invalidConfig([]{stage2::zCalibration[0].profileVelocityRaw=0;});
  invalidConfig([]{stage2::zCalibration[0].profileAccelerationRaw=0;});
  invalidConfig([]{stage2::zCalibration[0].profileAccelerationRaw=32768;});
  invalidConfig([]{stage2::zCalibration[0].expectedHomingOffsetRaw=INT32_MIN;});
  invalidConfig([]{stage2::zCalibration[0].expectedFirmwareVersion=-1;});
  for(int mode:{-1,4,8,12})invalidConfig([=]{stage2::zCalibration[0].expectedDriveMode=mode;});
  invalidConfig([]{stage2::zCalibration[0].expectedPwmLimitRaw=-1;});
  invalidConfig([]{stage2::zCalibration[0].expectedCurrentLimitRaw=-1;});
  invalidConfig([]{stage2::zCalibration[0].expectedVelocityLimitRaw=4;});
  configureMock();int32_t raw;
  stage2::motors[8].direction=-1;
  check(stage2::zJointRadToRawChecked(0,0.1f,raw)&&raw<2048,"Z direction and offset applied");
  check(!stage2::zJointRadToRawChecked(0,10.0f,raw),"Z never wraps out-of-turn angles");
  check(!stage2::zJointRadToRawChecked(0,INFINITY,raw),"Z rejects infinite angles");
}
void testSettings(){
  for(int item:{MODEL_NUMBER,FIRMWARE_VERSION,OPERATING_MODE,DRIVE_MODE,HOMING_OFFSET,MIN_POSITION_LIMIT,
               MAX_POSITION_LIMIT,PWM_LIMIT,CURRENT_LIMIT,VELOCITY_LIMIT,STATUS_RETURN_LEVEL,
               REGISTERED_INSTRUCTION,HARDWARE_ERROR_STATUS,BUS_WATCHDOG,STARTUP_CONFIGURATION}){
    configureMock();stage2::dxl.regs[{104,item}]++;
    check(!stage2::setupAllMotors(),"hardware setting mismatch blocks setup");
    check(!anyMotion(stage2::dxl),"last Z setting failure occurs before any torque-on/goal");
  }
  configureMock();stage2::dxl.regs[{104,SECONDARY_ID}]=1;
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"secondary alias rejected");
  for(int raw:{-1,999,1000,2037,2059,3001,6144}){
    configureMock();stage2::dxl.regs[{104,PRESENT_POSITION}]=raw;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"invalid startup position rejected before enable");
  }
  configureMock();stage2::dxl.regs[{104,PRESENT_VELOCITY}]=1;
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"moving Z rejected");
  configureMock();stage2::zCalibration[0].expectedHomingOffsetRaw=1024;
  stage2::zCalibration[0].minRaw=0;stage2::zCalibration[0].startupToleranceRaw=2000;
  stage2::dxl.regs[{101,HOMING_OFFSET}]=1024;stage2::dxl.regs[{101,MIN_POSITION_LIMIT}]=0;
  stage2::dxl.regs[{101,PRESENT_POSITION}]=500;
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"homing offset cannot hide a counter reset turn");
}
void testFaults(){
  for(bool packet:{false,true}){
    configureMock();stage2::dxl.failReadId=104;stage2::dxl.failReadItem=PRESENT_POSITION;
    stage2::dxl.packetFailure=packet;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"read error is not a valid zero position");
  }
  configureMock();stage2::dxl.failWriteId=101;stage2::dxl.failWriteItem=GOAL_POSITION;
  check(!stage2::setupAllMotors(),"failed preload aborts");
  check(std::none_of(stage2::dxl.writes.begin(),stage2::dxl.writes.end(),[](Event e){return e.item==TORQUE_ENABLE&&e.value==1;}),"failed preload cannot enable torque");
  configureMock();stage2::dxl.torqueJumpId=101;stage2::dxl.torqueJump=5;
  check(!stage2::setupAllMotors(),"position shift at torque-on aborts");failureCleanup();
  check(!stage2::hardwareReady&&!stage2::zAxisReady&&!stage2::torqueOffUnconfirmed,"failure cleans partial startup");
  configureMock();stage2::dxl.followGoal=false;stage2::dxl.regs[{101,PRESENT_POSITION}]=2053;
  check(!stage2::setupAllMotors(),"failure to reach neutral times out");failureCleanup();
  check(Serial2.output.find("z_settle_timeout")!=std::string::npos,"settle timeout reported");
  configureMock();bool fired=false;
  stage2::dxl.afterWrite=[&](Event e){if(!fired&&e.item==TORQUE_ENABLE&&e.value==1){fired=true;Serial2.feed("STOP\n");}};
  check(!stage2::setupAllMotors()&&stage2::mode==stage2::MODE_STOPPED,"STOP during Z setup aborts");
  check(!stage2::torqueOffUnconfirmed,"STOP confirms cleanup in mock");
  int ons=0;for(auto e:stage2::dxl.writes)if(e.item==TORQUE_ENABLE&&e.value==1)ons++;
  check(ons==1,"no further torque enables after STOP");
  configureMock();stage2::dxl.failWriteId=102;stage2::dxl.failWriteItem=TORQUE_ENABLE;
  stage2::dxl.regs[{102,TORQUE_ENABLE}]=1;stage2::safeCleanup();
  check(stage2::torqueOffUnconfirmed,"cleanup failure remains visible");
  check(stage2::dxl.regs[{104,TORQUE_ENABLE}]==0,"cleanup continues after a failed motor");
}
void testSuccess(){
  configureMock();stage2::dxl.regs[{101,PRESENT_POSITION}]=2053;
  check(stage2::setupAllMotors()&&stage2::zAxisReady,"valid mock setup succeeds");
  check(stage2::dxl.regs[{101,GOAL_POSITION}]==2048,"neutral command follows bounded startup");
  for(int id: {1,2,3,4,5,6,7,8,101,102,103,104}){
    auto &w=stage2::dxl.writes;
    auto on=std::find_if(w.begin(),w.end(),[=](Event e){return e.id==id&&e.item==TORQUE_ENABLE&&e.value==1;});
    check(on!=w.end()&&std::any_of(w.begin(),on,[=](Event e){return e.id==id&&e.item==GOAL_POSITION&&e.value==(id==101?2053:stage2::dxl.regs[{id,PRESENT_POSITION}]);}),"measured goal precedes torque-on");
    if(id>=101)check(stage2::dxl.regs[{id,PROFILE_VELOCITY}]>0&&stage2::dxl.regs[{id,PROFILE_ACCELERATION}]>0,"finite Z profiles");
  }
  stage2::hardwareReady=true;stage2::mode=stage2::MODE_STANDING;
  size_t n=stage2::dxl.writes.size();stage2::commandMotorModelRad(stage2::FL_Z,0);
  check(stage2::dxl.writes.size()==n,"leg command helper cannot command Z");
  stopCommand();check(stage2::mode==stage2::MODE_STOPPED&&!stage2::torqueOffUnconfirmed,"STOP disables all configured motors");
  char cmd[]="WALK";stage2::handleCommand(cmd);check(stage2::mode==stage2::MODE_STOPPED,"STOP is latched until reset");
  configureMock();stage2::setup();check(stage2::hardwareReady&&stage2::zAxisReady&&stage2::mode==stage2::MODE_STANDING,"full mock startup and standup succeed");
}
void compareWalking(){
  configureMock();stage2::zCalibrationReady=true;stage2::legStartupReady=true;stage2::zAxisReady=true;stage2::hardwareReady=true;
  stage1::dxl=Dynamixel2Arduino(Serial,2);
  float q1[2],q2[2];
  check(stage1::inverseKinematics(stage1::START_X,stage1::START_Y,q1)&&stage2::inverseKinematics(stage2::START_X,stage2::START_Y,q2),"start IK valid");
  float stand1[2],stand2[2];stage1::inverseKinematics(0,-150,stand1);stage2::inverseKinematics(0,-150,stand2);
  auto compareCommands=[](){
    for(auto &t:stage2::zLastGoodMs)t=millis();
    stage1::dxl.writes.clear();stage2::dxl.writes.clear();stage1::writeAllLegsRad();stage2::writeAllLegsRad();
    check(stage1::dxl.writes.size()==8&&stage2::dxl.writes.size()==8,"exactly eight walking commands");
    for(int i=0;i<8;i++){
      auto a=stage1::dxl.writes[i],b=stage2::dxl.writes[i];
      check(a.id==b.id&&a.item==b.item&&a.value==b.value,"identical ordered motor raw commands");
    }
  };
  for(int step=1;step<=300;step++){
    float t=(float)step/300,alpha=t*t*(3-2*t);float a[2],b[2];
    for(int j=0;j<2;j++){a[j]=q1[j]+alpha*(stand1[j]-q1[j]);b[j]=q2[j]+alpha*(stand2[j]-q2[j]);}
    stage1::setAllLegActions(a);stage2::setAllLegActions(b);compareCommands();
  }
  for(int step=0;step<1000;step++){
    bool walking=step<600||step>=800;
    for(int leg=0;leg<4;leg++){
      int shift=(leg==1||leg==2)?25:0;float x1=0,y1=-150,x2=0,y2=-150;
      if(walking){stage1::getCycleReferenceByIndex((step+shift)%50,x1,y1);stage2::getCycleReferenceByIndex((step+shift)%50,x2,y2);}
      check(x1==x2&&y1==y2,"identical foot references");
      stage1::computeAction(x1,y1,stage1::previousAction[leg]);stage2::computeAction(x2,y2,stage2::previousAction[leg]);
      for(int j=0;j<2;j++)check(stage1::previousAction[leg][j]==stage2::previousAction[leg][j],"identical limited model angles");
    }
    compareCommands();
  }
  std::cout<<"PASS: 300 standup + 1000 walking/standing samples; 10400 ordered raw motor commands identical\n";
}

void testActualDecoder(){
  configureMock();int32_t value;
  for(int item:{SECONDARY_ID,FIRMWARE_VERSION})for(int raw:{0,127,128,252,253,254,255}){
    stage2::dxl.regs[{101,item}]=raw;
    int32_t old=stage2::dxl.readControlTableItem((uint8_t)item,101);
    check(old==(raw<128?raw:raw-256),"actual D2A 0.8.2 decoder reproduces signed byte values");
    check(stage2::readMotorItem(8,(uint8_t)item,value)&&value==raw,"raw helper preserves unsigned byte");
  }
  stage2::dxl.regs[{101,HOMING_OFFSET}]=-1024;
  check(stage2::readMotorItem(8,HOMING_OFFSET,value)&&value==-1024,"signed four-byte bit pattern preserved");
  stage2::dxl.shortRead=true;
  check(!stage2::readMotorItem(8,PRESENT_POSITION,value),"short response rejected");
  for(int raw:{253,254,255}){
    configureMock();stage2::dxl.regs[{101,SECONDARY_ID}]=raw;
    stage2::dxl.regs[{101,FIRMWARE_VERSION}]=200;stage2::zCalibration[0].expectedFirmwareVersion=200;
    check(stage2::setupAllMotors(),"all three disabled secondary IDs and unsigned firmware accepted");
  }
  std::cout<<"PASS: actual library decoder and control table compiled; unsigned and signed-width regression\n";
}

void testLegSafety(){
  configureMock();stage2::MOTION_COMMISSIONING_VERIFIED=false;
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"physical commissioning gate cannot be skipped");
  for(int i=0;i<8;i++){
    configureMock();stage2::legStartup[i].verified=false;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"each walking joint needs mechanical signoff");
    configureMock();stage2::dxl.regs[{stage2::motors[i].id,PRESENT_POSITION}]+=11;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"wrong walking start pose blocked before any enable");
    configureMock();stage2::legStartup[i].minRaw=-1;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"walking verified flag alone cannot accept invalid limits");
  }
  for(int item:{MODEL_NUMBER,OPERATING_MODE,DRIVE_MODE,HOMING_OFFSET,MIN_POSITION_LIMIT,
                MAX_POSITION_LIMIT,STATUS_RETURN_LEVEL,REGISTERED_INSTRUCTION,HARDWARE_ERROR_STATUS,
                BUS_WATCHDOG,STARTUP_CONFIGURATION}){
    configureMock();stage2::dxl.regs[{6,item}]++;
    check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"last walking motor settings reject before any enable");
  }
  for(int item:{PROFILE_VELOCITY,PROFILE_ACCELERATION,GOAL_POSITION,TORQUE_ENABLE}){
    configureMock();stage2::dxl.failWriteId=6;stage2::dxl.failWriteItem=item;
    check(!stage2::setupAllMotors()&&!stage2::hardwareReady,"walking critical write failure cannot report readiness");
  }
  configureMock();stage2::dxl.afterWrite=[](Event e){
    if(e.id==6&&e.item==PROFILE_ACCELERATION)stage2::dxl.regs[{6,PROFILE_ACCELERATION}]=21;
  };
  check(!stage2::setupAllMotors()&&!anyMotion(stage2::dxl),"walking profile readback mismatch rejected");
  configureMock();stage2::dxl.afterWrite=[](Event e){
    if(e.id==6&&e.item==TORQUE_ENABLE&&e.value==1){stage2::dxl.failReadId=6;stage2::dxl.failReadItem=TORQUE_ENABLE;}
  };
  check(!stage2::setupAllMotors()&&stage2::mode==stage2::MODE_STOPPED&&stage2::torqueOffUnconfirmed,
        "last motor enable readback failure cleans partial initialization and remains unconfirmed");
  configureMock();stage2::dxl.torqueJumpId=6;stage2::dxl.torqueJump=3;
  check(!stage2::setupAllMotors()&&stage2::mode==stage2::MODE_STOPPED,"walking torque-on position shift stops all motors");
  configureMock();check(stage2::setupAllMotors(),"walking hold setup fixture");
  stage2::hardwareReady=true;stage2::dxl.regs[{6,PRESENT_POSITION}]+=3;
  auto count=stage2::dxl.writes.size();
  check(!stage2::smoothStandupBlocking(),"drift after setup blocks standup");
  for(size_t j=count;j<stage2::dxl.writes.size();j++)
    check(stage2::dxl.writes[j].item!=GOAL_POSITION,"no ramp goals after failed final hold check");
  std::cout<<"PASS: walking measured holds, mechanical gates, setup/readback and partial initialization failures\n";
}

void testDispatch(){
  configureMock();stage2::faultMotorIndex=10;
  stage2::dxl.transactions.clear();stage2::dxl.writes.clear();
  stage2::safeCleanup();
  check(stage2::dxl.writes.size()==12&&stage2::dxl.writes[0].id==103,"affected Z disabled first");
  auto &t=stage2::dxl.transactions;
  check(t.size()==24,"12 writes then 12 verification reads");
  for(int i=0;i<12;i++)check(t[i].item==TORQUE_ENABLE&&t[i].value==0,"all disable dispatch precedes first readback");
  check(!stage2::torqueOffUnconfirmed&&stage2::torqueOffDispatchAcknowledged,"successful dispatch and readback separately tracked");
  configureMock();stage2::dxl.lostWriteAck=true;stage2::safeCleanup();
  check(!stage2::torqueOffUnconfirmed&&!stage2::torqueOffDispatchAcknowledged,"lost ACK can still have confirmed zero readback");
  configureMock();stage2::dxl.failReadId=101;stage2::dxl.failReadItem=TORQUE_ENABLE;stage2::safeCleanup();
  check(stage2::torqueOffUnconfirmed&&stage2::torqueOffDispatchAcknowledged,"acknowledged request does not prove torque off");
  configureMock();check(stage2::setupAllMotors(),"dispatch fault fixture");
  stage2::hardwareReady=true;stage2::dxl.writes.clear();stage2::dxl.regs[{102,HARDWARE_ERROR_STATUS}]=4;
  check(!stage2::readZFeedback(1)&&stage2::dxl.writes[0].id==102,"runtime Z fault uses affected-first shutdown");
  std::cout<<"PASS: shutdown dispatch order, partial communication loss, commanded versus confirmed torque-off\n";
}

void testSupervision(){
  for(int item:{PRESENT_POSITION,TORQUE_ENABLE,HARDWARE_ERROR_STATUS,MOVING}){
    configureMock();check(stage2::setupAllMotors(),"supervision setup fixture");stage2::hardwareReady=true;
    stage2::dxl.regs[{104,item}]=(item==PRESENT_POSITION?2051:(item==TORQUE_ENABLE?0:1));
    check(!stage2::revalidateZReady()&&stage2::mode==stage2::MODE_STOPPED&&!stage2::zAxisReady,
          "holding position/torque/error/moving loss revokes readiness");
  }
  configureMock();check(stage2::setupAllMotors(),"prestandup Z fixture");stage2::hardwareReady=true;
  stage2::dxl.regs[{104,TORQUE_ENABLE}]=0;stage2::dxl.writes.clear();
  check(!stage2::smoothStandupBlocking()&&!anyMotion(stage2::dxl),"Z loss immediately before standup blocks ramp");
  configureMock();check(stage2::setupAllMotors(),"monitor scheduling fixture");stage2::hardwareReady=true;
  stage2::nextZMonitorMs=millis();stage2::dxl.transactions.clear();
  stage2::serviceZMonitor(millis()+3);
  check(stage2::dxl.transactions.empty(),"monitor does not consume a slot without 4 ms slack");
  stage2::serviceZMonitor(millis()+10);
  check(stage2::dxl.transactions.size()==1,"one snapshot per spare-time service");
  delay(stage2::Z_MONITOR_MAX_AGE_MS+1);stage2::dxl.writes.clear();stage2::commandMotorModelRad(0,0);
  check(stage2::mode==stage2::MODE_STOPPED&&!anyMotion(stage2::dxl),"stale feedback blocks next goal and latches stop");
  configureMock();check(stage2::setupAllMotors(),"slow feedback fixture");stage2::hardwareReady=true;
  stage2::dxl.snapshotUs=3001;
  check(!stage2::readZFeedback(0)&&stage2::mode==stage2::MODE_STOPPED,"monitor exceeding budget faults");
  for(bool packet:{false,true}){
    configureMock();check(stage2::setupAllMotors(),"feedback communication fixture");stage2::hardwareReady=true;
    stage2::dxl.failReadId=101;stage2::dxl.failReadItem=-72;stage2::dxl.packetFailure=packet;
    check(!stage2::readZFeedback(0)&&!stage2::zAxisReady,"feedback packet and transport errors fault");
  }
  configureMock();check(stage2::setupAllMotors(),"runtime goal failure fixture");stage2::hardwareReady=true;
  stage2::dxl.failWriteId=1;stage2::dxl.failWriteItem=GOAL_POSITION;
  stage2::commandMotorModelRad(0,0);
  check(stage2::mode==stage2::MODE_STOPPED&&!stage2::hardwareReady,"runtime goal write failure stops further commands");
  configureMock();check(stage2::setupAllMotors(),"status failure fixture");stage2::hardwareReady=true;
  stage2::dxl.failReadId=102;stage2::dxl.failReadItem=PRESENT_POSITION;stage2::sendStatus(true);
  check(stage2::mode==stage2::MODE_STOPPED&&!stage2::zAxisReady,"failed diagnostic read triggers cleanup too");
  std::cout<<"PASS: immediate prestandup revalidation, stale/failed feedback, slack scheduling and budget rejection\n";
}

void testSettling(){
  bool stable=false;unsigned long since=0;
  check(!stage2::zSettleObservation(true,350,0,stable,since)&&since==350,"delayed first good scan starts dwell after completion");
  check(!stage2::zSettleObservation(true,549,0,stable,since),"199 ms is insufficient");
  check(stage2::zSettleObservation(true,550,0,stable,since),"subsequent complete observation spans 200 ms");
  check(!stage2::zSettleObservation(false,551,0,stable,since)&&!stable,"bad observation resets dwell");
  check(!stage2::zSettleObservation(true,900,0,stable,since)&&since==900,"next good observation starts new dwell");
  check(!stage2::zSettleObservation(true,1099,0,stable,since),"interrupted interval cannot accumulate");
  check(stage2::zSettleObservation(true,1100,0,stable,since),"new complete stable interval accepted");
  check(!stage2::zSettleObservation(true,5000,0,stable,since),"overall timeout checked before readiness");
  configureMock();stage2::dxl.failReadId=102;stage2::dxl.failReadItem=MOVING;
  check(!stage2::setupAllMotors()&&!stage2::zAxisReady,"incomplete settling observation aborts without readiness");
  configureMock();bool delayed=false;
  stage2::dxl.beforeRead=[&](int,int item){if(!delayed&&item==MOVING){delayed=true;delay(5100);}};
  check(!stage2::setupAllMotors()&&!stage2::zAxisReady,"good scan completing after overall timeout rejected");
  configureMock();check(stage2::setupAllMotors(),"short snapshot fixture");stage2::dxl.shortRead=true;
  check(!stage2::readZFeedback(0)&&!stage2::zAxisReady,"incomplete snapshot never refreshes readiness");
  std::cout<<"PASS: delayed, interrupted, incomplete and over-timeout settling observations\n";
}

int main(){
  testDefault();testCalibration();testSettings();testFaults();testSuccess();
  testActualDecoder();testLegSafety();testDispatch();testSupervision();testSettling();compareWalking();
  std::cout<<"PASS: "<<checks<<" assertions; simulated transport/IMU only, no hardware validation\n";
}
