#include <Pushbutton.h>
#include <NewPing.h>
#include <ZumoMotors.h>
#include <math.h>
#include <Wire.h>
#include <LSM303.h>

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (x)
#else
  #define DEBUG_PRINT(x)
#endif

#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 5

Pushbutton button(ZUMO_BUTTON);
LSM303 compass;

#define LED_PIN 13
#define PING_KESKEL_PIN  11 // sonari triger pin
#define PING_PAREMAL_PIN 2
#define PING_VASAKUL_PIN 4
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define PING_MEDIAN_DELAY 0//29 // make pings faster

#define CAL_SPEED 400 // mootorite pöörlemis kiirus kalibreerimisel
#define TURN_BASE_SPEED 60 // mimimum pööramis kiirus laburindis
#define SPEED 400 // maksimum pööramis kiirus laburindis

#define DISABLE_ONE_PIN true

NewPing sonar_keskel(PING_KESKEL_PIN, PING_KESKEL_PIN, MAX_DISTANCE); // keskmine kaugusandur
NewPing sonar_paremal(PING_PAREMAL_PIN, PING_PAREMAL_PIN, MAX_DISTANCE); // parempoolne kaugusandur
NewPing sonar_vasaskul(PING_VASAKUL_PIN, PING_VASAKUL_PIN, MAX_DISTANCE); // vasakpoolne kaugusandur

ZumoMotors motors;

unsigned int vKaugus, pKaugus, kKaugus;
float target_heading;
int keskpunkt = 0;
bool turning = false; // kas me praegu sooritame pööret laburindis
int vspeed, pspeed;

typedef enum {OTSE, PAREM, VASAK, TUPIK} state_t;

void setup() {
    // The highest possible magnetic value to read in any direction is 2047
    // The lowest possible magnetic value to read in any direction is -2047
    LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

    // Initiate the Wire library and join the I2C bus as a master
    Wire.begin();

    // Initiate LSM303
    compass.init();

    // Enables accelerometer and magnetometer
    compass.enableDefault();

    compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
    compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate


    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    pinMode(13, OUTPUT);
    motors.flipLeftMotor(true);
    motors.flipRightMotor(true);

    // kompassi kalibreerimine võtab kaua aega
    if (button.isPressed()) {
        DEBUG_PRINT("cal skipped\n\r");
        goto skip_cal;
    }
    goto skip_cal;

    digitalWrite(LED_PIN, HIGH);
    DEBUG_PRINT("Waiting for cal start button\r\n");
    button.waitForButton();
    digitalWrite(LED_PIN, LOW);
    motors.setLeftSpeed(CAL_SPEED);
    motors.setRightSpeed(-CAL_SPEED);

    for(int i=0; i < CALIBRATION_SAMPLES; i++)
    {
        // Take a reading of the magnetic vector and store it in compass.m
        compass.read();

        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);

        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);

        DEBUG_PRINT(i);
        DEBUG_PRINT("\n\r");

        delay(50);
    }
    skip_cal:
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);


    compass.m_max.x = running_max.x;
    compass.m_max.y = running_max.y;
    compass.m_min.x = running_min.x;
    compass.m_min.y = running_min.y;

    digitalWrite(LED_PIN, HIGH);
    DEBUG_PRINT("cal finished, waiting for start button\r\n");
    button.waitForButton();
    DEBUG_PRINT("running\r\n");
    digitalWrite(LED_PIN, LOW);
    target_heading = averageHeading();
}

void loop() {
    float heading, relative_heading;
    int drive_speed;
    keskpunkt = 0;

    // Heading is given in degrees away from the magnetic vector, increasing clockwise
    heading = averageHeading();

    // This gives us the relative heading with respect to the target angle
    relative_heading = relativeHeading(heading, target_heading);

    DEBUG_PRINT("target_heading=");
    DEBUG_PRINT(target_heading);
    DEBUG_PRINT("\t");

    DEBUG_PRINT("relative_heading=");
    DEBUG_PRINT(relative_heading);
    DEBUG_PRINT("\t");

    get_kaugus();
    DEBUG_PRINT("Vasak=");
    //delay(10);
    DEBUG_PRINT(vKaugus);// / US_ROUNDTRIP_CM);
    DEBUG_PRINT("\tKeskmine=");
    //delay(10);
    DEBUG_PRINT(kKaugus);// / US_ROUNDTRIP_CM);
    DEBUG_PRINT("\tParem=");
    //delay(10);
    DEBUG_PRINT(pKaugus);// / US_ROUNDTRIP_CM);
    //DEBUG_PRINT("\t speed=");
    //int speed = (int)(pow((double)kKaugus/100, -2)*4000);
    //DEBUG_PRINT(speed);
    DEBUG_PRINT("\t");
    keskpunkt = pKaugus - vKaugus;
    DEBUG_PRINT("keskpunkt=");
    DEBUG_PRINT(keskpunkt);
    DEBUG_PRINT("\t");

    if (!turning) {//(abs(relative_heading) < DEVIATION_THRESHOLD) {
        // oleme otse
        if /*(true){/*/(kKaugus > 500) {
            int vpeed, pspeed;
            vspeed = 400 + (keskpunkt/2);
            pspeed = 400 - (keskpunkt/2);

            motors.setSpeeds(vspeed, pspeed);
            //drive(400, keskpunkt);
            DEBUG_PRINT("state= ees vaba");
        // } else if (kKaugus < 500){
        //     DEBUG_PRINT("state= ees blokitud");
        //     motors.setSpeeds(0, 0);

        } else if (pKaugus > 750) {
            DEBUG_PRINT("state= p vaba");
            motors.setSpeeds(400, 400);
            delay(800);
            motors.setSpeeds(0, 0);
            delay(100);
            target_heading = fmod(averageHeading() + 90, 360);
            //turning = true;
            motors.setSpeeds(400, -400);
            delay(1000);

        } else if (vKaugus > 750) {
            //motors.setSpeeds(0, 0);
            DEBUG_PRINT("state= v vaba");
            motors.setSpeeds(400, 400);
            delay(800);
            //motors.setSpeeds(0, 0);
            delay(100);
            // motors.setSpeeds(400, 400);
            // delay(3000);
            // if (vKaugus > 750) {
            motors.setSpeeds(-400, 400);
            delay(1000);
            target_heading = fmod(averageHeading() - 90, 360);
            //turning = true;
            // } else if (pKaugus > 750) {
            //     motors.setSpeeds(400, -400);
            //     target_heading = fmod(averageHeading() + 90, 360);
            // }
            // delay(2000);
        } else {
            DEBUG_PRINT("state= tupik");
            //motors.setSpeeds(0, 0);
            //motors.setSpeeds(-400, 400);
            //delay(2000);
        } 
    } else {
        // ei ole otse, keerame ennast otseks

        DEBUG_PRINT("state= turning");
        drive_speed = SPEED*relative_heading/180;

        if (drive_speed < 0)
          drive_speed -= TURN_BASE_SPEED;
        else
          drive_speed += TURN_BASE_SPEED;

        motors.setSpeeds(drive_speed, -drive_speed);

        if  (abs(relative_heading) < DEVIATION_THRESHOLD) { //enamvähem otse oleme
            turning = false;
        }
    }
    DEBUG_PRINT("\n\r");
}

void drive(int speed, int turn) {
    DEBUG_PRINT("turn=");
    DEBUG_PRINT(turn);
    DEBUG_PRINT("\t");
    motors.setSpeeds(speed + turn, speed - turn);
}

void get_kaugus() {
    #define median_iterations 3
    // midagi üle 1000hz on see siin võimeline kaugust uuendama 
    //delay(50);
    vKaugus = sonar_vasaskul.ping_median(median_iterations);
    //delay(50);
    kKaugus = sonar_keskel.ping_median(median_iterations);
    //delay(50);
    pKaugus = sonar_paremal.ping_median(median_iterations);
}

// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}
