// Macros.
#define MAX_DUTY 255  //< Value representing maximum PWM duty cycle for the 
                      //< motors. 255 is 100% duty, or always on.
#define DEADBAND_DELTA 100
#define NEUTRAL_CMD 1500

#define ANGULAR_RATE_PIN 0
#define LINEAR_RATE_PIN 1
#define M1_PWM_PIN 6
#define M1_DO_PIN 7
#define M2_PWM_PIN 5
#define M2_DO_PIN 4
#define LED_PIN 13

// Variables for global communication.
const unsigned int NEUTRAL_AR_CMD = NEUTRAL_CMD;
const unsigned int NEUTRAL_LR_CMD = NEUTRAL_CMD;

volatile unsigned long ulARStartTime = 0;
volatile unsigned int uiARCmd = NEUTRAL_AR_CMD;
volatile boolean bARCmdIsReady = false;

volatile unsigned long ulLRStartTime = 0;
volatile unsigned int uiLRCmd = NEUTRAL_AR_CMD;
volatile boolean bLRCmdIsReady = false;

void ANG_RATE_ISR()
{
  // If the pin is high then it was a rising edge.
  if (digitalRead(ANGULAR_RATE_PIN) == HIGH)
  {
    ulARStartTime = micros();
  }
  // Otherwise it's a falling edge.
  else
  {
    if (ulARStartTime && !bARCmdIsReady)
    {
      // Update Angular Rate command.
      uiARCmd = (unsigned int)(micros() - ulARStartTime);

      // Reset flags.
      ulARStartTime = 0;
      bARCmdIsReady = true;
    }
  }
}

void LIN_RATE_ISR()
{
  // If the pin is high then it was a rising edge.
  if (digitalRead(LINEAR_RATE_PIN) == HIGH)
  {
    ulLRStartTime = micros();
  }
  // Otherwise it's a falling edge.
  else
  {
    if (ulLRStartTime && !bLRCmdIsReady)
    {
      // Update Angular Rate command.
      uiLRCmd = (unsigned int)(micros() - ulLRStartTime);

      // Reset flags.
      ulLRStartTime = 0;
      bLRCmdIsReady = true;
    }
  }
}

class MotorCmd
{
  public:
    // Duty goes from 0 to 255.
    unsigned char duty;
    // Sign is either LOW or HIGH.
    unsigned char sign;
    // Pin that PWM is sent on.
    unsigned char pwm_pin;
    // Pin that "sign" is controlled with.
    unsigned char sign_pin;
};

class TwistCmd
{
  public:
    // Velocities range from -1 to 1.
    float vel, ang;
};

class DriveCmd
{
  public:
    /**
     * @brief Constructor for DriveCmd.
     * @param leftPwmPin    Pin that the pwm command for the left motor is sent on.
     * @param leftSignPin   Pin that the left motor "sign" is commanded.
     * @param rightPwmPin   Pin that the pwm command for the right motor is sent on.
     * @param rightSignPin  Pin that the right motor "sign" is commanded.
     */
    DriveCmd(unsigned char leftPwmPin, unsigned char leftSignPin, unsigned char rightPwmPin, unsigned char rightSignPin)
    {
      // Initialize motors.
      left_.duty = left_.sign = 0;
      right_.duty = right_.sign = 0;
      left_.pwm_pin = leftPwmPin;
      left_.sign_pin = leftSignPin;
      right_.pwm_pin = rightPwmPin;
      right_.sign_pin = rightSignPin;
    }
    /**
     * @brief Method to set pointers to the variables that receive the RC commands.
     * @param ang_rate_flag     Flag to be polled to check if there is a new angular rate command.
     * @param ang_rate_pw       Variable that holds the new angular rate command.
     * @param lin_rate_flag     Flag to be polled to check if there is a new linear rate command.
     * @param lin_rate_pw       Variable that holds the new angular rate command.
     */
    void attachRCPointers(volatile boolean * ang_rate_flag,
                          volatile unsigned int * ang_rate_pw,
                          volatile boolean * lin_rate_flag,
                          volatile unsigned int * lin_rate_pw )
    {
      is_ang_rate_ready_ = ang_rate_flag;
      is_lin_rate_ready_ = lin_rate_flag;
      ang_rate_micros_ = ang_rate_pw;
      lin_rate_micros_ = lin_rate_pw;
    }
    /**
     * @brief Method to update the robot drive commands.
     */
    void update()
    {
      unsigned int ang_rate_pw = 0;
      unsigned int lin_rate_pw = 0;
      if (*is_ang_rate_ready_ && *is_lin_rate_ready_)
      {
        // Make local copies to free up the flags.
        ang_rate_pw = *ang_rate_micros_;
        lin_rate_pw = *lin_rate_micros_;

        // Free flags.
        *is_ang_rate_ready_ = false;
        *is_lin_rate_ready_ = false;

        // Update robot.
        sendCommand(rc2Twist(ang_rate_pw, lin_rate_pw));
      }
    }
  private:
    /**
    * @brief Method to convert rc pulse widths to twist commands.
    * Pulse widths are in microseconds. They range from 1000 to 2000.
    * @param ang_rate_micros   Pulse width for angular rate command.
    * @param lin_rate_micros   Pulse width for linear rate command.
    */
    TwistCmd rc2Twist(unsigned int ang_rate_micros, unsigned int lin_rate_micros)
    {
      // Limit the pulse widths.
      ang_rate_micros = min(2000, max(1000, ang_rate_micros));
      lin_rate_micros = min(2000, max(1000, lin_rate_micros));

      // Deadband the commands.
      ang_rate_micros = deadbandFilter(ang_rate_micros);
      lin_rate_micros = deadbandFilter(lin_rate_micros);

      // Map the ranges.
      float ang_cmd, lin_cmd;
      ang_cmd = rc2vel(ang_rate_micros);
      lin_cmd = rc2vel(lin_rate_micros);

      TwistCmd twist;
      twist.vel = lin_cmd;
      twist.ang = ang_cmd;
      return twist;

    }
    /**
     * @brief Method to convert pulse widths to velocities.
     * @param pw    The pulse width in microseconds.
     * @return      Velocity command [-1.0, 1.0]
     */
    float rc2vel(unsigned int pw)
    {
      float vel_lim;
      signed int pw_max;
      if (pw > NEUTRAL_CMD)
      {
        // Map to upper range
        vel_lim = 1.0;
        pw_max = 2000;
      }
      else
      {
        // Map to lower range
        vel_lim = -1.0;
        pw_max = 1000;
      }

      // Compute slope and offset.
      float m, b;
      m = vel_lim / (float)(pw_max - NEUTRAL_CMD);
      b = -m * NEUTRAL_CMD;

      // Compute velocity.
      float vel = m * pw + b;

      // Shape velocity command to something more natural.
      vel = velShaper(vel);

      // Limit velocity.
      vel = min(1.0, max(-1.0, vel));

      return vel;
    }

    /**
     * @brief Method to shape the velocity command mapping to something more natural.
     * @param vel     Input velocity.
     * @return        Shaped velocity.
     */
    float velShaper(float vel)
    {
      // Preserve the sign of the velocity.
      float sign;
      if(vel >= 0)
      {
        sign = 1.0;
      }
      else
      {
        sign = -1.0;
      }
      return sign * pow(vel, 2.0);
    }

    /**
     * @brief Method to enforce deadband on pulse widths.
     * @param pw  Pulse width to deadband.
     * @return New pulse width.
     */
    unsigned int deadbandFilter(unsigned int pw)
    {
      // Check to see if the command is in the deadband.
      if (pw < NEUTRAL_CMD + DEADBAND_DELTA &&
          pw > NEUTRAL_CMD - DEADBAND_DELTA)
      {
        pw = NEUTRAL_CMD;
      }

      return pw;
    }

    /**
    * @brief Method to send drive commands to the robot.
    * @param twist Velocity command for the robot.
    */
    void sendCommand(TwistCmd twist)
    {
      float v_left, v_right;
      // Transform robot velocity to motor velocities.
      v_left = twist.vel - twist.ang;
      v_right = twist.vel + twist.ang;

      // Limit velocities.
      v_left = min(1.0, max(-1.0, v_left));
      v_right = min(1.0, max(-1.0, v_right));

      // Convert velocities to motor commands.
      commandMotors(v_left, v_right);
    }

    /**
     * @brief Method to convert motor velocities to motor commands.
     * @param v_left  Left motor velocity, ranges from -1.0 to 1.0.
     * @param v_right Right motor velocity, ranges from -1.0 to 1.0.
     */
    void commandMotors(float v_left, float v_right)
    {
      // Map left motor.
      if (v_left >= 0)
      {
        left_.sign = LOW;
        left_.duty = MAX_DUTY * v_left;
      }
      else
      {
        left_.sign = HIGH;
        left_.duty = MAX_DUTY * (1 + v_left);
      }

      // Map right motor.
      if (v_right >= 0)
      {
        right_.sign = LOW;
        right_.duty = MAX_DUTY * v_right;
      }
      else
      {
        right_.sign = HIGH;
        right_.duty = MAX_DUTY * (1 + v_right);
      }

      // Update motors.
      digitalWrite(left_.sign_pin, left_.sign);
      analogWrite(left_.pwm_pin, left_.duty);
      digitalWrite(right_.sign_pin, right_.sign);
      analogWrite(right_.pwm_pin, right_.duty);
    }
    MotorCmd left_, right_;  /// Objects to hold the commands for the left and right motors.
    volatile boolean * is_ang_rate_ready_;  /// Flag to check if a new angular rate command is available.
    volatile boolean * is_lin_rate_ready_;  /// Flag to check if a new linear rate command is available.
    volatile unsigned int * ang_rate_micros_;  /// Angular rate command from rc receiver in microseconds.
    volatile unsigned int * lin_rate_micros_;  /// Linear rate command from rc receiver in microseconds.
};

// Drive command for controlling the robot.
DriveCmd* robot;

void setup() {
  // Set LED pin for arduino indicator.
  pinMode(LED_PIN, OUTPUT);

  // Set up motor control pins.
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DO_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DO_PIN, OUTPUT);

  // Set input pins from receiver.
  pinMode(ANGULAR_RATE_PIN, INPUT);
  pinMode(LINEAR_RATE_PIN, INPUT);

  // Set baud for debug port.
  Serial.begin(9600);

  // Create interrupts to measure received PWM.
  attachInterrupt(digitalPinToInterrupt(ANGULAR_RATE_PIN),
                  ANG_RATE_ISR,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LINEAR_RATE_PIN),
                  LIN_RATE_ISR,
                  CHANGE);

  robot = new DriveCmd(M2_PWM_PIN, M2_DO_PIN, M1_PWM_PIN, M1_DO_PIN);
  robot->attachRCPointers(&bARCmdIsReady, &uiARCmd, &bLRCmdIsReady, &uiLRCmd);

  // Signal robot initiated.
  digitalWrite(LED_PIN, HIGH);
}


void loop() {
  // put your main code here, to run repeatedly:
  robot->update();
}
