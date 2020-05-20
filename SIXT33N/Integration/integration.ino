/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_3
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.3975;
float theta_right = 0.3609;
float beta_left = -16.85;
float beta_right = -20.08;
float v_star = 78.8;

// PWM inputs to jolt the car straight
int left_jolt = 165;
int right_jolt= 200;

// Control gains
float k_left = 0.7;
float k_right = 0.7;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (v_star + beta_left - k_left*delta)/ theta_left;
}

float driveStraight_right(float delta) {
  return (v_star + beta_right + k_right*delta)/ theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 9;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 1500, 2500, 1500};

float delta_reference(int k) {
  // YOUR CODE HERE
  float delta = (CAR_WIDTH*v_star/5*k)/TURN_RADIUS;
  if (drive_mode == DRIVE_RIGHT) {
    return delta;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -delta;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_4

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                  130
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define KMEANS_THRESHOLD            .06            
#define LOUDNESS_THRESHOLD          600

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[130] = {0.0209264380898, 0.0285890756235, 0.0337535017905, 0.0253297960253, 0.00199136114908, -0.0106479138388, -0.0270110168093, -0.0239379143903, -0.0233171640712, 0.0082187137926, 0.0355248519683, 0.0655229050032, 0.0815172270351, 0.087477238695, 0.10892612627, 0.128776437902, 0.13657455943, 0.148692077338, 0.16140750855, 0.154772756061, 0.173204529738, 0.188581419753, 0.20647553719, 0.230509232017, 0.242770033891, 0.239535752775, 0.242903481257, 0.233486733585, 0.219802089322, 0.201424414072, 0.192955395223, 0.161848950954, 0.140056857034, 0.12043565376, 0.0844208007167, 0.0627973599224, 0.042340013763, 0.0280228020806, 0.00486978263431, -0.0330642387563, -0.0526082886484, -0.0538371958873, -0.0829078883502, -0.0795577717668, -0.083565021683, -0.0766929583815, -0.0864735942664, -0.0862899194357, -0.0793863859405, -0.0935225378869, -0.102987548026, -0.0990033243867, -0.0921358124375, -0.0842073343211, -0.0814338962727, -0.0790842416312, -0.0714608323061, -0.0621297964713, -0.0483695677557, -0.0380622033946, -0.037348087004, -0.0293168510849, -0.0295992913259, -0.0241291027898, -0.0171316554281, -0.0163850994692, -0.00960604671367, -0.00743921164339, -0.00919348038197, -0.0144301591583, -0.0227963916905, -0.0360395460146, -0.0500449237705, -0.0598942435536, -0.0692678341334, -0.0735190273099, -0.0773032807732, -0.0808488325637, -0.0818180424996, -0.0850222893283, -0.0870434831535, -0.0842758896973, -0.0842059983129, -0.0838119002558, -0.0795512303862, -0.0762442889064, -0.0713920401034, -0.0720750677951, -0.0712124859597, -0.0655555416588, -0.0604333700419, -0.0557280753149, -0.0526104013658, -0.0475704918742, -0.0422081732057, -0.0398051321125, -0.0372555023466, -0.0337300094529, -0.0285650478875, -0.0264237049177, -0.0229226703425, -0.0206328874976, -0.0172525984287, -0.014758588272, -0.0149390444419, -0.016381039909, -0.0160730345627, -0.0154042155946, -0.0171397364715, -0.022339575911, -0.0220658141968, -0.0217224910022, -0.0225317152642, -0.0258011001549, -0.0257880255391, -0.025407501499, -0.0273294531064, -0.0236194428735, -0.021661284407, -0.021010463598, -0.0186326793076, -0.0171861393395, -0.0149738702706, -0.0150803742756, -0.0151767155646, -0.0148854254559, -0.0146466505069, -0.0133087122161, -0.0109841393391, -0.0122604265616};
float pca_vec2[130] = {0.0207213236963, 0.0245939469947, 0.0309214863319, 0.0393150180876, 0.0746176315979, 0.129759915013, 0.146156235965, 0.164467520453, 0.180878897308, 0.205102850589, 0.214193088359, 0.226142742584, 0.220697884303, 0.207651183783, 0.203276692331, 0.193408228341, 0.161232600842, 0.164443207568, 0.149687739806, 0.102963565982, 0.0840617705299, 0.0438824460475, -0.0221964529906, -0.0555171615293, -0.0571773662315, -0.0720578889079, -0.0685171222883, -0.0766972047649, -0.0994069461465, -0.0961808738185, -0.142553652635, -0.156000339393, -0.159603281623, -0.148665066739, -0.151647339183, -0.173637671822, -0.176579197965, -0.15755980537, -0.159972071517, -0.157201657422, -0.148339491577, -0.108508279944, -0.115097267058, -0.110790463691, -0.0998342854471, -0.0840536355726, -0.0768786444268, -0.0598671410187, -0.0549890011393, -0.0514027387024, -0.0648351850675, -0.0613589014687, -0.0542436141187, -0.0411463931256, -0.044439430962, -0.0476971092321, -0.0483378326061, -0.0514340054761, -0.0584494220406, -0.0618243467261, -0.0492511107175, -0.0525470385475, -0.0723420107661, -0.0619868976583, -0.0482857773058, -0.0484495075308, -0.0308729144133, -0.0308214152211, -0.0241780211015, -0.0233324820078, -0.0103343792913, 0.0100977486528, 0.0260188475114, 0.0320187649124, 0.0402855423321, 0.0468550068739, 0.0464033957894, 0.052405071279, 0.0535819669552, 0.0561696605281, 0.0625850158991, 0.0587285461918, 0.0559029063304, 0.0534887277295, 0.0514040719918, 0.0491524648325, 0.0484930915161, 0.0485544831534, 0.044967773979, 0.0423581158248, 0.0325763133211, 0.0301148177875, 0.02417981057, 0.029046400086, 0.0257826125319, 0.0254065293778, 0.026834574402, 0.0224447135325, 0.0185032187474, 0.0177631661362, 0.0130282657976, 0.0151235644734, 0.0110472094392, 0.00772582169774, 0.00624687265426, 0.00581044831555, 0.00304110570535, 0.00283202749309, -0.000600980985392, -0.00243509621327, -0.00672463452167, -0.00362739624182, -0.0070723081056, -0.00780177017013, -0.0111428163102, -0.0120832588438, -0.0133367955245, -0.0132289273464, -0.0103654995586, -0.011051903978, -0.00918210318043, -0.00984297251028, -0.00744437795235, -0.00582403920611, -0.00609969718711, -0.00473937481612, -0.00260648528294, -0.00215854751063, -0.00582906991006, -0.00485675120034};
float projected_mean_vec[2] = {0.0317883799486, 0.020217390534};
float centroid1[2] = {0.0620438527193, -0.00657159855303};
float centroid2[2] = {-0.0256892619825, -0.0055353214533};
float centroid3[2] = {-0.0305247443876, 0.0250517219304};
float centroid4[2] = {-0.00582984634914, -0.0129448019241};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      for (int i = 0; i < 4; i++) {
        float dist = l2_norm(proj1, proj2, centroids[i]);
          if(dist < best_dist){
              best_index = i;
              best_dist = dist;
          }
      }


      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (best_dist < KMEANS_THRESHOLD) {
        drive_mode = best_index; // from 0-3, inclusive
        Serial.println(best_index);
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
